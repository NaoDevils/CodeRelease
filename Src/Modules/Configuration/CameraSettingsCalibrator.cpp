
#include <cstdio>
#include "Tools/Settings.h"
#include "CameraSettingsCalibrator.h"

CameraSettingsCalibrator::CameraSettingsCalibrator()
{
  initializedLower = false;
  initializedUpper = false;
  lastSetTimeStampLower = 0;
  lastSetTimeStampUpper = 0;
  timeStampStart = 0;
  lastTimeStampExecuted = 0;
  localCameraSettings = CameraSettings();
  localCameraSettingsUpper = CameraSettingsUpper();
  sceneBrightnessLower.fill(200);
  sceneBrightnessUpper.fill(200);
  wasTooDarkLower = false;
  wasTooDarkUpper = false;
  switchedToAutoLower = false; // true -> this module is switched off
  switchedToAutoUpper = false; // true -> this module is switched off
};

void CameraSettingsCalibrator::execute()
{
  if (!initializedLower)
  {
    if (!readCameraSettings(false))
      setToAuto(false,true);
    initializedLower = true;
  }
  if (!initializedUpper)
  {
    if (!readCameraSettings(true))
      setToAuto(true,true);
    initializedUpper = true;
  }

  if (!Blackboard::getInstance().exists("FrameInfo") || theFrameInfo.time == 0) // no useful timestamps yet
  {
    return;
  }
  else if (timeStampStart == 0)
  {
    timeStampStart = theFrameInfo.time;
  }
  if (theFrameInfo.getTimeSince(timeStampStart) < 10000)
  {
    lastSetTimeStampLower = theFrameInfo.time;
    lastSetTimeStampUpper = theFrameInfo.time;
    return;
  }

  if (lastTimeStampExecuted != theFrameInfo.time)
  { 
    lastTimeStampExecuted = theFrameInfo.time;
    if (initializedLower && calibrateLower && theCameraMatrix.isValid)
      calibrateCamera(false);
    if (initializedUpper && calibrateUpper && theCameraMatrixUpper.isValid)
      calibrateCamera(true);
  }
}

void CameraSettingsCalibrator::calibrateCamera(const bool &upper)
{
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  CameraSettings &settings = upper ? (CameraSettings&)localCameraSettingsUpper : localCameraSettings;
  unsigned &lastSetTimeStamp = upper ? lastSetTimeStampUpper : lastSetTimeStampLower;
  bool &switchedToAuto = upper ? switchedToAutoUpper : switchedToAutoLower;
  RingBufferWithSum<int, 10> &sceneBrightness = upper ? sceneBrightnessUpper : sceneBrightnessLower;
  bool &wasTooDark = upper ? wasTooDarkUpper : wasTooDarkLower;

  sceneBrightness.push_front(fieldColor.fieldColorArray[0].fieldColorOptY);
  if (theFrameInfo.getTimeSince(lastSetTimeStamp) > timeForSettingUpdate)
  {
    lastSetTimeStamp = theFrameInfo.time;
    if (settings.settings[CameraSettings::AutoExposure].value != 0)
    { // check if auto settings are sufficient
      if (switchedToAuto) // do not switch back from auto settings
        return;
      if (settings.settings[CameraSettings::Exposure].value > exposureConstraints.maxVal
        && sceneBrightness.average() > maxBrightnessAverage) 
      { // if the image is still dark, keep auto settings -> TODO
        settings.settings[CameraSettings::AutoExposure].value = 0;
        settings.settings[CameraSettings::Exposure].value = exposureConstraints.maxVal;
        settings.settings[CameraSettings::Gain].value = gainConstraints.minVal;
      }
    }
    else // manual settings, update and check if switching to auto is neccessary
    {
      int avgBrightness = sceneBrightness.average();
      // Case: scene too dark.
      if (avgBrightness < (minBrightnessAverage + (wasTooDark ? hysteresis : 0)))
      {
        wasTooDark = true;
        if (settings.settings[CameraSettings::Exposure].value < exposureConstraints.maxVal)
        {
          settings.settings[CameraSettings::Exposure].value = // TODO: constants/equation below to params
            std::min(settings.settings[CameraSettings::Exposure].value + exposureChange + (minBrightnessAverage - avgBrightness), exposureConstraints.maxVal);
        }
        else if (settings.settings[CameraSettings::Gain].value < gainConstraints.maxVal)
        {
          settings.settings[CameraSettings::Gain].value = 
            std::min(settings.settings[CameraSettings::Gain].value + gainChange, gainConstraints.maxVal);
        }
        else // TODO: use auto at all?
        {
          setToAuto(false,false);
          switchedToAuto = true;
        }
      }
      // Case: scene too bright
      else if (avgBrightness > (maxBrightnessAverage + (wasTooDark ? 0 : hysteresis))) 
      {
        wasTooDark = false;
        if (settings.settings[CameraSettings::Gain].value > gainConstraints.minVal)
        {
          settings.settings[CameraSettings::Gain].value =
            std::max(settings.settings[CameraSettings::Gain].value - gainChange, gainConstraints.minVal);
        }
        else if (settings.settings[CameraSettings::Exposure].value > exposureConstraints.minVal)
        {
          settings.settings[CameraSettings::Exposure].value =
            std::max(settings.settings[CameraSettings::Exposure].value - exposureChange, exposureConstraints.minVal);
        }
      }
    }
  }
}

void CameraSettingsCalibrator::update(CameraSettings &cameraSettings)
{
  DEBUG_RESPONSE_ONCE("module:CameraCalibrator:LoadCameraSettings") readCameraSettings(false);
  execute();
  cameraSettings = localCameraSettings;
}

void CameraSettingsCalibrator::update(CameraSettingsUpper &cameraSettingsUpper)
{
  DEBUG_RESPONSE_ONCE("module:CameraCalibrator:LoadCameraSettingsUpper") readCameraSettings(true);
  execute();
  cameraSettingsUpper = localCameraSettingsUpper;
}

bool CameraSettingsCalibrator::readCameraSettings(bool upper)
{
  std::string name = upper ? "upperCameraSettings.cfg" : "lowerCameraSettings.cfg";
  InMapFile stream(name);
  if(stream.exists())
  {
    if (upper)
      stream >> localCameraSettingsUpper;
    else
      stream >> localCameraSettings;
  }
  else
  {
    return false;
  }

  return true;
}

void CameraSettingsCalibrator::setToAuto(bool upper, bool all)
{
  CameraSettings &settings = (upper ? (CameraSettings&)localCameraSettingsUpper : localCameraSettings);
  settings.settings[CameraSettings::AutoExposure].value = 1;
  if (all)
  {
    settings.settings[CameraSettings::AutoWhiteBalance].value = 1;
    settings.settings[CameraSettings::Contrast].value = 32;
    settings.settings[CameraSettings::FadeToBlack].value = 0;
    settings.settings[CameraSettings::Exposure].value = 50;
    settings.settings[CameraSettings::Hue].value = 0;
    settings.settings[CameraSettings::Sharpness].value = 2;
    settings.settings[CameraSettings::Saturation].value = 150;
    settings.settings[CameraSettings::Gain].value = 70;
    settings.settings[CameraSettings::WhiteBalance].value = 5000;
  }

  enforceBounds(upper);
}

void CameraSettingsCalibrator::enforceBounds(bool upper)
{
  CameraSettings &settings = (upper ? (CameraSettings&)localCameraSettingsUpper : localCameraSettings);
  settings.enforceBounds();
  //settings.enforceDependecies();  
}

MAKE_MODULE(CameraSettingsCalibrator, cognitionInfrastructure)
