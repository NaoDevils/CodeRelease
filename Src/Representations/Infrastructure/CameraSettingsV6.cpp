/**
 * @file CameraSettings.cpp
 * Implementation of struct CameraSettings.
 */

#include "CameraSettingsV6.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

#include <limits>

#ifdef CAMERA_INCLUDED
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#define __STRICT_ANSI__

#include "Representations/Infrastructure/CameraRegisters.h"
#endif

CameraSettingsV6::V4L2Setting::V4L2Setting() :
  V4L2Setting(0, 0, std::numeric_limits<int>::min(), std::numeric_limits<int>::max())
{}

CameraSettingsV6::V4L2Setting::V4L2Setting(int command, int value, int min, int max) :
  command(command), min(min), max(max), value(value)
{
  ASSERT(min <= max);
  for(CameraSetting& influenced : influencingSettings)
    influenced = numOfCameraSettings;
}

CameraSettingsV6::V4L2Setting& CameraSettingsV6::V4L2Setting::operator=(const V4L2Setting& other)
{
  if (this == &other)
    return *this;
  command = other.command;
  value = other.value;
  min = other.min;
  max = other.max;
  influencingSettings[0] = other.influencingSettings[0];
  influencingSettings[1] = other.influencingSettings[1];
  return *this;
}

CameraSettingsV6& CameraSettingsV6::operator=(const CameraSettingsV6& other)
{
  if (this == &other)
  {
    return *this;
  }
  for (int i = 0; i < numOfCameraSettings; ++i)
  {
	  settings[i] = other.settings[i];
  }
  
  windowPosition = other.windowPosition;
  windowSize = other.windowSize;
  windowWeights = other.windowWeights;
  registers = other.registers;
  return *this;
}

bool CameraSettingsV6::V4L2Setting::operator==(const V4L2Setting& other) const
{
  return command == other.command && value == other.value;
}

bool CameraSettingsV6::V4L2Setting::operator!=(const V4L2Setting& other) const
{
  return !(*this == other);
}

void CameraSettingsV6::V4L2Setting::enforceBounds()
{
  if(value < min)
    value = min;
  else if(value > max)
    value = max;
}

CameraSettingsV6::CameraSettingsV6()
#ifdef CAMERA_INCLUDED
{
  settings[AutoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, 0, 0, 3); //Edit
  settings[AutoExposure].influencingSettings[0] = Exposure;
  settings[AutoExposure].influencingSettings[1] = Gain;
  //settings[AutoExposureAlgorithm] = V4L2Setting(V4L2_CID_EXPOSURE_ALGORITHM, 0, 0, 3); //Edit
  //settings[BacklightCompensation] = V4L2Setting(V4L2_CID_BACKLIGHT_COMPENSATION, 1, 0, 4);
  settings[AutoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, 1, 0, 1); //Unchanged
  settings[AutoWhiteBalance].influencingSettings[0] = WhiteBalance; //Unchanged
  settings[Contrast] = V4L2Setting(V4L2_CID_CONTRAST, 32, 0, 255); //Edit
  settings[Exposure] = V4L2Setting(V4L2_CID_EXPOSURE_ABSOLUTE, 512, 0, 1048575); //Edit. Interpreted as exposure_absolute
  //settings[FadeToBlack] = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, 0, 0, 1);
  settings[Gain] = V4L2Setting(V4L2_CID_GAIN, 32, 0, 1023); //Edit
  settings[Hue] = V4L2Setting(V4L2_CID_HUE, 0, -180, 180); //Edit
  settings[Saturation] = V4L2Setting(V4L2_CID_SATURATION, 64, 0, 255); //Edit
  settings[Sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, 5, 0, 9); //Edit
  settings[WhiteBalance] = V4L2Setting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, 0, 2500, 6500); //Edit
  //settings[Gamma] = V4L2Setting(V4L2_CID_GAMMA, 220, 100, 280);
  //settings[PowerLineFrequency] = V4L2Setting(V4L2_CID_POWER_LINE_FREQUENCY, 1, 1, 2);
  //settings[TargetAverageLuma] = V4L2Setting(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA, 55, 0, 255);
  //settings[TargetAverageLumaDark] = V4L2Setting(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA_DARK, 27, 0, 255);
  //settings[TargetGain] = V4L2Setting(V4L2_MT9M114_AE_TARGET_GAIN, 128, 0, 65535);
  //settings[MinGain] = V4L2Setting(V4L2_MT9M114_AE_MIN_VIRT_GAIN, 32, 0, 65535);
  //settings[MaxGain] = V4L2Setting(V4L2_MT9M114_AE_MAX_VIRT_GAIN, 256, 0, 65535);
  //settings[AeMode] = V4L2Setting(V4L2_MT9M114_AE_MODE, 0, 0, 255);
  settings[Focus] = V4L2Setting(V4L2_CID_FOCUS_ABSOLUTE, 0, 0, 250); //New
  settings[AutoFocus] = V4L2Setting(V4L2_CID_FOCUS_AUTO, 0, 0, 1); //New
  settings[AutoFocus].influencingSettings[0] = Focus; //Unchanged

  windowPosition = Vector2s(0, 0);
  windowSize = Vector2s(1280, 960);
  windowWeights.fill(1);
}
#else
{}
#endif

bool CameraSettingsV6::operator==(const CameraSettingsV6& other) const
{
  for(int i = 0; i < numOfCameraSettings; ++i)
  {
    if(settings[i] != other.settings[i])
      return false;
  }
  return true;
}

bool CameraSettingsV6::operator!=(const CameraSettingsV6& other) const
{
  return !(*this == other);
}

void CameraSettingsV6::enforceBounds()
{
  for(int i = 0; i < numOfCameraSettings; ++i)
    settings[i].enforceBounds();
}

void CameraSettingsV6::serialize(In* in, Out* out)
{
  V4L2Setting& autoExposure = settings[AutoExposure];
  //V4L2Setting& autoExposureAlgorithm = settings[AutoExposureAlgorithm];
  //V4L2Setting& backlightCompensation = settings[BacklightCompensation];
  V4L2Setting& autoWhiteBalance = settings[AutoWhiteBalance];
  V4L2Setting& contrast = settings[Contrast];
  V4L2Setting& exposure = settings[Exposure];
  //V4L2Setting& fadeToBlack = settings[FadeToBlack];
  V4L2Setting& gain = settings[Gain];
  V4L2Setting& hue = settings[Hue];
  V4L2Setting& saturation = settings[Saturation];
  V4L2Setting& sharpness = settings[Sharpness];
  V4L2Setting& whiteBalance = settings[WhiteBalance];
  //V4L2Setting& gamma = settings[Gamma];
  //V4L2Setting& powerLineFrequency = settings[PowerLineFrequency];
  //V4L2Setting& targetAverageLuma = settings[TargetAverageLuma];
  //V4L2Setting& targetAverageLumaDark = settings[TargetAverageLumaDark];
  //V4L2Setting& targetGain = settings[TargetGain];
  //V4L2Setting& minGain = settings[MinGain];
  //V4L2Setting& maxGain = settings[MaxGain];
  //V4L2Setting& aeMode = settings[AeMode];
  V4L2Setting& focusAbsolute = settings[Focus]; //New
  V4L2Setting& autoFocus = settings[AutoFocus]; //New

  STREAM_REGISTER_BEGIN;
  STREAM(autoExposure);
  //STREAM(autoExposureAlgorithm);
  //STREAM(backlightCompensation);
  STREAM(autoWhiteBalance);
  STREAM(contrast);
  STREAM(exposure);
  //STREAM(fadeToBlack);
  STREAM(gain);
  STREAM(hue);
  STREAM(saturation);
  STREAM(sharpness);
  STREAM(whiteBalance);
  //STREAM(gamma);
  //STREAM(powerLineFrequency);
  //STREAM(targetAverageLuma);
  //STREAM(targetAverageLumaDark);
  //STREAM(targetGain);
  //STREAM(minGain);
  //STREAM(maxGain);
  //STREAM(aeMode);
  STREAM(windowPosition);
  STREAM(windowSize);
  STREAM(windowWeights);
  STREAM(focusAbsolute); //New
  STREAM(autoFocus); //New
  STREAM(registers);
  STREAM_REGISTER_FINISH;

  if(in)
    enforceBounds();
}
