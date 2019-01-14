/**
 * @file CameraSettings.cpp
 * Implementation of struct CameraSettings.
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

#include <limits>

#ifdef CAMERA_INCLUDED
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#define __STRICT_ANSI__

#include "Representations/Infrastructure/CameraRegisters.h"
#endif

CameraSettings::V4L2Setting::V4L2Setting() :
  V4L2Setting(0, 0, std::numeric_limits<int>::min(), std::numeric_limits<int>::max())
{}

CameraSettings::V4L2Setting::V4L2Setting(int command, int value, int min, int max) :
  command(command), min(min), max(max), value(value)
{
  ASSERT(min <= max);
  for(CameraSetting& influenced : influencingSettings)
    influenced = numOfCameraSettings;
}

CameraSettings::V4L2Setting& CameraSettings::V4L2Setting::operator=(const V4L2Setting& other)
{
  if (this == &other)
    return *this;
  command = other.command;
  value = other.value;
  max = other.max;
  min = other.min;
  influencingSettings[0] = other.influencingSettings[0];
  influencingSettings[1] = other.influencingSettings[1];
  return *this;
}

CameraSettings& CameraSettings::operator=(const CameraSettings& other)
{
  if (this == &other)
  {
    return *this;
  }
  for (int i = 0; i < numOfCameraSettings; ++i)
  {
	  settings[i] = other.settings[i];
  }
  for (int i = 0; i < 25; ++i)
  {
	  weights[i] = other.weights[i];
  }
  return *this;
}

bool CameraSettings::V4L2Setting::operator==(const V4L2Setting& other) const
{
  return command == other.command && value == other.value;
}

bool CameraSettings::V4L2Setting::operator!=(const V4L2Setting& other) const
{
  return !(*this == other);
}

void CameraSettings::V4L2Setting::enforceBounds()
{
  if(value < min)
    value = min;
  else if(value > max)
    value = max;
}

CameraSettings::CameraSettings()
#ifdef CAMERA_INCLUDED
{
  settings[AutoExposure] = V4L2Setting(V4L2_CID_EXPOSURE_AUTO, 1, 0, 3);
  settings[AutoExposure].influencingSettings[0] = Exposure;
  settings[AutoExposure].influencingSettings[1] = Gain;
  settings[AutoExposureAlgorithm] = V4L2Setting(V4L2_CID_EXPOSURE_ALGORITHM, 1,0,3);
  settings[BacklightCompensation] = V4L2Setting(V4L2_CID_BACKLIGHT_COMPENSATION, 1,0,4);
  settings[AutoWhiteBalance] = V4L2Setting(V4L2_CID_AUTO_WHITE_BALANCE, 1, 0, 1);
  settings[AutoWhiteBalance].influencingSettings[0] = WhiteBalance;
  settings[Contrast] = V4L2Setting(V4L2_CID_CONTRAST, 60, 16, 64);
  settings[Exposure] = V4L2Setting(V4L2_CID_EXPOSURE, 120, 0, 1000);
  settings[FadeToBlack] = V4L2Setting(V4L2_MT9M114_FADE_TO_BLACK, 0, 0, 1);
  settings[Gain] = V4L2Setting(V4L2_CID_GAIN, 100, 0, 255);
  settings[Hue] = V4L2Setting(V4L2_CID_HUE, 0, -22, 22);
  settings[Saturation] = V4L2Setting(V4L2_CID_SATURATION, 150, 0, 255);
  settings[Sharpness] = V4L2Setting(V4L2_CID_SHARPNESS, 2, -7, 7);
  settings[WhiteBalance] = V4L2Setting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, 5000, 2700, 6500);
  settings[Gamma] = V4L2Setting(V4L2_CID_GAMMA, 220, 100, 280);
  settings[PowerLineFrequency] = V4L2Setting(V4L2_CID_POWER_LINE_FREQUENCY, 1, 1, 2);
  settings[TargetAverageLuma] = V4L2Setting(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA, 55, 0, 255);
  settings[TargetAverageLumaDark] = V4L2Setting(V4L2_MT9M114_AE_TARGET_AVERAGE_LUMA_DARK, 27, 0, 255);
  settings[TargetGain] = V4L2Setting(V4L2_MT9M114_AE_TARGET_GAIN, 128, 0, 65535);
  settings[MinGain] = V4L2Setting(V4L2_MT9M114_AE_MIN_VIRT_GAIN, 32, 0, 65535);
  settings[MaxGain] = V4L2Setting(V4L2_MT9M114_AE_MAX_VIRT_GAIN, 256, 0, 65535);
  settings[AeMode] = V4L2Setting(V4L2_MT9M114_AE_MODE, 0, 0, 255);

  weights[ 0] = 25;  weights[ 1] = 25;  weights[ 2] = 25;  weights[ 3] = 25;  weights[ 4] = 25;
  weights[ 5] = 25;  weights[ 6] = 75;  weights[ 7] = 75;  weights[ 8] = 75;  weights[ 9] = 25;
  weights[10] = 25;  weights[11] = 75;  weights[12] =100;  weights[13] = 75;  weights[14] = 25;
  weights[15] = 25;  weights[16] = 75;  weights[17] = 75;  weights[18] = 75;  weights[19] = 25;
  weights[20] = 25;  weights[21] = 25;  weights[22] = 25;  weights[23] = 25;  weights[24] = 25;
}
#else
{}
#endif

bool CameraSettings::operator==(const CameraSettings& other) const
{
  for(int i = 0; i < numOfCameraSettings; ++i)
  {
    if(settings[i] != other.settings[i])
      return false;
  }
  return true;
}

bool CameraSettings::operator!=(const CameraSettings& other) const
{
  return !(*this == other);
}

void CameraSettings::enforceBounds()
{
  for(int i = 0; i < numOfCameraSettings; ++i)
    settings[i].enforceBounds();
}

void CameraSettings::serialize(In* in, Out* out)
{
  V4L2Setting& autoExposure = settings[AutoExposure];
  V4L2Setting& autoExposureAlgorithm = settings[AutoExposureAlgorithm];
  V4L2Setting& backlightCompensation = settings[BacklightCompensation];
  V4L2Setting& autoWhiteBalance = settings[AutoWhiteBalance];
  V4L2Setting& contrast = settings[Contrast];
  V4L2Setting& exposure = settings[Exposure];
  V4L2Setting& fadeToBlack = settings[FadeToBlack];
  V4L2Setting& gain = settings[Gain];
  V4L2Setting& hue = settings[Hue];
  V4L2Setting& saturation = settings[Saturation];
  V4L2Setting& sharpness = settings[Sharpness];
  V4L2Setting& whiteBalance = settings[WhiteBalance];
  V4L2Setting& gamma = settings[Gamma];
  V4L2Setting& powerLineFrequency = settings[PowerLineFrequency];
  V4L2Setting& targetAverageLuma = settings[TargetAverageLuma];
  V4L2Setting& targetAverageLumaDark = settings[TargetAverageLumaDark];
  V4L2Setting& targetGain = settings[TargetGain];
  V4L2Setting& minGain = settings[MinGain];
  V4L2Setting& maxGain = settings[MaxGain];
  V4L2Setting& aeMode = settings[AeMode];

  STREAM_REGISTER_BEGIN;
  STREAM(autoExposure);
  STREAM(autoExposureAlgorithm);
  STREAM(backlightCompensation);
  STREAM(autoWhiteBalance);
  STREAM(contrast);
  STREAM(exposure);
  STREAM(fadeToBlack);
  STREAM(gain);
  STREAM(hue);
  STREAM(saturation);
  STREAM(sharpness);
  STREAM(whiteBalance);
  STREAM(gamma);
  STREAM(powerLineFrequency);
  STREAM(targetAverageLuma);
  STREAM(targetAverageLumaDark);
  STREAM(targetGain);
  STREAM(minGain);
  STREAM(maxGain);
  STREAM(aeMode);
  STREAM(weights);
  STREAM_REGISTER_FINISH;

  if(in)
    enforceBounds();
}
