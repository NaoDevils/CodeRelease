/**
 * @file CameraSettingsV6.h
 * Declaration of a struct representing the settings of the PDA camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include <array>

struct CameraSettingsV6 : public Streamable
{
  ENUM(CameraSetting,
  {,
    AutoFocus, /*1: Enable continuous automatic focus adjustments, 0: Disable auto focus*/ //New
    AutoExposure, /* 1: Use auto exposure, 0: disable auto exposure. */
    // AutoExposureAlgorithm,
    //BacklightCompensation, /* 0 - 4 */
    AutoWhiteBalance, /* 1: Use auto white balance, 0: disable auto white balance. */
    Contrast,   /* The contrast in range of [16 .. 64]. Gravients from 0.5 (16) to 2.0 (64).*/
    Exposure, /**< The exposure time in the range of [0 .. 1000]. Time is measured in increments of 100µs. */
    //FadeToBlack, /**< Fade to black under low light conditions. 1: enabled, 0: disabled. */
    Gain, /**< The gain level in the range of [0 .. 255]. */
    Hue, /* The hue in range [-22 .. 22] */
    Saturation, /* The saturation in range of [0 .. 255] */
    Sharpness, /* The sharpness in range of [-7 .. 7] */
    WhiteBalance, /**< The white balance in Kelvin [2700 .. 6500] */
    //Gamma, /* The gamma value in range [100, 280] */
    //PowerLineFrequency, /* The local power frequency (1 = 50Hz, 2 = 60Hz) */
    //TargetAverageLuma, /* The target average brightness [0 .. 255] */
    //TargetAverageLumaDark, /* The target average brightness for dark [0 .. 255] */
    //TargetGain, /* The target analog gain [0 .. 65535] */
    //MinGain, /* The minimum value for the analog gain that AE Track is permitted to use [0 .. 65535] */
    //MaxGain, /* The maximum value for the analog gain that AE Track is permitted to use [0 .. 65535] */
    //AeMode, /* AE mode (indoor, ...) [0 .. 255] */
    Focus, /*This control sets the focal point of the camera to the specified position in [0,250], in 25 step*/ //New
  });


  STREAMABLE(CameraRegister,
  {
    ENUM(RegisterName,
    { ,
      aec_ctrl,
      aec_ctrl_stable_high,
      aec_ctrl_stable_low,
      aec_ctrl_unstable_high,
      aec_ctrl_unstable_low,
      aec_min_exposure,
      aec_max_expo_60hz,
      aec_max_expo_50hz,
      aec_gain_ceiling,
      aec_5060hz_ctrl0,
      aec_5060hz_ctrl1,
      aec_60hz_max_bands,
      aec_50hz_max_bands,
      timing_tc_reg21,
    });

    uint16_t getAddress() const
    {
      switch (reg)
      {
      case RegisterName::aec_ctrl: return 0x3A00;
      case RegisterName::aec_ctrl_stable_high: return 0x3A0F;
      case RegisterName::aec_ctrl_stable_low: return 0x3A10;
      case RegisterName::aec_ctrl_unstable_high: return 0x3A1B;
      case RegisterName::aec_ctrl_unstable_low: return 0x3A1E;
      case RegisterName::aec_min_exposure: return 0x3A01;
      case RegisterName::aec_max_expo_60hz: return 0x3A02;
      case RegisterName::aec_max_expo_50hz: return 0x3A14;
      case RegisterName::aec_gain_ceiling: return 0x3A18;
      case RegisterName::aec_5060hz_ctrl0: return 0x3C00;
      case RegisterName::aec_5060hz_ctrl1: return 0x3C01;
      case RegisterName::aec_60hz_max_bands: return 0x3A0D;
      case RegisterName::aec_50hz_max_bands: return 0x3A0E;
      case RegisterName::timing_tc_reg21: return 0x3821;
      default: return 0;
      }
    }

    uint8_t getSize() const
    {
      switch (reg)
      {
      case RegisterName::aec_max_expo_60hz: return 2;
      case RegisterName::aec_max_expo_50hz: return 2;
      case RegisterName::aec_gain_ceiling: return 2;
      default: return 1;
      }
    }

    bool operator==(const CameraRegister& other) const
    {
      return this->reg == other.reg && this->value == other.value;
    }
    ,

    (RegisterName)(0) reg,
    (int)(0) value,
  });

  STREAMABLE(V4L2Setting,
  {
    int command;

  private:
    int min;
    int max;

  public:
    PROTECT(std::array<CameraSetting, 2>) influencingSettings;

    V4L2Setting();
    V4L2Setting(int command, int value, int min, int max);
    virtual ~V4L2Setting() = default;

    V4L2Setting& operator=(const V4L2Setting& other);
    bool operator==(const V4L2Setting& other) const;
    bool operator!=(const V4L2Setting& other) const;

    void enforceBounds(),

    (int) value,
  });

  std::array<V4L2Setting, numOfCameraSettings> settings;
  std::vector<CameraRegister> registers;

  Vector2s windowPosition;
  Vector2s windowSize;
  std::array<uint8_t,16> windowWeights;

  CameraSettingsV6();
  virtual ~CameraSettingsV6() = default;

  CameraSettingsV6& operator=(const CameraSettingsV6& other);
  bool operator==(const CameraSettingsV6& other) const;
  bool operator!=(const CameraSettingsV6& other) const;

  void enforceBounds();

  virtual void serialize(In* in, Out* out);
};

STREAMABLE_WITH_BASE(CameraSettingsUpperV6, CameraSettingsV6,
{,

});
