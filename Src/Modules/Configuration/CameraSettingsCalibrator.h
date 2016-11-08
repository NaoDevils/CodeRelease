/**
* @file CameraSettingsCalibrator.h
*
* A module that optimizes the camera settings.
* Currently only calibrates on exposure and gain
* since the cameras auto exposure algorithms are not sufficient.
*
* @author Ingmar Schwarz
*/

#pragma once

#include "Tools/RingBufferWithSum.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"

STREAMABLE(Constraint,
{ ,
  (int) maxVal,
  (int) minVal,
});

MODULE(CameraSettingsCalibrator,
{ ,
  USES(CameraMatrix),
  USES(CameraMatrixUpper),
  USES(FrameInfo),
  USES(Image),
  USES(ImageUpper),
  USES(FieldColors),
  USES(FieldColorsUpper),
  PROVIDES(CameraSettings),
  PROVIDES(CameraSettingsUpper),
  LOADS_PARAMETERS(
  {,
    (bool) calibrateLower,
    (bool) calibrateUpper,
    (Constraint) exposureConstraints,
    (Constraint) gainConstraints,
    (int) timeForSettingUpdate,
    (int) changeGainAtExposure,
    (int) minBrightnessAverage,
    (int) maxBrightnessAverage,
    (int) hysteresis,
    (int) gainChange,
    (int) exposureChange,
  }),
});



class CameraSettingsCalibrator : public CameraSettingsCalibratorBase
{
public:

  CameraSettingsCalibrator();
private:
  void execute();
  void calibrateCamera(const bool &upper);
  void update(CameraSettings &cameraSettings);
  void update(CameraSettingsUpper &cameraSettingsUpper);
  bool readCameraSettings(bool upper); /**< Called once at start for each camera. */
  void setToAuto(bool upper, bool all); /**< Set to default auto settings. Done automatically if camera settings could not be read. */
  void enforceBounds(bool upper); /**< To enforce bounds of camera settings, avoiding impossible settings. */

  unsigned lastTimeStampExecuted;
  RingBufferWithSum<int, 10> sceneBrightnessLower; /**< Every frame brightness is added here. At the moment fieldColor represents scene brightness. */
  RingBufferWithSum<int, 10> sceneBrightnessUpper; /**< Every frame brightness is added here. At the moment fieldColor represents scene brightness. */
  unsigned timeStampStart; /**< Start time stamp of module, auto exposure is delayed to make sure all USED reps are available. */
  bool initializedLower; /**< If false, lower camera settings were not read from file. */
  bool initializedUpper; /**< If false, upper camera settings were not read from file. */
  bool wasTooDarkLower; /**< If false, last change was from brighter settings to darker in lower image (for hysteresis). */
  bool wasTooDarkUpper; /**< If false, last change was from brighter settings to darker in upper image (for hysteresis). */
  bool switchedToAutoLower; /**< If true, switch to original auto exposure was done due to dark lower image. */
  bool switchedToAutoUpper; /**< If true, switch to original auto exposure was done due to dark upper image. */
  unsigned lastSetTimeStampLower; /**< Time stamp of last setting change for lower camera settings. */
  unsigned lastSetTimeStampUpper; /**< Time stamp of last setting change for upper camera settings. */
  CameraSettings localCameraSettings; /**< Local copy of lower camera settings. */
  CameraSettingsUpper localCameraSettingsUpper; /**< Local copy of upper camera settings. */
};