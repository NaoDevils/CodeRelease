/**
* @file CameraProviderV6.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Platform/CameraV6.h"
#include "Representations/Infrastructure/CameraSettingsV6.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Tools/Module/Module.h"

class NaoCameraV6;

MODULE(CameraProviderV6,
{,
  USES(CameraIntrinsicsNext),
  USES(CameraResolutionRequest),
  REQUIRES(Image),
  PROVIDES_WITHOUT_MODIFY(Image),
  PROVIDES_WITHOUT_MODIFY(ImageUpper),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraInfoUpper),
  PROVIDES_WITHOUT_MODIFY(CameraSettingsV6),
  PROVIDES_WITHOUT_MODIFY(CameraSettingsUpperV6),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraResolution),
});

class CameraProviderV6 : public CameraProviderV6Base
{
private:
  static PROCESS_LOCAL CameraProviderV6* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCameraV6* upperCamera = nullptr;
  NaoCameraV6* lowerCamera = nullptr;
  CameraInfo lowerCameraInfo;
  CameraInfoUpper upperCameraInfo;
  CameraSettingsV6 lowerCameraSettings;
  CameraSettingsUpperV6 upperCameraSettings;
  CameraIntrinsics cameraIntrinsics;
  CameraResolution cameraResolution;
  float cycleTime;
#ifdef CAMERA_INCLUDED
  unsigned int imageTimeStamp;
  unsigned int imageTimeStampUpper;
  unsigned int lastImageTimeStamp;
  unsigned int lastImageUpperTimeStamp;
  unsigned long long lastImageTimeStampLL;
  unsigned long long lastImageTimeStampLLUpper;
#endif
  unsigned resetCounter = 0;

public:
  /**
  * Default constructor.
  */
  CameraProviderV6();

  /**
  * Destructor.
  */
  ~CameraProviderV6();

  /**
  * The method returns whether a new image is available.
  * @return Is an new image available?
  */
  static bool isFrameDataComplete();

  /**
  * The method waits for a new image.
  */
  static void waitForFrameData();
  void waitForFrameData2();

  void setUpCameras();

private:
  void update(Image& image);
  void update(ImageUpper& imageUpper);
  void update(FrameInfo& frameInfo);
  void update(CameraInfo& cameraInfo);
  void update(CameraInfoUpper& cameraInfoUpper);
  void update(CameraSettingsV6& cameraSettings);
  void update(CameraSettingsUpperV6& cameraSettingsUpper);
  void update(CameraIntrinsics& cameraIntrinsics);
  void update(CameraResolution& cameraResolution);

  bool readCameraSettings();
  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCameras();
};
