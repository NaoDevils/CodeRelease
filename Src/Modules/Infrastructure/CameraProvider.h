/**
* @file CameraProvider.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Platform/Camera.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Tools/Module/Module.h"

class NaoCamera;

MODULE(CameraProvider,
{,
  USES(CameraIntrinsicsNext),
  USES(CameraResolutionRequest),
  REQUIRES(Image),
  PROVIDES_WITHOUT_MODIFY(Image),
  PROVIDES_WITHOUT_MODIFY(ImageUpper),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraInfoUpper),
  PROVIDES_WITHOUT_MODIFY(CameraSettings),
  PROVIDES_WITHOUT_MODIFY(CameraSettingsUpper),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraResolution),
});

class CameraProvider : public CameraProviderBase
{
private:
  static PROCESS_LOCAL CameraProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* upperCamera = nullptr;
  NaoCamera* lowerCamera = nullptr;
  CameraInfo lowerCameraInfo;
  CameraInfoUpper upperCameraInfo;
  CameraSettings lowerCameraSettings;
  CameraSettingsUpper upperCameraSettings;
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

public:
  /**
  * Default constructor.
  */
  CameraProvider();

  /**
  * Destructor.
  */
  ~CameraProvider();

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
  void update(CameraSettings& cameraSettings);
  void update(CameraSettingsUpper& cameraSettingsUpper);
  void update(CameraIntrinsics& cameraIntrinsics);
  void update(CameraResolution& cameraResolution);

  bool readCameraSettings();
  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCameras();
};
