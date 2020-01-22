/**
* @file CameraProviderV6.cpp
* This file declares a module that provides camera images.
* @author Colin Graf
*/

#include <cstdio>

#include "CameraProviderV6.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Settings.h"
#include <chrono>
#include <thread>

PROCESS_LOCAL CameraProviderV6* CameraProviderV6::theInstance = 0;

CameraProviderV6::CameraProviderV6() :
  lowerCameraInfo(CameraInfo()), upperCameraInfo(CameraInfoUpper()),
  lowerCameraSettings(CameraSettingsV6()), upperCameraSettings(CameraSettingsUpperV6())
#ifdef CAMERA_INCLUDED
  , imageTimeStamp(0), imageTimeStampUpper(0), lastImageTimeStamp(0), lastImageUpperTimeStamp(0), lastImageTimeStampLL(0), lastImageTimeStampLLUpper(0)
#endif
{
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  setupCameras();
  VERIFY(readCameraSettings());

#ifdef CAMERA_INCLUDED
  upperCamera->setSettings(upperCameraSettings);
  lowerCamera->setSettings(lowerCameraSettings);
  upperCamera->writeCameraSettings();
  lowerCamera->writeCameraSettings();
#endif
  theInstance = this;
}

CameraProviderV6::~CameraProviderV6()
{
#ifdef CAMERA_INCLUDED
  if (upperCamera)
  {
    delete upperCamera;
    upperCamera = nullptr;
  }
  if (lowerCamera)
  {
    delete lowerCamera;
    lowerCamera = nullptr;
  }
#endif
  theInstance = nullptr;
}

void CameraProviderV6::update(Image& image)
{
  MODIFY_ONCE("representation:LowerCameraSettingsV6", lowerCameraSettings);
  //DEBUG_RESPONSE_ONCE("module:CameraProviderV6:LoadCameraSettings") readCameraSettings();
#ifdef CAMERA_INCLUDED
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:outputCurrentValuesLower") lowerCamera->outputCurrentValues();

  if (lowerCamera->hasImage())
  {
    image.setResolution(lowerCameraInfo.width, lowerCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(lowerCamera->getImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    if (Global::getSettings().naoVersion == RobotConfig::V6)
    {
      int steadyNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      imageTimeStamp = image.timeStamp = std::max((int)lastImageTimeStamp + 1,
        (int)SystemCall::getRealSystemTime() - (steadyNow - (int)(lowerCamera->getTimeStamp() / 1000)));
    }
    else
      imageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1,
        (unsigned)(lowerCamera->getTimeStamp() / 1000 - SystemCall::getSystemTimeBase()));
    lowerCamera->setSettings(lowerCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProviderV6:DoWhiteBalanceLower") lowerCamera->doAutoWhiteBalance();
    //DEBUG_RESPONSE_ONCE("module:CameraProviderV6:ReadCameraSettingsLower") lowerCamera->readCameraSettings();
    if (lowerCamera->writeCameraSettings())
      lowerCameraSettings = lowerCamera->getSettings();
  }
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
  MODIFY("module:CameraProviderV6:fullSize", image.isFullSize);
#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEG")
  {
    DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
  }
}

void CameraProviderV6::update(ImageUpper& imageUpper)
{
  MODIFY_ONCE("representation:UpperCameraSettingsV6", upperCameraSettings);
  //DEBUG_RESPONSE_ONCE("module:CameraProviderV6:LoadCameraSettingsUpperV6") readCameraSettings();
#ifdef CAMERA_INCLUDED
  static uint16_t addr, value;
  MODIFY_ONCE("module:CameraProviderV6:regAddr", addr);
  MODIFY_ONCE("module:CameraProviderV6:regValue", value);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:regWriteUpper") upperCamera->writeRegister(addr, value);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:regReadUpper") value = upperCamera->readRegister(addr);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:regWriteLower") lowerCamera->writeRegister(addr, value);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:regReadLower") value = lowerCamera->readRegister(addr);

  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:outputCurrentValuesUpper") upperCamera->outputCurrentValues();

  if (upperCamera->hasImage())
  {
    imageUpper.setResolution(upperCameraInfo.width, upperCameraInfo.height);
    imageUpper.setImage(const_cast<unsigned char*>(upperCamera->getImage()));
    //lastImageTimeStampLLUpper is in micro seconds and 64 bit
    lastImageTimeStampLLUpper = upperCamera->getTimeStamp();
    if (Global::getSettings().naoVersion == RobotConfig::V6)
    {
      int steadyNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      imageTimeStampUpper = imageUpper.timeStamp = std::max((int)imageTimeStampUpper + 1,
        (int)SystemCall::getRealSystemTime() - (steadyNow - (int)(upperCamera->getTimeStamp() / 1000)));
    }
    else
      imageTimeStampUpper = imageUpper.timeStamp = std::max(lastImageUpperTimeStamp + 1,
        (unsigned)(upperCamera->getTimeStamp() / 1000 - SystemCall::getSystemTimeBase()));
    upperCamera->setSettings(upperCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProviderV6:DoWhiteBalanceUpper") upperCamera->doAutoWhiteBalance();
    //DEBUG_RESPONSE_ONCE("module:CameraProviderV6:ReadCameraSettingsUpperV6") upperCamera->readCameraSettings();
    if(upperCamera->writeCameraSettings())
      upperCameraSettings = (CameraSettingsUpperV6&)upperCamera->getSettings();
  }

  ASSERT(imageUpper.timeStamp >= lastImageUpperTimeStamp);
  lastImageUpperTimeStamp = imageUpper.timeStamp;
  MODIFY("module:CameraProviderV6:fullSize", imageUpper.isFullSize);

  /*
  // some debug methods for mt9m114 driver
  int reg, size, value;
  MODIFY("module:CameraProviderV6:setRegister", reg);
  MODIFY("module:CameraProviderV6:setSize", size);
  MODIFY("module:CameraProviderV6:setValue", value);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:apply") upperCamera->setRegister(reg, size, value);
  DEBUG_RESPONSE_ONCE("module:CameraProviderV6:get") upperCamera->getRegister(reg, size);
  // */

#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEGUpper")
  {
    DEBUG_RESPONSE("representation:JPEGImageUpper") OUTPUT(idJPEGImageUpper, bin, JPEGImage(imageUpper));
  }
}

void CameraProviderV6::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime;
}

void CameraProviderV6::update(CameraInfo& cameraInfo)
{
  cameraInfo = lowerCameraInfo;
}

void CameraProviderV6::update(CameraInfoUpper& cameraInfoUpper)
{
  cameraInfoUpper = upperCameraInfo;
}

void CameraProviderV6::update(CameraSettingsV6& cameraSettings)
{
  cameraSettings = lowerCameraSettings;
}

void CameraProviderV6::update(CameraSettingsUpperV6& cameraSettingsUpperV6)
{
  cameraSettingsUpperV6 = upperCameraSettings;
}

void CameraProviderV6::update(CameraIntrinsics& cameraIntrinsics)
{
  cameraIntrinsics = this->cameraIntrinsics;
}

void CameraProviderV6::update(CameraResolution& cameraResolution)
{
  cameraResolution = this->cameraResolution;
}

bool CameraProviderV6::readCameraSettings()
{
  InMapFile upperStream("upperCameraSettingsV6.cfg");
  InMapFile lowerStream("lowerCameraSettingsV6.cfg");
  bool exist = upperStream.exists() && lowerStream.exists();
  if (exist)
  {
    upperStream >> upperCameraSettings;
    lowerStream >> lowerCameraSettings;
  }
  return exist;
}

bool CameraProviderV6::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if (exist)
  {
    stream >> cameraIntrinsics;
  }
  return exist;
}

bool CameraProviderV6::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if (exist)
  {
    stream >> cameraResolution;
  }
  return exist;
}

bool CameraProviderV6::processResolutionRequest()
{
  if (SystemCall::getMode() != SystemCall::Mode::physicalRobot)
  {
    return false;
  }
  if (theCameraResolutionRequest.timestamp > cameraResolution.timestamp)
  {
    switch (theCameraResolutionRequest.resolution)
    {
    case CameraResolution::Resolutions::noRequest:
      break;
    case CameraResolution::Resolutions::defaultRes:
      if (!readCameraResolution())
      {
        cameraResolution.resolution = CameraResolution::Resolutions::upper640;
        cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
      }
      break;
    case CameraResolution::Resolutions::upper640:
    case CameraResolution::Resolutions::lower640:
    case CameraResolution::Resolutions::both320:
      cameraResolution.resolution = theCameraResolutionRequest.resolution;
      cameraResolution.timestamp = theCameraResolutionRequest.timestamp;
      break;
    default:
      ASSERT(false);
      return false;
    }
    return true;
  }
  else
    return false;
}

void CameraProviderV6::setupCameras()
{
  // set resolution
  switch (cameraResolution.resolution)
  {
  case CameraResolution::Resolutions::upper640:
    upperCameraInfo.width = 640;
    upperCameraInfo.height = 480;
    lowerCameraInfo.width = 320;
    lowerCameraInfo.height = 240;
    break;
  case CameraResolution::Resolutions::lower640:
    upperCameraInfo.width = 320;
    upperCameraInfo.height = 240;
    lowerCameraInfo.width = 640;
    lowerCameraInfo.height = 480;
    break;
  case CameraResolution::Resolutions::both320:
    upperCameraInfo.width = 320;
    upperCameraInfo.height = 240;
    lowerCameraInfo.width = 320;
    lowerCameraInfo.height = 240;
    break;
  case CameraResolution::Resolutions::both640:
    upperCameraInfo.width = 640;
    upperCameraInfo.height = 480;
    lowerCameraInfo.width = 640;
    lowerCameraInfo.height = 480;
    break;
  default:
    ASSERT(false);
    break;
  }

  // set opening angle
  upperCameraInfo.openingAngleWidth = cameraIntrinsics.upperOpeningAngleWidth;
  upperCameraInfo.openingAngleHeight = cameraIntrinsics.upperOpeningAngleHeight;
  lowerCameraInfo.openingAngleWidth = cameraIntrinsics.lowerOpeningAngleWidth;
  lowerCameraInfo.openingAngleHeight = cameraIntrinsics.lowerOpeningAngleHeight;
  // set optical center
  upperCameraInfo.opticalCenter.x() = cameraIntrinsics.upperOpticalCenter.x() * upperCameraInfo.width;
  upperCameraInfo.opticalCenter.y() = cameraIntrinsics.upperOpticalCenter.y() * upperCameraInfo.height;
  lowerCameraInfo.opticalCenter.x() = cameraIntrinsics.lowerOpticalCenter.x() * lowerCameraInfo.width;
  lowerCameraInfo.opticalCenter.y() = cameraIntrinsics.lowerOpticalCenter.y() * lowerCameraInfo.height;
  // update focal length
  upperCameraInfo.updateFocalLength();
  lowerCameraInfo.updateFocalLength();
#ifdef CAMERA_INCLUDED
  if (upperCamera != nullptr)
    delete upperCamera;
  if (lowerCamera != nullptr)
    delete lowerCamera;
  upperCamera = new NaoCameraV6("/dev/video0", true, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera = new NaoCameraV6("/dev/video1", false, lowerCameraInfo.width, lowerCameraInfo.height, false);
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());

  lowerCamera->readCameraSettings();
  upperCamera->readCameraSettings();

  //lowerCameraSettings = lowerCamera->getSettings();
  //upperCameraSettings = (CameraSettingsUpperV6&)upperCamera->getSettings();

#else
  upperCamera = lowerCamera = nullptr;
  cycleTime = 1.f / 30.f;
#endif
}

bool CameraProviderV6::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if (theInstance)
    return theInstance->upperCamera->hasImage() && theInstance->lowerCamera->hasImage();
  else
#endif
    return true;
}

void CameraProviderV6::waitForFrameData2()
{
#ifdef CAMERA_INCLUDED
  // update cameraIntrinsics

  if (theCameraIntrinsicsNext.hasNext())
  {
    cameraIntrinsics = const_cast<CameraIntrinsicsNext&>(theCameraIntrinsicsNext).getNext();
    // set opening angle
    upperCameraInfo.openingAngleWidth = cameraIntrinsics.upperOpeningAngleWidth;
    upperCameraInfo.openingAngleHeight = cameraIntrinsics.upperOpeningAngleHeight;
    lowerCameraInfo.openingAngleWidth = cameraIntrinsics.lowerOpeningAngleWidth;
    lowerCameraInfo.openingAngleHeight = cameraIntrinsics.lowerOpeningAngleHeight;
    // set optical center
    upperCameraInfo.opticalCenter.x() = cameraIntrinsics.upperOpticalCenter.x() * upperCameraInfo.width;
    upperCameraInfo.opticalCenter.y() = cameraIntrinsics.upperOpticalCenter.y() * upperCameraInfo.height;
    lowerCameraInfo.opticalCenter.x() = cameraIntrinsics.lowerOpticalCenter.x() * lowerCameraInfo.width;
    lowerCameraInfo.opticalCenter.y() = cameraIntrinsics.lowerOpticalCenter.y() * lowerCameraInfo.height;
    // update focal length
    upperCameraInfo.updateFocalLength();
    lowerCameraInfo.updateFocalLength();
  }


  // update resolution
  if (processResolutionRequest())
  {
    setupCameras();
  }

  // release images
  if (lowerCamera->hasImage())
    lowerCamera->releaseImage();
  if (upperCamera->hasImage())
    upperCamera->releaseImage();

  const unsigned int timeout = 2000 * 10;

  for (;;)
  {
    if (upperCamera->hasImage() && lowerCamera->hasImage())
    {
      return;
    }

    bool resetUpper = false;
    bool resetLower = false;
    if (!lowerCamera->captureNew(timeout))
    {
      BH_TRACE;
      OUTPUT_WARNING("CameraProviderV6: Poll failed. Resetting lower camera");
      resetLower = true;
    }
    if (!upperCamera->captureNew(timeout))
    {
      BH_TRACE;
      OUTPUT_WARNING("CameraProviderV6: Poll failed. Resetting upper camera");
      resetUpper = true;
    }

    DEBUG_RESPONSE_ONCE("module:CameraProviderV6:resetUpper") resetUpper = true;
    DEBUG_RESPONSE_ONCE("module:CameraProviderV6:resetLower") resetLower = true;


    if (resetUpper)
      OUTPUT_WARNING("CameraProviderV6: Capturing image failed. Resetting upper camera.");
    if (resetLower)
      OUTPUT_WARNING("CameraProviderV6: Capturing image failed. Resetting lower camera.");

    unsigned int now = SystemCall::getRealSystemTime();

    if (!resetUpper && imageTimeStampUpper && now - imageTimeStampUpper >= timeout)
    {
      std::string msg = "imageTimeStampUpper " + std::to_string(imageTimeStampUpper) + " now " + std::to_string(now);
      BH_TRACE_MSG(msg.c_str());
      OUTPUT_WARNING("CameraProviderV6: Capturing image timed out. Resetting upper camera.");
      resetUpper = true;
    }

    if (!resetLower && imageTimeStamp && now - imageTimeStamp >= timeout)
    {
      std::string msg = "imageTimeStamp " + std::to_string(imageTimeStamp) + " now " + std::to_string(now);
      BH_TRACE_MSG(msg.c_str());
      OUTPUT_WARNING("CameraProviderV6: Capturing image timed out. Resetting lower camera.");
      resetLower = true;
    }

    // executing reset-camera.sh requires both cameras to be restarted
    if (resetUpper) resetLower = true;
    if (resetLower) resetUpper = true;

    if (resetUpper)
    {
      BH_TRACE;
      delete upperCamera;
    }
    if (resetLower)
    {
      BH_TRACE;
      delete lowerCamera;
    }
    if (resetUpper || resetLower)
    {
      VERIFY(++resetCounter <= 3);
      SystemCall::execute("/bin/bash /usr/libexec/reset-cameras.sh toggle");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (resetUpper)
    {
      upperCamera = new NaoCameraV6("/dev/video0", true, upperCameraInfo.width, upperCameraInfo.height, true);
      upperCamera->readCameraSettings();
      upperCamera->captureNew(timeout);
      imageTimeStampUpper = 0;
      lastImageUpperTimeStamp = 1;
      //SystemCall::playSound("cameraReset.wav");
    }

    if (resetLower)
    {
      lowerCamera = new NaoCameraV6("/dev/video1", false, lowerCameraInfo.width, lowerCameraInfo.height, false);
      lowerCamera->readCameraSettings();
      lowerCamera->captureNew(timeout);
      imageTimeStamp = 0;
      lastImageTimeStamp = 1;
      //SystemCall::playSound("cameraReset.wav");
    }

    // in case the image order is wrong (should not happen in NDD version of Framework)
    // TODO: check what happens in rest of code in this case.. just discarded?
    if (upperCamera->hasImage() && upperCamera->getTimeStamp() < lastImageTimeStampLLUpper)
    {
      BH_TRACE;
      upperCamera->releaseImage();
    }
    if (lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
    {
      BH_TRACE;
      lowerCamera->releaseImage();
    }

    BH_TRACE;
  }
#endif
}

void CameraProviderV6::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if (theInstance)
    theInstance->waitForFrameData2();
#endif
}

MAKE_MODULE(CameraProviderV6, cognitionInfrastructure)
