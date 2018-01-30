/**
* @file CameraProvider.cpp
* This file declares a module that provides camera images.
* @author Colin Graf
*/

#include <cstdio>

#include "CameraProvider.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/Stopwatch.h"

PROCESS_LOCAL CameraProvider* CameraProvider::theInstance = 0;

CameraProvider::CameraProvider() :
  lowerCameraInfo(CameraInfo()), upperCameraInfo(CameraInfoUpper()),
  lowerCameraSettings(CameraSettings()), upperCameraSettings(CameraSettingsUpper())
#ifdef CAMERA_INCLUDED
  , imageTimeStamp(0), imageTimeStampUpper(0), lastImageTimeStamp(0), lastImageUpperTimeStamp(0), lastImageTimeStampLL(0), lastImageTimeStampLLUpper(0)
#endif
{
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());

  setupCameras();
  VERIFY(readCameraSettings());
#ifdef CAMERA_INCLUDED
  upperCamera->writeCameraSettings();
  lowerCamera->writeCameraSettings();
#endif
  theInstance = this;
}

CameraProvider::~CameraProvider()
{
#ifdef CAMERA_INCLUDED
  if(upperCamera)
  {
    delete upperCamera;
    upperCamera = nullptr;
  }
  if(lowerCamera)
  {
    delete lowerCamera;
    lowerCamera = nullptr;
  }
#endif
  theInstance = nullptr;
}

void CameraProvider::update(Image& image)
{
  MODIFY_ONCE("representation:LowerCameraSettings", lowerCameraSettings);
  //DEBUG_RESPONSE_ONCE("module:CameraProvider:LoadCameraSettings") readCameraSettings();
#ifdef CAMERA_INCLUDED
  if(lowerCamera->hasImage())
  {
    image.setResolution(lowerCameraInfo.width, lowerCameraInfo.height);
    image.setImage(const_cast<unsigned char*>(lowerCamera->getImage()));
    lastImageTimeStampLL = lowerCamera->getTimeStamp();
    imageTimeStamp = image.timeStamp = std::max(lastImageTimeStamp + 1, (unsigned)(lowerCamera->getTimeStamp() / 1000 - SystemCall::getSystemTimeBase()));
    lowerCamera->setSettings(lowerCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProvider:DoWhiteBalanceLower") lowerCamera->doAutoWhiteBalance();
    //DEBUG_RESPONSE_ONCE("module:CameraProvider:ReadCameraSettingsLower") lowerCamera->readCameraSettings();
    lowerCamera->writeCameraSettings();
    lowerCameraSettings = lowerCamera->getSettings();
  }
  ASSERT(image.timeStamp >= lastImageTimeStamp);
  lastImageTimeStamp = image.timeStamp;
  MODIFY("module:CameraProvider:fullSize", image.isFullSize);
#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEG")
  {
    DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
  }
}

void CameraProvider::update(ImageUpper& imageUpper)
{
  MODIFY_ONCE("representation:UpperCameraSettings", upperCameraSettings);
  //DEBUG_RESPONSE_ONCE("module:CameraProvider:LoadCameraSettingsUpper") readCameraSettings();
#ifdef CAMERA_INCLUDED
  if (upperCamera->hasImage())
  {
    imageUpper.setResolution(upperCameraInfo.width, upperCameraInfo.height);
    imageUpper.setImage(const_cast<unsigned char*>(upperCamera->getImage()));
    //lastImageTimeStampLLUpper is in micro seconds and 64 bit
    lastImageTimeStampLLUpper = upperCamera->getTimeStamp();
    imageTimeStampUpper = imageUpper.timeStamp = std::max(lastImageUpperTimeStamp + 1, (unsigned)(upperCamera->getTimeStamp() / 1000 - SystemCall::getSystemTimeBase()));
    upperCamera->setSettings(upperCameraSettings);
    DEBUG_RESPONSE_ONCE("module:CameraProvider:DoWhiteBalanceUpper") upperCamera->doAutoWhiteBalance();
    //DEBUG_RESPONSE_ONCE("module:CameraProvider:ReadCameraSettingsUpper") upperCamera->readCameraSettings();
    upperCamera->writeCameraSettings();
    upperCameraSettings = (CameraSettingsUpper&)upperCamera->getSettings();
  }
  
  ASSERT(imageUpper.timeStamp >= lastImageUpperTimeStamp);
  lastImageUpperTimeStamp = imageUpper.timeStamp;
  MODIFY("module:CameraProvider:fullSize", imageUpper.isFullSize);

  /*
  // some debug methods for mt9m114 driver
  int reg, size, value;
  MODIFY("module:CameraProvider:setRegister", reg);
  MODIFY("module:CameraProvider:setSize", size);
  MODIFY("module:CameraProvider:setValue", value);
  DEBUG_RESPONSE_ONCE("module:CameraProvider:apply") upperCamera->setRegister(reg, size, value);
  DEBUG_RESPONSE_ONCE("module:CameraProvider:get") upperCamera->getRegister(reg, size);
  // */

#endif // CAMERA_INCLUDED
  STOPWATCH("compressJPEGUpper")
  {
    DEBUG_RESPONSE("representation:JPEGImageUpper") OUTPUT(idJPEGImageUpper, bin, JPEGImage(imageUpper));
  }
}

void CameraProvider::update(FrameInfo& frameInfo)
{
  frameInfo.time = theImage.timeStamp;
  frameInfo.cycleTime = cycleTime;
}

void CameraProvider::update(CameraInfo& cameraInfo)
{
  cameraInfo = lowerCameraInfo;
}

void CameraProvider::update(CameraInfoUpper& cameraInfoUpper)
{
  cameraInfoUpper = upperCameraInfo;
}

void CameraProvider::update(CameraSettings& cameraSettings)
{
  cameraSettings = lowerCameraSettings;
}

void CameraProvider::update(CameraSettingsUpper& cameraSettingsUpper)
{
  cameraSettingsUpper = upperCameraSettings;
}

void CameraProvider::update(CameraIntrinsics& cameraIntrinsics)
{
  cameraIntrinsics = this->cameraIntrinsics;
}

void CameraProvider::update(CameraResolution& cameraResolution)
{
  cameraResolution = this->cameraResolution;
}

bool CameraProvider::readCameraSettings()
{
  InMapFile upperStream("upperCameraSettings.cfg");
  InMapFile lowerStream("lowerCameraSettings.cfg");
  bool exist = upperStream.exists() && lowerStream.exists();
  if(exist)
  {
    upperStream >> upperCameraSettings;
    lowerStream >> lowerCameraSettings;
  }
  return exist;
}

bool CameraProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
  {
    stream >> cameraIntrinsics;
  }
  return exist;
}

bool CameraProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
  {
    stream >> cameraResolution;
  }
  return exist;
}

bool CameraProvider::processResolutionRequest()
{
  if(SystemCall::getMode() != SystemCall::Mode::physicalRobot)
  {
    return false;
  }
  if(theCameraResolutionRequest.timestamp > cameraResolution.timestamp)
  {
    switch(theCameraResolutionRequest.resolution)
    {
      case CameraResolution::Resolutions::noRequest:
        break;
      case CameraResolution::Resolutions::defaultRes:
        if(!readCameraResolution())
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

void CameraProvider::setupCameras()
{
  // set resolution
  switch(cameraResolution.resolution)
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
  if(upperCamera != nullptr)
    delete upperCamera;
  if(lowerCamera != nullptr)
    delete lowerCamera;
  upperCamera = new NaoCamera("/dev/video0", true, upperCameraInfo.width, upperCameraInfo.height, true);
  lowerCamera = new NaoCamera("/dev/video1", false, lowerCameraInfo.width, lowerCameraInfo.height, false);
  cycleTime = upperCamera->getFrameRate();
  ASSERT(upperCamera->getFrameRate() == lowerCamera->getFrameRate());

  lowerCamera->readCameraSettings();
  upperCamera->readCameraSettings();
  lowerCameraSettings = lowerCamera->getSettings();
  upperCameraSettings = (CameraSettingsUpper&) upperCamera->getSettings();

#else
  upperCamera = lowerCamera = nullptr;
  cycleTime = 1.f / 30.f;
#endif
}

bool CameraProvider::isFrameDataComplete()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    return theInstance->upperCamera->hasImage() && theInstance->lowerCamera->hasImage();
  else
#endif
    return true;
}

void CameraProvider::waitForFrameData2()
{
#ifdef CAMERA_INCLUDED
  // update cameraIntrinsics
  if(theCameraIntrinsicsNext.hasNext())
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
  if(processResolutionRequest())
  {
    setupCameras();
  }

  // release images
  if (lowerCamera->hasImage())
    lowerCamera->releaseImage();
  if (upperCamera->hasImage())
    upperCamera->releaseImage();

  const unsigned int timeout = 2000 * 10;

  for(;;)
  {
    if (upperCamera->hasImage() && lowerCamera->hasImage())
    {
      return;
    }

    bool resetUpper = false;
    bool resetLower = false;
    if(!lowerCamera->captureNew(timeout))
    {
      OUTPUT_WARNING("CameraProvider: Poll failed. Resetting lower camera");
      resetLower = true;
    }
    if (!upperCamera->captureNew(timeout))
    {
      OUTPUT_WARNING("CameraProvider: Poll failed. Resetting upper camera");
      resetUpper = true;
    }
    if(resetUpper)
      OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting upper camera.");
    if(resetLower)
      OUTPUT_WARNING("CameraProvider: Capturing image failed. Resetting lower camera.");

    unsigned int now = SystemCall::getRealSystemTime();

      if(!resetUpper && imageTimeStampUpper && now - imageTimeStampUpper >= timeout)
    {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting upper camera.");
      resetUpper = true;
    }
    if(!resetLower && imageTimeStamp && now - imageTimeStamp >= timeout)
    {
      OUTPUT_WARNING("CameraProvider: Capturing image timed out. Resetting lower camera.");
      resetLower = true;
    }

    if(resetUpper)
    {
      delete upperCamera;
      upperCamera = new NaoCamera("/dev/video0", true, upperCameraInfo.width, upperCameraInfo.height, true);
      imageTimeStampUpper = 0;
      SystemCall::playSound("cameraReset.wav");
    }

    if(resetLower)
    {
      delete lowerCamera;
      lowerCamera = new NaoCamera("/dev/video1", false, lowerCameraInfo.width, lowerCameraInfo.height, false);
      imageTimeStamp = 0;
      SystemCall::playSound("cameraReset.wav");
    }

    // in case the image order is wrong (should not happen in NDD version of Framework)
    // TODO: check what happens in rest of code in this case.. just discarded?
    if(upperCamera->hasImage() && upperCamera->getTimeStamp() < lastImageTimeStampLLUpper)
      upperCamera->releaseImage();
    if(lowerCamera->hasImage() && lowerCamera->getTimeStamp() < lastImageTimeStampLL)
      lowerCamera->releaseImage();
  }

#endif
}

void CameraProvider::waitForFrameData()
{
#ifdef CAMERA_INCLUDED
  if(theInstance)
    theInstance->waitForFrameData2();
#endif
}

MAKE_MODULE(CameraProvider, cognitionInfrastructure)
