/**
 * @file CognitionLogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CognitionLogDataProvider.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Infrastructure/YoloInput.h"
#include "Tools/Debugging/DebugDrawings3D.h"

PROCESS_LOCAL CognitionLogDataProvider* CognitionLogDataProvider::theInstance = nullptr;

MAKE_MODULE(CognitionLogDataProvider, cognitionInfrastructure)

CognitionLogDataProvider::CognitionLogDataProvider() :
  frameDataComplete(false),
  lowFrameRateImage(nullptr),
  lowFrameRateImageUpper(nullptr),
  sequenceImage(nullptr),
  sequenceImageUpper(nullptr)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  if(lowFrameRateImage)
    delete lowFrameRateImage;
  if (lowFrameRateImageUpper)
    delete lowFrameRateImageUpper;
  if(sequenceImage)
    delete sequenceImage;
  if (sequenceImageUpper)
    delete sequenceImageUpper;
  theInstance = 0;
}

void CognitionLogDataProvider::update(Image& image)
{
  if(SystemCall::getMode() == SystemCall::logfileReplay)
  {
    CameraInfo& info = (CameraInfo&) Blackboard::getInstance()["CameraInfo"];
    FrameInfo& frameInfo = (FrameInfo&)Blackboard::getInstance()["FrameInfo"];
    if(lowFrameRateImage)
    {
      if(lowFrameRateImage->imageUpdated)
        lastImages = lowFrameRateImage->image;
      else if (image.timeStamp != frameInfo.time)
        lastImages = image;

      image = lastImages;
    } else if(sequenceImage)
    {
      if(sequenceImage->noInSequence > 0)
        lastImages = sequenceImage->image;
      else if (image.timeStamp != frameInfo.time)
        lastImages = image;

      image = lastImages;
    }
    else if(info.width != image.width || info.height != image.height)
      image = Image(true, info.width, info.height);
  }

  static const float distance = 300.f;

  DECLARE_DEBUG_DRAWING3D("representation:Image", "camera");
  IMAGE3D("representation:Image", distance, 0, 0, 0, 0, 0,
          distance * theCameraInfo.width / theCameraInfo.focalLength,
          distance * theCameraInfo.height / theCameraInfo.focalLength,
          image);
  DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(image));
}

void CognitionLogDataProvider::update(ImageUpper& imageUpper)
{
  if (SystemCall::getMode() == SystemCall::logfileReplay)
  {
    CameraInfo& info = (CameraInfo&)Blackboard::getInstance()["CameraInfoUpper"];
    FrameInfo& frameInfo = (FrameInfo&)Blackboard::getInstance()["FrameInfo"];
    if (lowFrameRateImageUpper)
    {
      if (lowFrameRateImageUpper->imageUpdated)
        lastImagesUpper = lowFrameRateImageUpper->image;
      else if (imageUpper.timeStamp != frameInfo.time)
        lastImagesUpper = imageUpper;

      imageUpper = lastImagesUpper;
    } else if (sequenceImageUpper)
    {
      if (sequenceImageUpper->noInSequence > 0)
        lastImagesUpper = sequenceImageUpper->image;
      else if (imageUpper.timeStamp != frameInfo.time)
        lastImagesUpper = imageUpper;

      imageUpper = lastImagesUpper;
    }
    else if (info.width != imageUpper.width || info.height != imageUpper.height)
      imageUpper = ImageUpper(true, info.width, info.height);
  }

  static const float distance = 300.f;

  DECLARE_DEBUG_DRAWING3D("representation:ImageUpper", "camera");
  IMAGE3D("representation:ImageUpper", distance, 0, 0, 0, 0, 0,
    distance * theCameraInfoUpper.width / theCameraInfoUpper.focalLength,
    distance * theCameraInfoUpper.height / theCameraInfoUpper.focalLength,
    imageUpper);
  DEBUG_RESPONSE("representation:JPEGImageUpper") OUTPUT(idJPEGImage, bin, JPEGImage(imageUpper));
}

void CognitionLogDataProvider::update(SequenceImage& image)
{
  if (SystemCall::getMode() == SystemCall::logfileReplay)
  {
    if (sequenceImage && sequenceImage->noInSequence > 0)
    {
      image = *sequenceImage;
    }
    else 
    {
      image = SequenceImage();
    }
      
  }
}

void CognitionLogDataProvider::update(SequenceImageUpper& imageUpper)
{
  if (SystemCall::getMode() == SystemCall::logfileReplay)
  {
    if (sequenceImageUpper && sequenceImageUpper->noInSequence > 0)
    {
      imageUpper = *sequenceImageUpper;
    }
    else
    {
      imageUpper = SequenceImageUpper();
    }

  }
}

void CognitionLogDataProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);
  DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
  ARROW("loggedHorizon",
        imageCoordinateSystem.origin.x(),
        imageCoordinateSystem.origin.y(),
        imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 0) * 50,
        imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 0) * 50,
        0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  ARROW("loggedHorizon",
        imageCoordinateSystem.origin.x(),
        imageCoordinateSystem.origin.y(),
        imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 1) * 50,
        imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 1) * 50,
        0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  COMPLEX_IMAGE(corrected)
  {
    if(Blackboard::getInstance().exists("Image"))
    {
      const Image& image = (const Image&) Blackboard::getInstance()["Image"];
      INIT_DEBUG_IMAGE_BLACK(corrected, theCameraInfo.width, theCameraInfo.height);
      int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
      for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
        for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
        {
          int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x();
          for(int xSrc = 0; xSrc < theCameraInfo.width; ++xSrc)
          {
            for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x(); xDest <= xDest2; ++xDest)
            {
              DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfo.opticalCenter.x() + 0.5f),
                                        yDest + int(theCameraInfo.opticalCenter.y() + 0.5f),
                                        image[ySrc][xSrc].y,
                                        image[ySrc][xSrc].cb,
                                        image[ySrc][xSrc].cr);
            }
          }
        }
      SEND_DEBUG_IMAGE(corrected);
    }
  }
}

void CognitionLogDataProvider::update(ImageCoordinateSystemUpper& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfoUpper);
  DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
  ARROW("loggedHorizon",
    imageCoordinateSystem.origin.x(),
    imageCoordinateSystem.origin.y(),
    imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 0) * 50,
    imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 0) * 50,
    0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  ARROW("loggedHorizon",
    imageCoordinateSystem.origin.x(),
    imageCoordinateSystem.origin.y(),
    imageCoordinateSystem.origin.x() + imageCoordinateSystem.rotation(0, 1) * 50,
    imageCoordinateSystem.origin.y() + imageCoordinateSystem.rotation(1, 1) * 50,
    0, Drawings::solidPen, ColorRGBA(255, 0, 0));
  COMPLEX_IMAGE(correctedUpper)
  {
    if (Blackboard::getInstance().exists("ImageUpper"))
    {
      const Image& image = (const Image&)Blackboard::getInstance()["ImageUpper"];
      INIT_DEBUG_IMAGE_BLACK(correctedUpper, theCameraInfoUpper.width, theCameraInfoUpper.height);
      int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
      for (int ySrc = 0; ySrc < theCameraInfoUpper.height; ++ySrc)
        for (int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
        {
          int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x();
          for (int xSrc = 0; xSrc < theCameraInfoUpper.width; ++xSrc)
          {
            for (int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x(); xDest <= xDest2; ++xDest)
            {
              DEBUG_IMAGE_SET_PIXEL_YUV(correctedUpper, xDest + int(theCameraInfoUpper.opticalCenter.x() + 0.5f),
                yDest + int(theCameraInfoUpper.opticalCenter.y() + 0.5f),
                image[ySrc][xSrc].y,
                image[ySrc][xSrc].cb,
                image[ySrc][xSrc].cr);
            }
          }
        }
      SEND_DEBUG_IMAGE(correctedUpper);
    }
  }
}

void CognitionLogDataProvider::update(RobotPoseHypotheses& robotPoseHypotheses)
{
  if (robotPoseHypothesesCompressed)
    robotPoseHypotheses = *robotPoseHypothesesCompressed;
  else
    robotPoseHypotheses.hypotheses.clear();
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
    return true;
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idFrameInfo:
      if (handle(message))
      {
        if (Blackboard::getInstance().exists("Image"))
          ((Image&)Blackboard::getInstance()["Image"]).timeStamp = ((const FrameInfo&)Blackboard::getInstance()["FrameInfo"]).time;

        if (Blackboard::getInstance().exists("ImageUpper"))
          ((ImageUpper&)Blackboard::getInstance()["ImageUpper"]).timeStamp = ((const FrameInfo&)Blackboard::getInstance()["FrameInfo"]).time;
      }
      return true;

    case idImage:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = (FrameInfo&) Blackboard::getInstance()["FrameInfo"];
        const Image& image = (const Image&) Blackboard::getInstance()["Image"];
        frameInfo.cycleTime = (float) (image.timeStamp - frameInfo.time) * 0.001f;
        frameInfo.time = image.timeStamp;
      }
      return true;

    case idThumbnail:
      if(Blackboard::getInstance().exists("Image"))
      {
        Thumbnail thumbnail;
        message.bin >> thumbnail;
        thumbnail.toImage((Image&) Blackboard::getInstance()["Image"]);
      }
      return true;
    case idThumbnailUpper:
      if (Blackboard::getInstance().exists("ImageUpper"))
      {
        ThumbnailUpper thumbnail;
        message.bin >> thumbnail;
        thumbnail.toImage((ImageUpper&)Blackboard::getInstance()["ImageUpper"]);
      }
      return true;
    case idYoloInput:
      if (Blackboard::getInstance().exists("Image"))
      {
        YoloInput yoloInput;
        message.bin >> yoloInput;
        yoloInput.toImage((Image&)Blackboard::getInstance()["Image"]);
      }
      return true;
    case idYoloInputUpper:
      if (Blackboard::getInstance().exists("ImageUpper"))
      {
        YoloInputUpper yoloInputUpper;
        message.bin >> yoloInputUpper;
        yoloInputUpper.toImage((ImageUpper&)Blackboard::getInstance()["ImageUpper"]);
      }
      return true;
    case idProcessFinished:
      frameDataComplete = true;
      return true;

    case idStopwatch:
    {
      const int size = message.getMessageSize();
      std::vector<unsigned char> data;
      data.resize(size);
      message.bin.read(&data[0], size);
      Global::getDebugOut().bin.write(&data[0], size);
      Global::getDebugOut().finishMessage(idStopwatch);
      return true;
    }

    case idAnnotation:
    {
      const int size = message.getMessageSize();
      std::vector<unsigned char> data;
      data.resize(size);
      message.bin.read(&data[0], size);
      Global::getDebugOut().bin.write(&data[0], size);
      Global::getDebugOut().finishMessage(idAnnotation);
      return true;
    }

    case idJPEGImage:
      if(Blackboard::getInstance().exists("Image"))
      {
        JPEGImage jpegImage;
        message.bin >> jpegImage;
        jpegImage.toImage((Image&) Blackboard::getInstance()["Image"]);
      }
      if(Blackboard::getInstance().exists("FrameInfo"))
        ((FrameInfo&) Blackboard::getInstance()["FrameInfo"]).time = ((Image&) Blackboard::getInstance()["Image"]).timeStamp;
      return true;

    case idLowFrameRateImage:
      if(Blackboard::getInstance().exists("Image"))
      {
        if(!lowFrameRateImage)
          lowFrameRateImage = new LowFrameRateImage;
        message.bin >> *lowFrameRateImage;
      }
      return true;
    case idLowFrameRateImageUpper:
      if (Blackboard::getInstance().exists("ImageUpper"))
      {
        if (!lowFrameRateImageUpper)
          lowFrameRateImageUpper = new LowFrameRateImageUpper;
        message.bin >> *lowFrameRateImageUpper;
      }
      return true;

    case idSequenceImage:
      if(Blackboard::getInstance().exists("Image"))
      {
        if(!sequenceImage)
          sequenceImage = new SequenceImage;
        message.bin >> *sequenceImage;
      }
      return true;
    case idSequenceImageUpper:
      if (Blackboard::getInstance().exists("ImageUpper"))
      {
        if (!sequenceImageUpper)
          sequenceImageUpper = new SequenceImageUpper;
        message.bin >> *sequenceImageUpper;
      }
      return true;

    case idRobotPoseHypothesesCompressed:
      if (!robotPoseHypothesesCompressed)
        robotPoseHypothesesCompressed.reset(new RobotPoseHypothesesCompressed);
      
      message.bin >> *robotPoseHypothesesCompressed;
      return true;

    case idGameInfo:
      if (handle(message) && Blackboard::getInstance().exists("RawGameInfo"))
        (GameInfo&) Blackboard::getInstance()["RawGameInfo"] = (GameInfo&)Blackboard::getInstance()["GameInfo"];
      return true;

    default:
      return LogDataProvider::handle(message);
  }
}
