/**
 * @file CoordinateSystemProvider.cpp
 * This file implements a module that provides a coordinate system in image coordinates
 * that is parallel to the ground and compensates for distortions resulting from the
 * rolling shutter.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CoordinateSystemProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"

MAKE_MODULE(CoordinateSystemProvider, perception)

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.col(0) = horizon.direction;
  imageCoordinateSystem.rotation.col(1) = Vector2f(-horizon.direction.y(), horizon.direction.x());
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixPrev;
  RotationMatrix r(theCameraMatrix.rotation.inverse() * cmPrev.rotation);
  imageCoordinateSystem.offset = Vector2f(r.getZAngle(), r.getYAngle());

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theJointSensorData.timestamp - cameraMatrixPrevTimeStamp, false);
  cameraMatrixPrev = theCameraMatrix;
  cameraMatrixPrevTimeStamp = theJointSensorData.timestamp;

  COMPLEX_IMAGE(corrected)
  {
    INIT_DEBUG_IMAGE_BLACK(corrected, theCameraInfo.width, theCameraInfo.height);
    int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
    for(int ySrc = 0; ySrc < theImage.height; ++ySrc)
      for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
      {
        int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x();
        for(int xSrc = 0; xSrc < theImage.width; ++xSrc)
        {
          for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x(); xDest <= xDest2; ++xDest)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfo.opticalCenter.x() + 0.5f),
                                                 yDest + int(theCameraInfo.opticalCenter.y() + 0.5f),
                                                 theImage[ySrc][xSrc].y,
                                                 theImage[ySrc][xSrc].cb,
                                                 theImage[ySrc][xSrc].cr);
          }
        }
      }
    SEND_DEBUG_IMAGE(corrected);
  }

  COMPLEX_IMAGE(horizonAligned)
  {
    INIT_DEBUG_IMAGE_BLACK(horizonAligned, theCameraInfo.width, theCameraInfo.height);
    for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
      for(int xSrc = 0; xSrc < theCameraInfo.width; ++xSrc)
      {
        Vector2f corrected(imageCoordinateSystem.toCorrected(Vector2i(xSrc, ySrc)));
        corrected.x() -= theCameraInfo.opticalCenter.x();
        corrected.y() -= theCameraInfo.opticalCenter.y();
        const Vector2f& horizonAligned(imageCoordinateSystem.toHorizonAligned(corrected));

        DEBUG_IMAGE_SET_PIXEL_YUV(horizonAligned, int(horizonAligned.x() + theCameraInfo.opticalCenter.x() + 0.5f),
                                                  int(horizonAligned.y() + theCameraInfo.opticalCenter.y() + 0.5f),
                                                  theImage[ySrc][xSrc].y,
                                                  theImage[ySrc][xSrc].cb,
                                                  theImage[ySrc][xSrc].cr);
      }
    SEND_DEBUG_IMAGE(horizonAligned);
  }
}

void CoordinateSystemProvider::update(ImageCoordinateSystemUpper& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfoUpper);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrixUpper, theCameraInfoUpper);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.col(0) = horizon.direction;
  imageCoordinateSystem.rotation.col(1) = Vector2f(-horizon.direction.y(), horizon.direction.x());
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixUpperPrev;
  RotationMatrix r(theCameraMatrixUpper.rotation.inverse() * cmPrev.rotation);
  imageCoordinateSystem.offset = Vector2f(r.getZAngle(), r.getYAngle());

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theJointSensorData.timestamp - cameraMatrixUpperPrevTimeStamp, false);
  cameraMatrixUpperPrev = theCameraMatrixUpper;
  cameraMatrixUpperPrevTimeStamp = theJointSensorData.timestamp;

  COMPLEX_IMAGE(correctedUpper)
  {
    INIT_DEBUG_IMAGE_BLACK(correctedUpper, theCameraInfoUpper.width, theCameraInfoUpper.height);
    int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
    for (int ySrc = 0; ySrc < theImageUpper.height; ++ySrc)
      for (int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
      {
        int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x();
        for (int xSrc = 0; xSrc < theImageUpper.width; ++xSrc)
        {
          for (int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x(); xDest <= xDest2; ++xDest)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(correctedUpper, xDest + int(theCameraInfoUpper.opticalCenter.x() + 0.5f),
              yDest + int(theCameraInfoUpper.opticalCenter.y() + 0.5f),
              theImageUpper[ySrc][xSrc].y,
              theImageUpper[ySrc][xSrc].cb,
              theImageUpper[ySrc][xSrc].cr);
          }
        }
      }
    SEND_DEBUG_IMAGE(correctedUpper);
  }

  COMPLEX_IMAGE(horizonAlignedUpper)
  {
    INIT_DEBUG_IMAGE_BLACK(horizonAlignedUpper, theCameraInfoUpper.width, theCameraInfoUpper.height);
    for (int ySrc = 0; ySrc < theCameraInfoUpper.height; ++ySrc)
      for (int xSrc = 0; xSrc < theCameraInfoUpper.width; ++xSrc)
      {
        Vector2f corrected(imageCoordinateSystem.toCorrected(Vector2i(xSrc, ySrc)));
        corrected.x() -= theCameraInfoUpper.opticalCenter.x();
        corrected.y() -= theCameraInfoUpper.opticalCenter.y();
        const Vector2f& horizonAligned(imageCoordinateSystem.toHorizonAligned(corrected));

        DEBUG_IMAGE_SET_PIXEL_YUV(horizonAlignedUpper, int(horizonAligned.x() + theCameraInfoUpper.opticalCenter.x() + 0.5f),
          int(horizonAligned.y() + theCameraInfoUpper.opticalCenter.y() + 0.5f),
          theImageUpper[ySrc][xSrc].y,
          theImageUpper[ySrc][xSrc].cb,
          theImageUpper[ySrc][xSrc].cr);
      }
    SEND_DEBUG_IMAGE(horizonAlignedUpper);
  }
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff, bool upper) const
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  if(abTimeDiff)
  {
    float timeDiff = (float) int(abTimeDiff) * 0.001f; // in seconds
    float timeDiff2 = (float) int(theFrameInfo.time - theJointSensorData.timestamp) * 0.001f; // in seconds
    a = (timeDiff2 - imageRecordingTime - imageRecordingDelay) / timeDiff;
    b = imageRecordingTime / image.height / timeDiff;
  }
  else
    a = b = 0;
}
