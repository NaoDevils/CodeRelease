#include "CLIPBallPerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Platform/File.h"

typedef int (*t_cnn_fp) (float x0[16][16][1], int *res, float *scores);
std::array<t_cnn_fp , 10> cnns;
namespace cnn1
{
#include "cnn.c"
}

namespace cnn_keras
{
#include "cnn_keras.c"
}

/*namespace cnn_rc18
{
#include "cnn_rc18.c"
}

namespace cnn_big_rc18
{
#include "cnn_big_rc18.c"
}*/

#define CNN_SIZE  16

CLIPBallPerceptor::CLIPBallPerceptor()
{
  lastImageTimeStamp = 0;
  lastImageUpperTimeStamp = 0;
  minDistOfCenterFromImageBorder = 5;
  ballPerceptState.reset();
  if (useCNN)
    ballHypothesis.resize(CNN_SIZE*CNN_SIZE);
  else
    ballHypothesis.resize(400);
  ballHypothesisLog.resize(30 * 30);
  
  cnns[0] = cnn1::cnn;
  cnns[1] = cnn_keras::cnn;
  //cnns[2] = cnn_rc18::cnn;
  //cnns[3] = cnn_big_rc18::cnn;

#ifdef TARGET_ROBOT
  if (logTestCircles)
  {
    std::string dir1 = std::string("mkdir ") + std::string(File::getBHDir()) + std::string("/Config/Logs/");
    std::string dir2 = std::string("mkdir ") + std::string(File::getBHDir()) + std::string("/Config/Logs/PNGs");
    std::string dir3 = std::string("mkdir ") + std::string(File::getBHDir()) + std::string("/Config/Logs/PNGs/0");
    std::string dir4 = std::string("mkdir ") + std::string(File::getBHDir()) + std::string("/Config/Logs/PNGs/1");
    system(dir1.c_str());
    system(dir2.c_str());
    system(dir3.c_str());
    system(dir4.c_str());
  }
#endif
}

void CLIPBallPerceptor::update(BallPercept &theBallPercept)
{
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:additionalScan", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScannedCenter", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballValidity", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:yJumpScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:fittingPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:upper", "drawingOnImage");

  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:wideStanceSearchArea", "drawingOnImage");
  
  if (lastImageTimeStamp != theImage.timeStamp || lastImageUpperTimeStamp != theImageUpper.timeStamp)
  {
    lastImageTimeStamp = theImage.timeStamp;
    lastImageUpperTimeStamp = theImageUpper.timeStamp;

    ballHullPoints.clear();
    localBallPercept.positionInImage = Vector2f::Zero();
    localBallPercept.relativePositionOnField = Vector2f::Zero();
    localBallPercept.status = BallPercept::notSeen;
    localBallPercept.validity = 0.f;
    localBallSpots.ballSpots.clear();
    localBallSpots.ballSpotsUpper.clear();
    noBallSpots.clear();
    execute(false);
    noBallSpots.clear();
    execute(true);
  }
  theBallPercept = localBallPercept;
}

void CLIPBallPerceptor::update(MultipleBallPercept &theMultipleBallPercept)
{
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:additionalScan", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScannedCenter", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballValidity", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:yJumpScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:fittingPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:upper", "drawingOnImage");

  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:integralImageInput:lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:integralImageInput:upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:integralImageOutput:lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:integralImageOutput:upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:wideStanceSearchArea", "drawingOnImage");

  if (lastImageTimeStamp != theImage.timeStamp || lastImageUpperTimeStamp != theImageUpper.timeStamp)
  {
    lastImageTimeStamp = theImage.timeStamp;
    lastImageUpperTimeStamp = theImageUpper.timeStamp;
    localMultipleBallPercept.balls.clear();

    ballHullPoints.clear();
    localBallPercept.positionInImage = Vector2f::Zero();
    localBallPercept.relativePositionOnField = Vector2f::Zero();
    localBallPercept.status = BallPercept::notSeen;
    localBallPercept.validity = 0.f;
    localBallSpots.ballSpots.clear();
    localBallSpots.ballSpotsUpper.clear();
    noBallSpots.clear();
    execute(false, true);
    noBallSpots.clear();
    execute(true, true);
  }
  theMultipleBallPercept = localMultipleBallPercept;
}


void CLIPBallPerceptor::update(BallHypotheses &theBallHypotheses)
{
  INIT_DEBUG_IMAGE_BLACK(BallHypothesesLower, theImage.width, theImage.height);
  INIT_DEBUG_IMAGE_BLACK(BallHypothesesUpper, theImageUpper.width, theImageUpper.height);
  theBallHypotheses.ballSpots.clear();
  theBallHypotheses.ballSpotsUpper.clear();
  int copyCount = 0;
  for (auto &spot : theBallSpots.ballSpots)
  {
    int sizeInOriginalImage = static_cast<int>(getBallDiameterAt(
      static_cast<float>(theImage.width / 2),
      static_cast<float>(spot.position.y()),
      theImage,
      theCameraMatrix,
      theCameraInfo));
    int radius = sizeInOriginalImage / 2;
    if (!theImage.isOutOfImage(spot.position.x(), spot.position.y(), radius + 5) && radius > 4)
    {
      theImage.copyAndResizeArea(spot.position.x() - radius, spot.position.y() - radius, sizeInOriginalImage, sizeInOriginalImage, 20, 20, ballHypothesis);
      // now anti-resize :)
      const float step = static_cast<float>(sizeInOriginalImage) / 20.f;
      for (int y = 0; y < 20; y++)
        for (int x = 0; x < 20; x++)
          DEBUG_IMAGE_SET_PIXEL_YUV(BallHypothesesLower,
            spot.position.x() - radius + static_cast<int>(x*step), spot.position.y() - radius + static_cast<int>(y*step),
            ballHypothesis[y * 20 + x], 128, 128);
      copyCount++;
    }
  }
  for (auto &spot : theBallSpots.ballSpotsUpper)
  {
    int sizeInOriginalImage = static_cast<int>(getBallDiameterAt(
      static_cast<float>(theImageUpper.width / 2),
      static_cast<float>(spot.position.y()),
      theImageUpper,
      theCameraMatrixUpper,
      theCameraInfoUpper));
    int radius = sizeInOriginalImage / 2;
    if (!theImageUpper.isOutOfImage(spot.position.x(), spot.position.y(), radius + 5) && radius > 4)
    {
      theImageUpper.copyAndResizeArea(spot.position.x() - radius, spot.position.y() - radius, sizeInOriginalImage, sizeInOriginalImage, 20, 20, ballHypothesis);
      // now anti-resize :)
      const float step = static_cast<float>(sizeInOriginalImage) / 20.f;
      for (int y = 0; y < 20; y++)
        for (int x = 0; x < 20; x++)
          DEBUG_IMAGE_SET_PIXEL_YUV(BallHypothesesUpper,
            spot.position.x() - radius + static_cast<int>(x*step), spot.position.y() - radius + static_cast<int>(y*step),
            ballHypothesis[y * 20 + x], 128, 128);
      copyCount++;
    }
  }
  SEND_DEBUG_IMAGE(BallHypothesesLower);
  SEND_DEBUG_IMAGE(BallHypothesesUpper);
}

void CLIPBallPerceptor::execute(const bool &upper, bool multi)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  imageWidth = image.width;
  imageHeight = image.height;

  const std::vector<BallSpot> &ballSpots = (upper ? theBallSpots.ballSpotsUpper : theBallSpots.ballSpots);
  for (int i = 0; i < (int)ballSpots.size(); i++)
  {
    ballPerceptState.reset();
    const BallSpot &spot = ballSpots.at(i);
    BallSpot temp = spot;
    temp.found = false;
    temp.centerFound = false;
    if ((!isOnBallSpot(spot.position, upper)) && calcBallSpot2016(temp, upper))
    {
      if (verifyBallPercept(temp, upper))
      {
        if (multi)
        {
          localMultipleBallPercept.balls.push_back(localBallPercept);
          noBallSpots.push_back(temp);
        }
        else
          return;
      }
    }
  }
  if (addExtraScan && upper && localBallPercept.status != BallPercept::seen)
  {
    localBallSpots.ballSpots.clear();
    localBallSpots.ballSpotsUpper.clear();
    additionalBallSpotScan();
    for (int i = 0; i < (int)localBallSpots.ballSpotsUpper.size(); i++)
    {
      ballPerceptState.reset();
      const BallSpot &spot = localBallSpots.ballSpotsUpper.at(i);
      BallSpot temp = spot;
      if ((!isOnBallSpot(spot.position, upper)) && calcBallSpot2016(temp, upper))
      {
        if (verifyBallPercept(temp, upper))
        {
          if (multi)
          {
            localMultipleBallPercept.balls.push_back(localBallPercept);
            noBallSpots.push_back(temp);
          }
          else
            return;
        }
      }
    }
  }
}

bool CLIPBallPerceptor::verifyBallPercept(BallSpot &spot, const bool &upper)
{
  float radiusInImage = spot.radiusInImage;
  Vector2i posInImage(spot.position);
  if (!ballPerceptState.cnnCheck)
    return false;
  
  localBallPercept.status = BallPercept::seen;
  localBallPercept.positionInImage = posInImage.cast<float>();
  localBallPercept.radiusInImage = (float)radiusInImage;
  localBallPercept.relativePositionOnField = ballPerceptState.ballOnField;
  localBallPercept.validity = ballPerceptState.validity;
  localBallPercept.fromUpper = upper;
  return true;
}

int CLIPBallPerceptor::getFittingPoints(const Geometry::Circle &baseCircle,
  float &distSum, float &maxDist, const bool &upper)
{
  // was 3.0*(imageHeight/240) (param originated from orange ball on 320x240 image)
  const float minAllowedRadiusDiff = std::max<float>(baseCircle.radius / 5.f, 3.f);
  std::vector< BallHullPoint > &points = (ballPerceptState.ballOnFieldLine ? goodBallHullPoints : ballHullPoints);
  std::vector< BallHullPoint >::iterator point = points.begin();
  BallHullPoint lastPoint = *point;
  std::vector< Vector2f > pointsToCheck;
  bool onCurve = false;
  bool roundTrip = false;
  size_t maxNoOfGoodConnectedHullPoints = 0;
  size_t goodHullPartSum = 0;
  size_t firstGoodCount = 0;
  int badCircleCount = 0;
  if (points.size() < 3)
    return 0;
  while (maxNoOfGoodConnectedHullPoints < points.size())
  {
    DOT("module:CLIPBallPerceptor:fittingPoints", point->pointInImage.x(), point->pointInImage.y(), ColorRGBA::yellow, ColorRGBA::yellow);
    int idDiff = point->directionID - lastPoint.directionID; // each id covers a 16th part of the circle
    if (idDiff > 2) // this allows a small hole of one missing good scan line in the ball hull
    {
      if (pointsToCheck.size() > 2)
      {
        goodHullPartSum += pointsToCheck.size();
        if (firstGoodCount == 0)
          firstGoodCount = goodHullPartSum;
      }
      pointsToCheck.clear();
      onCurve = false;
      if (roundTrip)
      {
        goodHullPartSum -= firstGoodCount;
        break;
      }
    }
    pointsToCheck.push_back(point->pointInImage);
    if (pointsToCheck.size() > 2)
    {
      Geometry::Circle testCircle;
      Geometry::computeCircleOnFieldLevenbergMarquardt(pointsToCheck, testCircle);
      if (std::abs(testCircle.radius - baseCircle.radius) > minAllowedRadiusDiff)
      {
        if (pointsToCheck.size() > 3)
        {
          goodHullPartSum += pointsToCheck.size();
          if (firstGoodCount == 0)
            firstGoodCount = goodHullPartSum;
          pointsToCheck.clear();
        }
        else
          pointsToCheck.erase(pointsToCheck.begin());
        badCircleCount++;
        onCurve = false;
        if (roundTrip)
        {
          goodHullPartSum -= firstGoodCount;
          break;
        }
      }
      else
        onCurve = true;
    }
    else
      onCurve = true;
    maxNoOfGoodConnectedHullPoints = std::max(maxNoOfGoodConnectedHullPoints, pointsToCheck.size());
    lastPoint = *point;
    point++;
    if (point == points.end())
    {
      point = points.begin();
      roundTrip = true;
      lastPoint.directionID -= 16;
    }
  }
  goodHullPartSum = std::max(goodHullPartSum, maxNoOfGoodConnectedHullPoints);
  return badCircleCount == 0 ? (int)goodHullPartSum : (int)maxNoOfGoodConnectedHullPoints;
}

bool CLIPBallPerceptor::isOnBallSpot(const Vector2i &pos, const bool &upper)
{
  const BodyContour &bc = upper ? (BodyContour&)theBodyContourUpper : theBodyContour;
  int yClipped = pos.y();
  bc.clipBottom(pos.x(), yClipped);
  if (yClipped < pos.y())
    return true;
  for (std::vector<BallSpot>::const_iterator spot = localBallSpots.ballSpots.begin(); spot != localBallSpots.ballSpots.end(); ++spot)
  {
    if ((spot->position - pos).cast<float>().norm() < (3 * spot->radiusInImage) / 2)
      return true;
  }
  for (std::vector<BallSpot>::const_iterator spot = noBallSpots.begin(); spot != noBallSpots.end(); ++spot)
  {
    if ((spot->position - pos).cast<float>().norm() < (3 * spot->radiusInImage) / 2)
      return true;
  }
  return false;
}

void CLIPBallPerceptor::additionalBallSpotScan()
{
  // only upper image for now
  bool upper = true;
  const Image &image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : (CameraMatrix&)theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  imageWidth = image.width;
  imageHeight = image.height;

  int maxStepSizeX = imageWidth / 8;
  int imgXStart = 4 + maxStepSizeX / 4 + random(maxStepSizeX / 2);
  int imgXEnd = std::min(imgXStart + maxStepSizeX * 2, imageWidth - 4);
  int minImgY = std::max<int>(std::min<int>((int)Geometry::calculateHorizon(cameraMatrix, cameraInfo).base.y(), imageHeight / 4), 4);
  int imgYStart = imageHeight - 4;
  int imgYEnd = minImgY;
  Vector2i scanPoint;

  int ballSize = 0;

  //int timeSinceLastSeen = (int)theFrameInfo.getTimeSince(std::max(theBallModel.timeWhenLastSeen,theBallModel.timeWhenLastSeenByTeamMate));
  Vector2f pImage = Vector2f::Zero();
  if (Transformation::robotToImage(
    theBallModel.estimate.position, cameraMatrix, cameraInfo, pImage)
    && !image.isOutOfImage(pImage.x(), pImage.y(), 4))
  {
    ballSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius, theBallModel.estimate.position.norm());
    if (ballSize > 50 || ballSize < 3)
      return;
    int scanRadius = std::max(ballSize * 5, 50);
    imgXStart = std::max(4, (int)pImage.x() - scanRadius);
    imgXEnd = std::min(imgXStart + scanRadius * 2, imageWidth - 4);
    imgYStart = std::min(imageHeight - 4, (int)pImage.y() + scanRadius);
    imgYEnd = std::max(imgYStart - scanRadius * 2, minImgY);
  }
  else
    return;

  if (ballSize <= 0 || imgYStart < imgYEnd || imgXEnd < imgXStart)
    return;
  scanPoint.x() = imgXStart;
  scanPoint.y() = imgYStart;
  int xStepSize = std::min(ballSize * 2, std::max((imgXEnd - imgXStart) / 10, 5));
  int yStepSize = std::min(ballSize * 2, std::max((imgYStart - imgYEnd) / 10, 5));
  int imgX = imgXStart;
  int imgY = imgYStart;
  int stepSize = imageHeight / 240;
  int stepsX = (imgXEnd - imgXStart) / stepSize;
  int stepsY = (imgYStart - imgYEnd) / stepSize;

  for (imgY = imgYStart; imgY > imgYEnd; imgY -= yStepSize)
  {
    LINE("module:CLIPBallPerceptor:additionalScan", imgXStart, imgY, imgXEnd, imgY, 2, Drawings::solidPen, ColorRGBA::orange);
    runBallSpotScanLine(imgX, imgY, stepSize, 0, stepsX, upper);
  }

  for (imgX = imgXStart; imgX < imgXEnd; imgX += xStepSize)
  {
    LINE("module:CLIPBallPerceptor:additionalScan", imgX, imgYStart, imgX, imgYEnd, 2, Drawings::solidPen, ColorRGBA::orange);
    runBallSpotScanLine(imgX, imgYEnd, 0, stepSize, stepsY, upper);
  }

}

void CLIPBallPerceptor::runBallSpotScanLine(int x, int y, int stepX, int stepY, int steps, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  //const std::vector<BallSpot> &ballSpots = (upper ? localBallSpots.ballSpotsUpper : localBallSpots.ballSpots);

  int stepNo = 0;
  const int step = std::max(stepX, stepY);
  int expectedBallSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius * 2, theBallModel.estimate.position.norm());
  ASSERT(step > 0);
  int lastY = image[y][x].y;

  bool onGreen = false;
  bool onBall = false;
  bool isEdge = false;
  int cb = 0;
  int cr = 0;
  int ballSegmentSize = 0;
  int lastBallSegmentSize = 0;
  while (stepNo < steps && !image.isOutOfImage(x, y, 10))
  {
    Image::Pixel p = image[y][x];
    isEdge = std::abs(p.y - lastY) > maxColorDiff;
    onGreen = fieldColor.isPixelFieldColor(p.y, p.cb, p.cr) || (onGreen && !isEdge);
    onBall = (!onGreen && !onBall && isEdge) || (onBall && !onGreen);
    cb += onBall*p.cb;
    cr += onBall*p.cr;

    lastBallSegmentSize = ballSegmentSize;
    ballSegmentSize = onBall * (ballSegmentSize + 1);
    if (ballSegmentSize < lastBallSegmentSize)
    {
      if (lastBallSegmentSize*step > expectedBallSize / 2 && lastBallSegmentSize*step < expectedBallSize * 2)
      {
        BallSpot bs;
        bs.position.x() = x - (lastBallSegmentSize*stepX) / (2 * step);
        bs.position.y() = y - (lastBallSegmentSize*stepY) / (2 * step);
        bs.cb = cb / lastBallSegmentSize;
        bs.cr = cr / lastBallSegmentSize;
        localBallSpots.ballSpotsUpper.push_back(bs);
        LINE("module:CLIPBallPerceptor:additionalScan", x, y,
          x - lastBallSegmentSize*stepX, y - lastBallSegmentSize*stepY,
          2, Drawings::solidPen, ColorRGBA::red);
      }
      cb = 0;
      cr = 0;
    }
    x += stepX;
    y += stepY;
    lastY = p.y;
  }
}

bool CLIPBallPerceptor::calcBallSpot2016(BallSpot &spot, const bool &upper)
{
  // TODO: shorten/split
  // TODO: far ball with obstacle overlap can be discarded? maybe only if tracking a good ball percept..!
  // TODO: too much ball spots are scanned.. check runtime! (improve speed?)
  // TODO: get ball on line, if possible not using field lines percept (center circle!)
  // TODO: remember onShadow state and correct ball position later/ remove those edge points? Problem is getting circle along shadow instead of ball.
  // TODO: how to find center if left/right overlap??
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  if (image.isOutOfImage(spot.position.x(), spot.position.y(), minDistFromImageBorder))
    return false;

  // theoretical diameter if cameramatrix is correct
  Vector2f posOnField;
  if (!Transformation::imageToRobotHorizontalPlane(
    Vector2f(spot.position.cast<float>()),
    theFieldDimensions.ballRadius,
    cameraMatrix,
    cameraInfo,
    posOnField))
    return false;
  Geometry::Circle expectedCircle;
  if (!Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle))
    return false;
  float expectedSizeInImage = expectedCircle.radius * 2;
  // how far scan lines will run - can be shortened if center is (approx) right!
  int maxScanSize = spot.found ? static_cast<int>(expectedCircle.radius*1.5f) : (spot.centerFound ?
    static_cast<int>(expectedSizeInImage) + 1 : static_cast<int>(expectedSizeInImage*1.5f));
  if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack && spot.found && (
    posOnField.norm() < 1000 || posOnField.norm() > 2000))
    maxScanSize = static_cast<int>(expectedCircle.radius*1.3f);
  maxScanSize = std::max(maxScanSize, static_cast<int>(expectedCircle.radius) + 5);
  // only check for yJumps ON ball (avoids adding useless jumps on edge of ball)
  float yJumpRadius = spot.found ? (expectedSizeInImage / 2 - std::max(2.f, expectedSizeInImage / 12)) : 0;
  int maxGreenCount = std::max(2, std::min(5, static_cast<int>(expectedCircle.radius / 6.f)));

  Vector2i scanPoint(spot.position); // the current point on the scan line
  // for yJump detection
  int yJumpSum = 0;
  int yJumpLine = -1;
  int lastY = -500;
  int newY = 0;
  bool goingUp = false;
  bool first = true;
  // for running scan line with only int operations
  const int increment = 3 * (!spot.found) + 1; // skip 3 scan lines if only scanning for center
  int xInc = increment;
  int yInc = -increment;
  int xDir = 0;
  int yDir = 4;
  float lengthPerStep = 1.f;
  float scanLength = 0.f;
  int maxI = spot.found ? 16 : 4; // TODO: make full scan dynamic according to parameter file
  // helper variables for quality check on scan line
  bool foundGreen = false;
  bool foundBallColor = false;
  int wrongColorCount = 0;
  int greenCount = 0;
  int ballColorCount = 0;
  int pointNo = 0;
  int pointNoSum = 0;
  // for detecting ball hull/center
  ballHullPoints.clear();
  goodBallHullPoints.clear();
  ballPerceptState.overlapAreas.clear();
  Vector2i lastBallColorPos(spot.position);
  scannedCenter = spot.position.cast<float>();
  scannedRadius = 0.f;
  int maxY = 0;
  if (spot.found) // when using all star like scanlines, collect y info for yAvg
  {
    for (int i = 0; i < 32; i++)
      ballPerceptState.yHistogram[i] = 0;
  }

  OverlapArea lastOVA;
  lastOVA.startID = -1;
  lastOVA.endID = -1;
  ballPerceptState.ballObstacleOverlap = false; // TODO: as of now, reset for every scan, check this
  bool overlap = false;
  bool wasOverlap = false;
  bool onImageEdge = false;
  bool onShadow = false;
  const int expectedShadowBrightness = std::min(128, fieldColor.fieldColorArray[0].fieldColorOptY);
  Image::Pixel lastP;
  int i = 0;

  // run maxI scan lines
  for (i = 0; i < maxI; i++)
  {
    scanPoint = spot.position;
    yJumpLine = 0;
    first = true;
    int divisor = std::max(std::abs(xDir), std::abs(yDir));
    lengthPerStep = std::sqrt(sqr(static_cast<float>(xDir)) + sqr(static_cast<float>(yDir))) / divisor;
    wrongColorCount = 0;
    greenCount = 0;
    overlap = false;
    foundGreen = false;
    foundBallColor = false;
    pointNo = 0;
    lastP = image[scanPoint.y()][scanPoint.x()];
    goingUp = lastP.y > 128;
    maxY = 0;
    scanLength = 0.f;
    onImageEdge = false;
    // run single scan line from spot.position in direction given via xDir/yDir
    for (int j = 0; scanLength < maxScanSize && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 4); j++)
    {
      Image::Pixel p = image[scanPoint.y()][scanPoint.x()];
      newY = p.y;
      maxY = std::max(maxY, newY);
      pointNo++;
      bool isGreen = fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
      if (isGreen || std::abs(p.cb - lastP.cb) > maxColorJumpDiff ||
        std::abs(p.cb - spot.cb) > maxColorDiff ||
        std::abs(p.cr - spot.cr) > maxColorDiff || std::abs(p.cr - lastP.cr) > maxColorJumpDiff)
      {
        wrongColorCount++;
        greenCount += isGreen;

        if ((wrongColorCount > 5 && !(theFieldDimensions.ballType == FieldDimensions::BallType::any)) ||
          greenCount > maxGreenCount)
        {
          break;
        }
        if ((theFieldDimensions.ballType == FieldDimensions::BallType::any) && greenCount == 0)
          lastBallColorPos = scanPoint;
      }
      else
      {
        ballColorCount += isPixelBallColor(p.y, p.cb, p.cr, fieldColor);
        onShadow = p.y < expectedShadowBrightness && yDir > -2; // no shadow on top of ball
        pointNoSum++;
        wrongColorCount = 0;
        lastBallColorPos = scanPoint;
        foundBallColor = true;

        // yJump stuff
        if (scanLength < yJumpRadius)
        {
          if (first)
          {
            first = false;
            lastY = newY;
          }
          int yDiff = std::abs(newY - lastY);
          if (!goingUp && ((newY - lastY) > 30 || newY - maxY < 50) && yDiff < 255)
          {
            yJumpLine++;
            goingUp = true;
          }
          if (goingUp && (newY - lastY) < -30 && yDiff < 255)
          {
            maxY = 0;
            yJumpLine++;
            goingUp = false;
          }
          lastY = newY;
          ballPerceptState.yHistogram[newY / 8]++;
        }
      }
      scanPoint.x() = spot.position.x() + (j*xDir) / divisor;
      scanPoint.y() = spot.position.y() + (j*yDir) / divisor;
      scanLength += lengthPerStep;
      lastP = p;
    }
    if (image.isOutOfImage(scanPoint.x(), scanPoint.y(), 4))
      onImageEdge = true;
    //xInc = xInc*(2 * (!(xInc - 4)) - 1);
    // for detecting possible ball center, later compared to center of circle match on ball hull
    if (yDir == 4)
    {
      if (onImageEdge)
        scannedCenter.y() = spot.position.y() + expectedSizeInImage / 2;
      else
        scannedCenter.y() = static_cast<float>(lastBallColorPos.y());
    }
    if (xDir == 4)
    {
      scannedCenter.x() = static_cast<float>(lastBallColorPos.x());
      xInc = -increment;
    }
    if (xDir == -4)
    {
      scannedRadius = (scannedRadius + scannedCenter.x() - lastBallColorPos.x()) / 2;
      scannedCenter.x() = (scannedCenter.x() + lastBallColorPos.x()) / 2;
      xInc = increment;
    }
    if (yDir == -4)
    {
      if (pointNo >= maxScanSize || onImageEdge)
      {
        scannedRadius = expectedSizeInImage / 2;
        scannedCenter.y() = scannedCenter.y() - expectedSizeInImage / 2;
      }
      else
      {
        scannedRadius = scannedCenter.y() - lastBallColorPos.y();
        scannedCenter.y() = (scannedCenter.y() + lastBallColorPos.y()) / 2;
      }
      yInc = increment;
    }
    yJumpSum += yJumpLine;
    /*ColorRGBA lineColor = spot.found ? ColorRGBA(255, 0, 0) : (spot.centerFound ? ColorRGBA(200, 0, 0) : ColorRGBA(100, 0, 0));
    LINE("module:CLIPBallPerceptor:ballScanLines",
      spot.position.x, spot.position.y,
      scanPoint.x, scanPoint.y,
      std::max(1, static_cast<int>(spot.radiusInImage / 15)), Drawings::ps_solid, lineColor);*/
    foundGreen = (greenCount > 0);
    if (scanLength >= maxScanSize + wrongColorCount*lengthPerStep && greenCount < maxGreenCount + 1)
    {
      // usually this is a bad sign, but for white/black ball this could happen in front of robot/goal
      if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack)
      {
        /*if (yDir == 0 && xDir > 0)
          rightOverlap = true;
        if (yDir == 0 && xDir < 0)
          leftOverlap = true;*/
        bool overlap = !onShadow;
        ballPerceptState.ballObstacleOverlap = ballPerceptState.ballObstacleOverlap || overlap;
        if (overlap && !wasOverlap)
        {
          lastOVA.startID = i;
        }
        if (overlap)
          lastOVA.endID = i;
        if (yDir == 4)
          scannedCenter.y() = spot.position.y() + expectedSizeInImage / 2;

        wasOverlap = overlap;
        xDir += xInc;
        yDir += yInc;
        continue; // do not add to ball hull points here
      }
      else
        return false;
    }
    if (!overlap && wasOverlap)
    {
      OverlapArea ova;
      ova.startID = lastOVA.startID;
      ova.endID = lastOVA.endID;
      ballPerceptState.overlapAreas.push_back(ova);
    }
    wasOverlap = overlap;
    if (!onImageEdge)
    {
      BallHullPoint bhp;
      bhp.pointInImage = lastBallColorPos.cast<float>();
      bhp.directionID = i;
      // good ball edge point, if green was found, ball color was found and scanline not too long
      if (foundBallColor && scanLength <= expectedSizeInImage && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), minDistFromImageBorder) && (wrongColorCount > 0 && foundGreen))
      {
        goodBallHullPoints.push_back(bhp);
        yJumpSum += yJumpLine;
      }
      if (pointNo > wrongColorCount)
        ballHullPoints.push_back(bhp);
    }
    xDir += xInc;
    yDir += yInc;

  }

  if (wasOverlap)
  {
    if (lastOVA.startID == 0)
    {
      if (ballPerceptState.overlapAreas.empty())
        return false;
      else
      {
        ballPerceptState.overlapAreas[0].startID = lastOVA.startID - 16;
      }
    }
    else
    {
      bool lastIsFirst = false;
      for (auto &ova : ballPerceptState.overlapAreas)
      {
        if (ova.startID == 0)
        {
          ova.startID = lastOVA.startID;
          lastIsFirst = true;
          break;
        }
      }
      if (!lastIsFirst)
      {
        OverlapArea ova;
        ova.startID = lastOVA.startID;
        ova.endID = lastOVA.endID;
        ballPerceptState.overlapAreas.push_back(ova);
      }
    }
  }
  // remove everything that has an overlap to the lower side
  for (auto &ova : ballPerceptState.overlapAreas)
  {
    if (ova.startID < 2 || ova.startID > 14 || ova.endID > 14)
    {
      if (ova.endID - ova.startID > (upper ? 1 : 2))
        return false;
      else
        ballPerceptState.detailedCheckNeeded = true;
    }
  }

  if (ballPerceptState.overlapAreas.size() == 2)
  {
    const OverlapArea &firstOVA = ballPerceptState.overlapAreas[0];
    const OverlapArea &secondOVA = ballPerceptState.overlapAreas[1];
    if ((firstOVA.endID - firstOVA.startID) < 2 && (secondOVA.endID - secondOVA.startID) < 2 && std::abs(Angle::normalize(firstOVA.startID*pi_8 - secondOVA.startID*pi_8)) > 75_deg)
      ballPerceptState.ballOnFieldLine = true;
  }

  DEBUG_DRAWING("module:CLIPBallPerceptor:overlapAreasRaw", "drawingOnImage")
  {
    if (spot.found)
    {
      float radius = expectedSizeInImage / 2;
      int numOfPoints = 3;
      Vector2f points[3];
      for (auto &ova : ballPerceptState.overlapAreas)
      {
        points[0] = spot.position.cast<float>();
        points[1] = Vector2f(0.f, 1.f);
        points[1].normalize(radius);
        points[1].rotate(ova.startID*pi_8);
        points[1] += points[0];
        points[1] = Vector2f(0.f, 1.f);
        points[1].normalize(radius);
        points[1].rotate(ova.endID*pi_8);
        points[2] += points[0];
        POLYGON("module:CLIPBallPerceptor:overlapAreasRaw", numOfPoints, points, 3, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
      }
    }
  }

  // scans done now check if spot does make sense so far

  if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack && !ballPerceptState.ballOnFieldLine)
  {
    for (auto &line : theCLIPFieldLinesPercept.lines)
    {
      if (line.fromUpper != upper || line.lineWidthStart < 5)
        continue;
      Geometry::Line lineToCheck;
      lineToCheck.base = line.startInImage.cast<float>();
      lineToCheck.direction = (line.endInImage - line.startInImage).cast<float>();
      Vector2f firstInterSection, secondInterSection; // not used
      if (Geometry::getIntersectionOfLineAndCircle(
        lineToCheck,
        Geometry::Circle(scannedCenter, scannedRadius / 2),
        firstInterSection,
        secondInterSection) > 0)
      {
        ballPerceptState.ballOnFieldLine = true;
        Vector2f directionNormed = lineToCheck.direction; //TODO: check if direction is always left to right!
        directionNormed.normalize(scannedRadius);
        Vector2f toLineBorder = directionNormed;
        toLineBorder.rotateLeft();
        toLineBorder.normalize((line.lineWidthStart + line.lineWidthEnd) / 4); // TODO: wrong, just approx.. enough?
        lineUpperBorder.base = scannedCenter - directionNormed + toLineBorder;
        lineUpperBorder.direction.x() = (line.lineWidthStart + line.lineWidthEnd) / 2; // to remember width
        lineUpperBorder.direction.y() = lineUpperBorder.direction.x();
        lineLowerBorder.base = scannedCenter + directionNormed - toLineBorder; // TODO: see above
        lineLowerBorder.direction = lineUpperBorder.direction;
        break;
      }
    }
  }

  // overlap allowed? 
  if (ballPerceptState.ballObstacleOverlap && !allowBallObstacleOverlap && !ballPerceptState.ballOnFieldLine)
  {
    return false;
  }

  // specific colored balls have different requirements
  int minPercent = 50;
  if (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack)
  {
    if (spot.found && yJumpSum < minNumberOfYJumps && scannedRadius / 2 > minRadiusInImage)
      return false;
    minPercent = 40;
  }
  if (theFieldDimensions.ballType != SimpleFieldDimensions::BallType::any && pointNoSum > 0 && (100 * ballColorCount) / pointNoSum < minPercent)
    return false;

  ColorRGBA centerColor = spot.found ? ColorRGBA(50, 50, 50) : (spot.centerFound ? ColorRGBA(25, 25, 25) : ColorRGBA(0, 0, 0));
  CROSS("module:CLIPBallPerceptor:ballScannedCenter", scannedCenter.x(), scannedCenter.y(), 5, 2, Drawings::solidPen, centerColor);

  if (!spot.centerFound) // 1st call, checks for scannedCenter scan
  {
    // only go on if at least 3 of 4 of the orthogonal lines are reasonable
    // exception: ball on possible field line (only allow horizontal lines and no other overlapping!)
    if (ballHullPoints.size() < 3 && !(ballPerceptState.ballOnFieldLine && ballHullPoints.size() >= 2))
      return false;
    spot.centerFound = true;
    spot.position = scannedCenter.cast<int>();
    spot.found = calcBallSpot2016(spot, upper); //2nd scan
    if (!spot.found)
      return false;
    spot.position = scannedCenter.cast<int>();
    return calcBallSpot2016(spot, upper); //3rd scan
  }
  else if (!spot.found) // checks for 2nd call (center verification)
  {
    // only go on if at least 3 of 4 of the orthogonal lines are reasonable
    // exception: ball on possible field line (only allow horizontal lines and no other overlapping!)
    if (ballHullPoints.size() >= 3 || (ballHullPoints.size() >= 2 && ballPerceptState.ballOnFieldLine))
      return true;
  }
  else // checks for 3rd call (ball hull verification)
  {
    // TODO: use yHistogram
    ballPerceptState.circleOK = false;
    if (verifyBallHull(spot, upper))
      return true;
    else if (ballPerceptState.circleOK && theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack)
    {
      if (!upper && spot.radiusInImage > 25)
        ballPerceptState.detailedCheckNeeded = true;
      else if (ballPerceptState.ballObstacleOverlap && !ballPerceptState.ballOnFieldLine)
        return false;
      Vector2f posOnField;
      if (!Transformation::imageToRobotHorizontalPlane(
        Vector2f(spot.position.cast<float>()),
        theFieldDimensions.ballRadius,
        cameraMatrix,
        cameraInfo,
        posOnField))
        return false;
      if (true)//(posOnField - theBallModel.estimate.position).abs() < 500)
      {
        BallSpot newBallSpot;
        newBallSpot.position = spot.position;
        newBallSpot.radiusInImage = spot.radiusInImage;
        localBallSpots.ballSpots.push_back(newBallSpot);
        COMPLEX_DRAWING("module:CLIPBallPerceptor:ballScanLines")
        {
          std::vector<BallHullPoint > &points = ballPerceptState.ballObstacleOverlap ? goodBallHullPoints : ballHullPoints;
          for (int i = 0; i < (int)points.size(); i++)
            LINE("module:CLIPBallPerceptor:ballScanLines",
              newBallSpot.position.x(), newBallSpot.position.y(),
              points[i].pointInImage.x(), points[i].pointInImage.y(), 1, Drawings::solidPen, ColorRGBA::yellow);
        }
        return true;
      }
    }
  }
  return false; // default
}

bool CLIPBallPerceptor::verifyBallHull(BallSpot &spot, const bool &upper)
{
  // TODO: 'roundness' could/should be better detected when ball is big in image!
  // TODO: check distribution of ball hull points! they have to be equally distributed (maybe in getFittingPoints)!
  // TODO: check all conditions
  // TODO: improve ballObstacleOverlap stuff
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  //const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  // now find best fitting circle for all ballHullPoints
  int fittingPoints = 0;
  float maxDist = 0.f;
  float distSum = 0;

  // In case of default the maximum radius of the ball is set to a quarter of the image height
  int maxRadius = image.height / 4;

  // TODO: do this with overlap-areas
  /*if (ballPerceptState.ballOnFieldLine)
  {
    std::vector< Vector2f >::iterator bhp = goodBallHullPoints.begin();
    while (bhp != goodBallHullPoints.end())
    {
      if (bhp->x() > lineUpperBorder.base.x() && bhp->x() < lineLowerBorder.base.x() &&
        ((bhp->x() < scannedCenter.x() && bhp->y() < lineUpperBorder.base.y() && bhp->y() > lineUpperBorder.base.y() - lineUpperBorder.direction.y()) ||
        (bhp->x() > scannedCenter.x() && bhp->y() > lineLowerBorder.base.y() && bhp->y() < lineLowerBorder.base.y() + lineLowerBorder.direction.y())))
        bhp = goodBallHullPoints.erase(bhp);
      else
        bhp++;
    }
    bhp = ballHullPoints.begin();
    while (bhp != ballHullPoints.end())
    {
      if (bhp->x() > lineUpperBorder.base.x() && bhp->x() < lineLowerBorder.base.x() &&
        ((bhp->x() < scannedCenter.x() && bhp->y() < lineUpperBorder.base.y() && bhp->y() > lineUpperBorder.base.y() - lineUpperBorder.direction.y()) ||
        (bhp->x() > scannedCenter.x() && bhp->y() > lineLowerBorder.base.y() && bhp->y() < lineLowerBorder.base.y() + lineLowerBorder.direction.y())))
        bhp = ballHullPoints.erase(bhp);
      else
        bhp++;
    }
  }*/
  Geometry::Circle testCircle;
  goodBallPoints.clear();
  ballPoints.clear();
  for (unsigned i = 0; i < goodBallHullPoints.size(); i++)
    goodBallPoints.push_back(goodBallHullPoints[i].pointInImage);
  for (unsigned i = 0; i < ballHullPoints.size(); i++)
    ballPoints.push_back(ballHullPoints[i].pointInImage);
  if (Geometry::computeCircleOnFieldLevenbergMarquardt(goodBallPoints, testCircle) && testCircle.radius < maxRadius)
  {
    if (testCircle.radius < 20)
      testCircle.radius *= (1.f + (20-testCircle.radius)/20.f);
    if ((!image.isOutOfImage(testCircle.center.x(), testCircle.center.y(), (int)testCircle.radius)) &&
      (scannedCenter - testCircle.center).norm() > std::max<float>(testCircle.radius, (float)(imageHeight / 80)))
      return false;
    // clip with body contour
    int yClipped = static_cast<int>(testCircle.center.y());
    theBodyContour.clipBottom(static_cast<int>(testCircle.center.x()), yClipped);
    if (yClipped - 2 > testCircle.center.y())
      return false;

    // check if ball size and position on field makes sense (from old verifyBallPercept)
    if (!logTestCircles)
    {
      if (!verifyBallSizeAndPosition(testCircle.center, testCircle.radius, upper))
        return false;
    }

    int cnnResult = -1;
    if (useCNN)
    {
      int px = static_cast<int>(testCircle.center.x() - testCircle.radius);
      int py = static_cast<int>(testCircle.center.y() - testCircle.radius);
      float imgbuf[CNN_SIZE][CNN_SIZE][1];
      float scores[2];
      if (image.projectIntoImage(
		    px, 
		    py,
		    static_cast<int>(testCircle.radius * 2),
		    static_cast<int>(testCircle.radius * 2)))
      {
        image.copyAndResizeArea(static_cast<int>(px),
          static_cast<int>(py),
          static_cast<int>(testCircle.radius * 2),
          static_cast<int>(testCircle.radius * 2),
          CNN_SIZE,
          CNN_SIZE,
          ballHypothesis
        );
        for (int i = 0; i < CNN_SIZE; i++)
          for (int j = 0; j < CNN_SIZE; j++)
            imgbuf[j][i][0] = ballHypothesis[i * CNN_SIZE + j] / 255.f;

        STOPWATCH("CNN")
        {
          cnns[cnnIndex](imgbuf, &cnnResult, scores);
        }
        DRAWTEXT("module:CLIPBallPerceptor:testCircles:lower", testCircle.center.x(), testCircle.center.y(), 15, ColorRGBA::yellow, "Score:" << static_cast<int>(scores[1] * 1000));
        DRAWTEXT("module:CLIPBallPerceptor:testCircles:upper", testCircle.center.x(), testCircle.center.y(), 15, ColorRGBA::yellow, "Score:" << static_cast<int>(scores[1] * 1000));
        ballPerceptState.validity = std::min(scores[1], 1.f); // we no have a new validity, always the score for "is a ball", also if net says "is no ball"
        float scoreUpperThreshold = minScoreUpper;
        if (testCircle.radius < 15)
          scoreUpperThreshold -= ((15 - testCircle.radius) / 75.f);
        scoreUpperThreshold = std::max(0.1f, scoreUpperThreshold);
        float minScoreForImage = upper ? scoreUpperThreshold : minScore;
        if ((cnnResult == 0 || cnnResult == 1) && scores[cnnResult] < minScoreForImage)
          cnnResult = 0;
        if ((logTestCircles && cnnResult != -1) || (cnnResult == 1 && logPositives))
        {
          static long image_id = 0;
          static const int divider = 1;
          if (image_id % divider == 0)
          {
            std::string subfolder = "";
            if (cnnResult == 0)
              subfolder = "0/";
            if (cnnResult == 1)
              subfolder = "1/";
            stbi_write_png((std::string(File::getBHDir()) + std::string("/Config/Logs/PNGs/") + subfolder + std::to_string(image_id / divider) + std::string(".png")).c_str(),
              CNN_SIZE, CNN_SIZE, 1, &ballHypothesis[0], CNN_SIZE);
          }
          image_id++;
        }
      }
    }


    // if cnn response is positive, drawn in blue
    DEBUG_RESPONSE("debug drawing:module:CLIPBallPerceptor:testCircles:lower")
    {
      ColorRGBA testCircleColor = (cnnResult == 1) ? ColorRGBA(0, 0, 255) : ColorRGBA(0, 255, 0);
      if (!upper)
        CIRCLE("module:CLIPBallPerceptor:testCircles:lower",
          testCircle.center.x(), testCircle.center.y(), testCircle.radius,
          2, Drawings::solidPen, testCircleColor, Drawings::noBrush, testCircleColor);
    }
    DEBUG_RESPONSE("debug drawing:module:CLIPBallPerceptor:testCircles:upper")
    {
      ColorRGBA testCircleColor = (cnnResult == 1) ? ColorRGBA(0, 0, 255) : ColorRGBA(0, 255, 0);
      if (upper)
        CIRCLE("module:CLIPBallPerceptor:testCircles:upper",
          testCircle.center.x(), testCircle.center.y(), testCircle.radius,
          2, Drawings::solidPen, testCircleColor, Drawings::noBrush, testCircleColor);
    }

    if (useCNNOnly)
    {
      spot.radiusInImage = testCircle.radius;
      spot.position = testCircle.center.cast<int>();
      if (cnnResult == 1)
      {
        ballPerceptState.cnnCheck = true;
        if (!useBallValidity)
          ballPerceptState.validity = 1.f;
        return true;
      }
      else
        return false;
    }

    int minFittingPointNo = minFittingPoints;
    // watch out for goal posts on both upper and lower image top border (due to white ball vs goal post or robot feet)
    if ((int)testCircle.center.y() < (int)(lowerImageUpperBorderDistanceFactor*testCircle.radius))
    {
      minFittingPointNo = minFittingPointsForSafeBall;
    }

    minDistOfCenterFromImageBorder = testCircle.radius*1.5f;
    // watch out for half white balls at left/right image border on upper image (lower border should be okay)
    if (upper && (testCircle.center.x() < minDistOfCenterFromImageBorder || testCircle.center.x() > imageWidth - minDistOfCenterFromImageBorder))
      return false;
    spot.position = testCircle.center.cast<int>();
    spot.radiusInImage = testCircle.radius;
    ballPerceptState.circleOK = true;

    fittingPoints = getFittingPoints(testCircle, distSum, maxDist, upper);
    DRAWTEXT("module:CLIPBallPerceptor:fittingPoints", testCircle.center.x(), testCircle.center.y() - 5, 10, ColorRGBA::yellow, "" << fittingPoints);
    if (fittingPoints >= minFittingPointNo)
    {
      Geometry::Circle fitCircle;
      // TODO: check this condition.. Why all ballhullpoints? 
      // should maybe just check if that circle is not larger than the other
      // but that should be handled by getFittingPoints()..
      std::vector<Vector2f > &points = ballPerceptState.ballObstacleOverlap ? goodBallPoints : ballPoints;
      if ((Geometry::computeCircleOnFieldLevenbergMarquardt(points, fitCircle) &&
        std::abs(fitCircle.radius - testCircle.radius) < std::max<float>(2.f, testCircle.radius / 6.f) &&
        (fitCircle.center - testCircle.center).norm() < std::max<float>(2.f, testCircle.radius / 5)))
      {
        // TODO: check conditions, to parameters?
        if (fittingPoints >= minFittingPointsForSafeBall)
          ballPerceptState.hullState = HullCheckState::good;
        else if (fittingPoints >= minFittingPoints)
          ballPerceptState.hullState = HullCheckState::normal;

        // TODO: use fitCircle if overlapping
        BallSpot newBallSpot;
        newBallSpot.position = testCircle.center.cast<int>();
        newBallSpot.radiusInImage = testCircle.radius;
        localBallSpots.ballSpots.push_back(newBallSpot);
        COMPLEX_DRAWING("module:CLIPBallPerceptor:ballScanLines")
        {
          for (int i = 0; i < (int)points.size(); i++)
            LINE("module:CLIPBallPerceptor:ballScanLines",
              newBallSpot.position.x(), newBallSpot.position.y(),
              (int)points[i].x(), (int)points[i].y(), 1, Drawings::solidPen, ColorRGBA::yellow);
        }
        return true;
      }
      else if (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && !upper && fittingPoints >= minFittingPointsForSafeBall)
      {
        ballPerceptState.ballObstacleOverlap = true;
      }
    }
    else
    {
      BallSpot newBallSpot;
      newBallSpot.position = testCircle.center.cast<int>();
      newBallSpot.radiusInImage = testCircle.radius;
      noBallSpots.push_back(newBallSpot);
      if (ballPerceptState.ballObstacleOverlap)
        return false;

      if (fittingPoints > 2)
        ballPerceptState.hullState = HullCheckState::none;
      else
        ballPerceptState.hullState = HullCheckState::edgy;
    }

  }
  return false;
}

bool CLIPBallPerceptor::verifyBallSizeAndPosition(const Vector2f &posInImage, const float &radius, const bool &upper)
{
  //const Image &image = upper ? (Image&)theImageUpper : theImage; unused
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  ballPerceptState.validity = 0;
  Vector2f posOnField;
  if (!Transformation::imageToRobotHorizontalPlane(
    posInImage,
    theFieldDimensions.ballRadius,
    cameraMatrix,
    cameraInfo,
    posOnField))
    return false;
  ballPerceptState.ballOnField = posOnField;
  Vector2f ballFieldCoords = Transformation::robotToField(theRobotPose, posOnField);

  if (useRobotPose)
  {
    if (!theFieldDimensions.isInsideCarpet(ballFieldCoords))
      return false;
    if (posOnField.norm() < 1000 &&
      (ballFieldCoords.x() < theFieldDimensions.xPosOwnGroundline - 200 ||
        ballFieldCoords.x() > theFieldDimensions.xPosOpponentGroundline + 200))
      return false;
  }
  // TODO: check
  // project ball on field into image to verify image radius and get better validity
  Geometry::Circle ballByDistance;
  if (!Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, ballByDistance))
    return false;

  float validity = 1.f - std::abs(std::min<float>(1.f, 1.f - ballByDistance.radius / radius));
  DRAWTEXT("module:CLIPBallPerceptor:ballValidity",
    posInImage.x(), posInImage.y() + 1.5*(posInImage.y() > imageHeight / 2 ? -radius : radius),
    15, ColorRGBA::yellow, "validity : " << validity << ";" << ballByDistance.radius << ";" << radius);
  // closer balls need better validity
  float validityFactor = ballByDistance.radius > radius ? 0.015f : 0.003f;
  if (std::abs(ballByDistance.radius - radius) < 2 ||
    ((theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any) && validity > 0.55f))
    validity = std::max<float>(validity, 0.81f);
  /*if ((ballType.ballColor == BallType::any) && image.isOutOfImage(posInImage.x, posInImage.y, radiusInImage * 1.5) &&
  !(!upper && posInImage.y + 3*radiusInImage/4 > image.resolutionHeight))
  return false;*/
  float minValidityForThisBall = std::min(lowestValidity + radius*validityFactor, minValidity);
  if (/*std::abs(posOnField.angle()) > fromDegrees(130) || */localBallPercept.validity >= validity || minValidityForThisBall > validity)
  {
    CIRCLE("module:CLIPBallPerceptor:ballScanLines",
      posInImage.x(), posInImage.y(),
      radius,
      2,
      Drawings::solidPen, ColorRGBA::yellow,
      Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    return false;
  }
  ballPerceptState.validity = validity;
  return true;
}

MAKE_MODULE(CLIPBallPerceptor, perception)
