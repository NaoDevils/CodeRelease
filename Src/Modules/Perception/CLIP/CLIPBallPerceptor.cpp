#include "CLIPBallPerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/RingBufferWithSum.h"

CLIPBallPerceptor::CLIPBallPerceptor()
{
  lastImageTimeStamp = 0;
  lastImageUpperTimeStamp = 0;
  minDistOfCenterFromImageBorder = 5;
  ballPerceptState.reset();
  ballPerceptState.ballFeatures.reserve(20);
}

void CLIPBallPerceptor::update(BallPercept &theBallPercept)
{
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:additionalScan","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScanLines","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScannedCenter","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballValidity", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:yJumpScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballFeatures", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:fittingPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles", "drawingOnImage");
  //DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:featureHistogram", "drawingOnImage");
  //DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:distanceHistogram", "drawingOnImage");
  
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
    if (localBallPercept.validity <= minValidity)
    {
      noBallSpots.clear();
      execute(true);
    }
  }
  theBallPercept = localBallPercept;
}

void CLIPBallPerceptor::execute(const bool &upper)
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
    if ((!isOnBallSpot(spot.position)) && calcBallSpot2016(temp,upper))
    {
      if (verifyBallPercept(temp,upper))
        return;
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
      if ((!isOnBallSpot(spot.position)) && calcBallSpot2016(temp,upper))
      {
        if (verifyBallPercept(temp, upper))
          return;
      }
    }
  }
}

bool CLIPBallPerceptor::verifyBallPercept(BallSpot &spot, const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const Image &image = upper ? (Image&)theImageUpper : theImage;

  Vector2f posOnField(0,0);
  float radiusInImage = spot.radiusInImage;
  Vector2i posInImage(spot.position);
  if (radiusInImage < minRadiusInImage)
    return false;
  // is on robot percept?
  for (auto &robot : theRobotsPercept.robots)
  {
    if (upper == robot.fromUpperImage)
    {
      if ((posInImage.x() > robot.imageUpperLeft.x() && posInImage.x() < robot.imageLowerRight.x())
        && (posInImage.y() > robot.imageUpperLeft.y() && posInImage.y() < robot.imageUpperLeft.y() + (robot.imageLowerRight.y() - robot.imageUpperLeft.y())/2))
        return false;
    }
  }
  // there should not be much green on the ball (but could be a little in bad image due to reflections)
  if (countGreenOnBallSpot(localBallSpots.ballSpots.back(),upper) > std::max((int)radiusInImage/2,3))
    return false;
  /*Vector2<> angles(0,0);
  Geometry_D::calculateAnglesForPoint(posInImage,CameraMatrix,cameraInfo,angles);
  float alpha = angles.x;*/
  if (!Transformation::imageToRobotHorizontalPlane(
    posInImage.cast<float>(), 
    theFieldDimensions.ballRadius,
    cameraMatrix, 
    cameraInfo, 
    posOnField))
    return false;
  Vector2f ballFieldCoords = Transformation::robotToField(theRobotPose,posOnField);
      
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

  if (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack)
  {
    if (radiusInImage < minRadiusInImageForFeatures)
    {
      if (image.isOutOfImage(spot.position.x(), spot.position.y(), static_cast<int>(radiusInImage) * 3))
        return false;
      ballPerceptState.featureCheckNeeded = false;
    }
    if (!checkYJumps(spot, upper))
      return false;
    if (ballPerceptState.ballObstacleOverlap)
      ballPerceptState.featureCheckNeeded = true;
    if (ballPerceptState.hullState == HullCheckState::edgy)
    {
      ballPerceptState.detailedCheckNeeded = true;
      ballPerceptState.featureCheckNeeded = true;
    }
    // TODO: replace detailedCheckNeeded with featureCheckNeeded?? might be dangerous
    bool overlapOK = (!ballPerceptState.detailedCheckNeeded && !(spot.radiusInImage < minRadiusInImageForFeatures)) || !scanForOverlap(spot, upper);
    bool featuresOK = !ballPerceptState.featureCheckNeeded || checkFeatures(spot, upper);
    /*if (featureCheckNeeded && ((!possibleBallInFrontOfRobot && !(detailedCheckNeeded && ballOnFieldLine) && scanForOverlap(spot, upper)) || !checkFeatures(spot, upper)))
      return false;*/
    // TODO: watch out for ballOnFieldLine = true on robots!!
    if (!overlapOK || !featuresOK)
      return false;
  }

  float validity = 1.f-std::abs(std::min<float>(1.f,1.f-ballByDistance.radius/radiusInImage));
  DRAWTEXT("module:CLIPBallPerceptor:ballValidity",
    posInImage.x(),posInImage.y() + 1.5*(posInImage.y() > imageHeight/2 ? -radiusInImage : radiusInImage),
    15,ColorRGBA::yellow,"validity : " << validity << ";" << ballByDistance.radius << ";" << radiusInImage);
  // closer balls need better validity
  float validityFactor = ballByDistance.radius > radiusInImage ? 0.015f : 0.003f;
  if (std::abs(ballByDistance.radius-radiusInImage) < 2 || 
    ((theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any) && validity > 0.55f))
    validity = std::max<float>(validity,0.81f);
  /*if ((ballType.ballColor == BallType::any) && image.isOutOfImage(posInImage.x, posInImage.y, radiusInImage * 1.5) &&
    !(!upper && posInImage.y + 3*radiusInImage/4 > image.resolutionHeight))
    return false;*/
  float minValidityForThisBall = std::min(lowestValidity + radiusInImage*validityFactor,minValidity);
  if (/*std::abs(posOnField.angle()) > fromDegrees(130) || */localBallPercept.validity >= validity || minValidityForThisBall > validity)
  {
    CIRCLE("module:CLIPBallPerceptor:ballScanLines",
      posInImage.x(),posInImage.y(),
      radiusInImage,
      2,
      Drawings::solidPen, ColorRGBA::yellow,
      Drawings::solidBrush, ColorRGBA(255,128,64,100));
    return false;
  }
  localBallPercept.status = BallPercept::seen;
  localBallPercept.positionInImage = posInImage.cast<float>();
  localBallPercept.radiusInImage = (float)radiusInImage;
  localBallPercept.relativePositionOnField = posOnField;
  localBallPercept.validity = validity;
  localBallPercept.fromUpper = upper;
  if (localBallPercept.validity > minValidityForThisBall)
    return true;
  else
    return false;
}

bool CLIPBallPerceptor::checkYJumps(BallSpot &spot, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  //const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix; unused
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo; unused
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;

  const int radius = static_cast<int>(spot.radiusInImage - std::max(2.f, spot.radiusInImage / 8));
  const int yBlackToWhiteSplit = fieldColor.fieldColorArray[0].fieldColorOptY;
  const int thresholdJump = std::max(20, 30 - std::abs(90 - yBlackToWhiteSplit) / 10);
  const int thresholdJumpSure = 3 * thresholdJump / 2;
  /*if (upper && !ballPerceptState.ballObstacleOverlap && ballPerceptState.hullState == HullCheckState::good)
    return true;*/
  if (image.isOutOfImage(spot.position.x(), spot.position.y(), 3))
    return false;
  int yJumpSum = 0;
  int yJumpLine = -1;
  int lastY = -500;
  int newY = 0;
  Vector2i scanPoint(spot.position);
  bool goingUp = true;
  bool first = true;
  int xInc = 1;
  int yInc = -1;
  int xDir = 0;
  int yDir = 4;
  Vector2i onFeature(0, 0);
  int yAvg = 0;
  int yCount = 0;
  int maxY = 0;
  //const int minY = std::min(128, fieldColor.fieldColorOptY); unused
  //const float expectedFeatureSize = expectedBallFeatureSize*spot.radiusInImage; unused
  int ySum = 0;
  int ySumCount = 0;
  for (int i = 0; i < 16; i++)
  {
    scanPoint = spot.position;
    yJumpLine = 0;
    onFeature = spot.position;
    yAvg = 0;
    yCount = 0;
    first = true;
    goingUp = image[scanPoint.y()][scanPoint.x()].y > 128;

    int divisor = std::max(std::abs(xDir), std::abs(yDir));
    for (int j = 0; (scanPoint - spot.position).cast<float>().norm() < radius && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 2); j++)
    {
      newY = image[scanPoint.y()][scanPoint.x()].y;
      maxY = std::max(newY, maxY);
      if (first)
      {
        first = false;
        lastY = newY;
      }
      int yDiff = std::abs(newY - lastY);
      if (!goingUp && yCount > 0 && ((newY - lastY) > thresholdJump || (maxY - newY > thresholdJumpSure)) && yDiff < 255)
      {
        maxY = 0;
        yJumpLine++;
        goingUp = true;
        /*BallFeature bf;
        bf.yAvg = yAvg/yCount;
        if (bf.yAvg < fieldColor.fieldColorOptY && scanFeature(bf,
          Vector2<>(static_cast<float>(scanPoint.x + onFeature.x) / 2,
          static_cast<float>(scanPoint.y + onFeature.y) / 2),
          expectedFeatureSize, upper))
        {
          bool onOtherFeature = bf.scannedSize > expectedFeatureSize + expectedFeatureSize*params.maxBallFeatureSizeDeviation;
          for (auto bfToCheck : ballFeatures)
          {
            if ((bfToCheck.center - bf.center).abs() < expectedFeatureSize)
            {
              bfToCheck.center = (bfToCheck.center + bf.center) / 2;
              bfToCheck.yAvg = (bfToCheck.yAvg + bf.yAvg) / 2;
              bfToCheck.scannedSize = std::max(bfToCheck.scannedSize, bf.scannedSize);
              onOtherFeature = true;
            }
          }
          if (!onOtherFeature)
            ballFeatures.emplace_back(bf);
        }*/
      }
      if (goingUp && ((newY - lastY) < -thresholdJump || (newY - maxY) < -thresholdJumpSure) && yDiff < 255)
      {
        onFeature = scanPoint;
        yAvg = 0;
        yCount = 0;
        yJumpLine++;
        goingUp = false;
      }
      yAvg += newY;
      yCount++;
      scanPoint.x() = spot.position.x() + (j*xDir) / divisor;
      scanPoint.y() = spot.position.y() + (j*yDir) / divisor;
      lastY = newY;
      ySum += newY;
      ySumCount++;
    }
    /*if (yCount > 0)
    {
      BallFeature bf;
      bf.yAvg = yAvg / yCount;
      if (bf.yAvg < minY && scanFeature(bf,
        Vector2<>(static_cast<float>(scanPoint.x + onFeature.x) / 2,
        static_cast<float>(scanPoint.y + onFeature.y) / 2),
        expectedFeatureSize, upper))
      {
        bool onOtherFeature = bf.scannedSize > expectedFeatureSize + expectedFeatureSize*params.maxBallFeatureSizeDeviation;
        for (auto bfToCheck : ballFeatures)
        {
          if ((bfToCheck.center - bf.center).abs() < expectedFeatureSize)
          {
            bfToCheck.center = (bfToCheck.center + bf.center) / 2;
            bfToCheck.yAvg = (bfToCheck.yAvg + bf.yAvg) / 2;
            bfToCheck.scannedSize = std::max(bfToCheck.scannedSize, bf.scannedSize);
            onOtherFeature = true;
          }
        }
        if (!onOtherFeature)
          ballFeatures.emplace_back(bf);
      }
    }*/
    if (xDir == 4)
      xInc = -1;
    if (xDir == -4)
      xInc = 1;
    if (yDir == -4)
      yInc = 1;
    xDir += xInc;
    yDir += yInc;
    yJumpSum += yJumpLine;
    LINE("module:CLIPBallPerceptor:yJumpScanLines", 
      spot.position.x(), spot.position.y(), 
      scanPoint.x(), scanPoint.y(), 
      std::max(1, static_cast<int>(spot.radiusInImage / 15)), Drawings::solidPen, ColorRGBA::green);
  }
  if (ySumCount > 0)
    spot.y = ySum / ySumCount;
  else
    spot.y = fieldColor.fieldColorArray[0].fieldColorOptY + 30;
  /*if (ballObstacleOverlap)
  {
    // check features for white/black ball
    if (ballFeatures.empty())
      return false;
    Geometry::Circle ballModelBall;
    if (ballFeatures.size() < 2 && !Geometry::calculateBallInImage(theBallModel.estimate.position, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, ballModelBall))
      return false;
    if (Vector2<>(ballModelBall.center.x - spot.position.x, ballModelBall.center.y - spot.position.y).abs() > spot.radiusInImage * 2)
    {
      if (ballFeatures.size() < 2)
        return false;
      // distance check, size check is done before adding features
      float expectedFeatureDistance = params.expectedBallFeatureDistance*spot.radiusInImage;
      int goodDistanceCount = 0;
      for (unsigned i = 0; i < ballFeatures.size(); i++)
      {
        for (unsigned j = i + 1; j < ballFeatures.size(); j++)
        {
          float centerDistance = (ballFeatures[i].center - ballFeatures[j].center).abs();
          goodDistanceCount += (std::abs(1 - centerDistance / expectedFeatureDistance) < params.maxBallFeatureDistanceDeviation);
        }
      }
      if (goodDistanceCount < ballFeatures.size() - 1)
        return false;
    }
  }*/
  if (yJumpSum > std::max(upper ? minNumberOfYJumps : 16, static_cast<int>(spot.radiusInImage / 3)))
    return true;
  else
    return false;
}

bool CLIPBallPerceptor::checkFeatures(const BallSpot &spot, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  //const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : (CameraMatrix&)theCameraMatrix; unused
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo; unused
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;

  const int radius = static_cast<int>(spot.radiusInImage - std::max(2.f, spot.radiusInImage / 8));
  if (image.isOutOfImage(spot.position.x(), spot.position.y(), 3) || radius > 50)
    return false;
  
  //const int minY = std::min(128, fieldColor.fieldColorOptY); unused
  const float expectedFeatureSize = expectedBallFeatureSize*spot.radiusInImage;
  const float maxXDistanceOnFeature = expectedFeatureSize / 4;
  const float minFeatureSize = expectedFeatureSize / 4;
  static const float pi_16 = pi / 16.f;
  // scan ball horizontaly
  Vector2f startingPoint;
  startingPoint.x() = 0;
  startingPoint.y() = static_cast<float>(radius);
  int xOffsetToCenterPoint = 0;
  //int maxFeatureVal = 0; unused
  //int maxDistanceVal = 0; unused
  ballPerceptState.ballFeatures.clear();
  int wrongColorCount = 0;
  int pixelCount = 0;
  int lastYPos = 0;
  int maxAllowedYValForFeature = std::min(128, 6 * fieldColor.fieldColorArray[0].fieldColorOptY / 5);
  int highLowScore = 0;
  int maxWhiteScore = 0;
  int whiteScore = 0; // TODO: only for a small window of pixels!
  int maxBlackScore = 0;
  int blackScore = 0; // TODO: only for a small window of pixels!
  
  for (int row = 0; row < 15; row++)
  {
    startingPoint.rotate(pi_16);
    xOffsetToCenterPoint = static_cast<int>(startingPoint.x());
    int yPos = static_cast<int>(startingPoint.y()) + spot.position.y();
    if (yPos == lastYPos)
      continue;
    int firstY = -1;
    bool onHigh = false;
    int maxYVal = 0;
    int minYVal = 255;
    int minX = spot.position.x() + xOffsetToCenterPoint;
    int maxX = spot.position.x() - xOffsetToCenterPoint;
    int startFeature = minX;
    for (int xPos = minX; xPos < maxX; xPos++)
    {
      if (image.isOutOfImage(xPos, yPos, 3))
        continue;
      Image::Pixel p = image[yPos][xPos];
      if (fieldColor.isPixelFieldColor(p.y, p.cb, p.cr))
        wrongColorCount += 5;
      else if (!isPixelBallColor(p.y, p.cb, p.cr,fieldColor))
        wrongColorCount++;
      pixelCount++;
      int yVal = p.y;

      if (firstY < 0)
      {
        firstY = yVal;
        onHigh = yVal > maxAllowedYValForFeature;
      }
      if (onHigh)
      {
        blackScore = 0;
        maxYVal = std::max(maxYVal, yVal);
        highLowScore++;
        whiteScore++;
        maxWhiteScore = std::max(maxWhiteScore, whiteScore);
      }
      else
      {
        whiteScore = 0;
        minYVal = std::min(minYVal, yVal);
        highLowScore--;
        blackScore++;
        maxBlackScore = std::min(maxBlackScore, blackScore);
      }
      if (onHigh && yVal < maxAllowedYValForFeature && yVal < maxYVal - 50)
      {
        startFeature = xPos;
        onHigh = false;
        maxYVal = 0;
        minYVal = std::min(minYVal, yVal);
      }
      if (!onHigh && (yVal > minYVal + 50 || (xPos == maxX - 1)))
      {
        maxYVal = std::max(maxYVal, yVal);
        int featureSize = xPos - startFeature;
        int newCenterX = (xPos + startFeature) / 2;
        if (featureSize > minFeatureSize)
        {
          bool foundAlready = false;
          for (auto &bf : ballPerceptState.ballFeatures)
          {
            if (bf.center.y() - bf.scannedSizeY - lastYPos < 2 && std::abs(bf.center.x() - newCenterX) < maxXDistanceOnFeature)
            {
              bf.scannedSizeY += lastYPos - yPos;
              bf.scannedSizeX = std::max(bf.scannedSizeX, static_cast<float>(featureSize));
              foundAlready = true;
            }
          }
          if (!foundAlready)
          {
            BallFeature bf;
            bf.center.x() = static_cast<float>(newCenterX); // TODO
            bf.center.y() = static_cast<float>(yPos);
            bf.scannedSizeY = 0;
            bf.scannedSizeX = static_cast<float>(featureSize);
            ballPerceptState.ballFeatures.emplace_back(bf);
          }
        }
        onHigh = true;
        minYVal = 255;
      }
      LINE("module:CLIPBallPerceptor:ballFeatures", minX, yPos, maxX, yPos, 2, Drawings::solidPen, ColorRGBA(0, 255, 0));
    }
    lastYPos = yPos;
  }
  float maxSize = (ballPerceptState.detailedCheckNeeded ? expectedFeatureSize * 1.2f : (expectedFeatureSize + expectedFeatureSize * maxBallFeatureSizeDeviation));
  size_t minFeatureCount = (ballPerceptState.detailedCheckNeeded ? 3 : 2);
  bool detailedCheckPassed = ballPerceptState.ballFeatures.size() > 2;
  int score = (highLowScore*100) / pixelCount;
  whiteScore = (maxWhiteScore * 100) / pixelCount;
  DRAWTEXT("module:CLIPBallPerceptor:ballFeatures", spot.position.x(), spot.position.y(), spot.radiusInImage / 2, ColorRGBA::magenta, "S:" << score << "," << whiteScore);
  for (auto &bf : ballPerceptState.ballFeatures)
  {
    CIRCLE("module:CLIPBallPerceptor:ballFeatures", 
      bf.center.x(), bf.center.y(), std::max(bf.scannedSizeX,bf.scannedSizeY) / 2, 
      2, Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::noBrush, ColorRGBA(255, 0, 0));
    if (std::max(bf.scannedSizeX, bf.scannedSizeY) > maxSize)
      return false;
  }
  bool onImageEdge = image.isOutOfImage(spot.position.x(), spot.position.y(), static_cast<int>(spot.radiusInImage) + 10);
  bool simpleFeatureCheckOk = !upper && !ballPerceptState.ballObstacleOverlap && !ballPerceptState.ballOnFieldLine;
  return pixelCount > 0
    && wrongColorCount < pixelCount / maxWrongColorDiv
    && ballPerceptState.ballFeatures.size() >= minFeatureCount
    && score <= (maxBallFeatureScore + (detailedCheckPassed ? (simpleFeatureCheckOk ? 15 : 5) : 0))
    && score >= (minBallFeatureScore - (detailedCheckPassed ? (5) : 0))
    && whiteScore < (maxBallWhiteScore + (detailedCheckPassed ? (5) : 0))
    && checkFeatureDistribution(spot,upper);
}

bool CLIPBallPerceptor::checkFeatureDistribution(const BallSpot &spot, const bool &upper)
{
  return true;
}

bool CLIPBallPerceptor::scanForOverlap(const BallSpot &spot, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo; unused
  int overlapCount = 0;
  int overlapAreas = 0;
  bool lastOverlap = false;
  bool onShadow = false;
  const int expectedShadowBrightness = fieldColor.fieldColorArray[0].fieldColorOptY;
  if (image.isOutOfImage(spot.position.x(), spot.position.y(), 3))
    return true;
  int xInc = 0;
  int yInc = -1;
  Image::Pixel lastP = image[spot.position.y()][spot.position.x()];
  Vector2i scanPoint = spot.position;
  const int maxRadius = static_cast<int>(spot.radiusInImage + std::min(10.f,std::max(5.f, spot.radiusInImage / 2)));
  int countSinceWrongColor = 0;
  int wrongColorCount = 0;
  const float distPer45Step = std::sqrt(2.f) - 1.f;
  float distPerStep = 1.f;
  float dist = 0.f;

  // TODO!
  for (int i = 0; i < 8; i++)
  {
    countSinceWrongColor = 0;
    wrongColorCount = 0;
    distPerStep = 1 + (i % 2)*distPer45Step;
    for (dist = 0.f; dist < maxRadius; dist += distPerStep)
    {
      countSinceWrongColor++;
      scanPoint.x() += xInc;
      scanPoint.y() += yInc;
      if (image.isOutOfImage(scanPoint.x(), scanPoint.y(), 3))
        break;
      Image::Pixel p = image[scanPoint.y()][scanPoint.x()];
      onShadow = p.y < expectedShadowBrightness;
      bool onGreen = fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
      if (!onGreen && isPixelBallColor(p.y, p.cb, p.cr, fieldColor))
        continue;
      if (std::abs(p.cb - lastP.cb) > maxColorJumpDiff ||
        std::abs(p.cb - spot.cb) > maxColorDiff ||
        std::abs(p.cr - spot.cr) > maxColorDiff || 
        std::abs(p.cr - lastP.cr) > maxColorJumpDiff ||
        onGreen)
      {
        wrongColorCount++;
        countSinceWrongColor = -1;
        if (wrongColorCount > 2)
          break;
      }
    }
    // TODO: use distPerStep here
    int y = scanPoint.y();
    if (!upper)
      theBodyContour.clipBottom(scanPoint.x(), y);
    bool ownBodyOverlap = yInc > 0 && y < scanPoint.y();
    if (!ownBodyOverlap && !(onShadow && (wrongColorCount < 2 || countSinceWrongColor < 5)) &&
      (dist >= maxRadius || wrongColorCount < 2 || countSinceWrongColor > spot.radiusInImage / 2))
    {
      overlapAreas += !lastOverlap;
      overlapCount++;
    }
    scanPoint = spot.position;
    if (xInc == 0 && yInc == -1)
      xInc = -1;
    else if (xInc == -1 && yInc == -1)
      yInc = 0;
    else if (yInc == 0 && xInc == -1)
      yInc = 1;
    else if (yInc == 1 && xInc == -1)
      xInc = 0;
    else if (xInc == 0)
      xInc = 1;
    else if (yInc == 1)
      yInc = 0;
    else if (yInc == 0)
      yInc = -1;

  }
  const int maxOverlapAreas = ballPerceptState.ballOnFieldLine ? 3 : (ballPerceptState.ballObstacleOverlap ? 2 : 0);
  const int maxOverlaps = 6;
  return overlapAreas > maxOverlapAreas || overlapCount > maxOverlaps;
}

bool CLIPBallPerceptor::scanFeature(BallFeature &feature, const Vector2f &center, const float &expectedSize, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  Vector2f dir(1.f, 0);
  Vector2f scanPoint = center;
  int newY, lastY;
  float scannedSizeX = 0.f;
  float scannedSizeY = 0.f;
  bool foundEdge;
  float maxSize = expectedSize*1.5f;
  // 1st find center of feature
  for (int i = 0; i < 4; i++)
  {
    newY = lastY = image[static_cast<int>(center.y())][static_cast<int>(center.x())].y;
    scanPoint = center;
    foundEdge = false;
    for (int i = 0; i < maxSize && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 2); i++)
    {
      scanPoint += dir;
      newY = image[static_cast<int>(scanPoint.y())][static_cast<int>(scanPoint.x())].y;
      if (newY - lastY > 30)
      {
        foundEdge = true;
        if (dir.x() > 0)
          feature.center.x() = scanPoint.x();
        if (dir.x() < 0)
          feature.center.x() = (feature.center.x() + scanPoint.x())*0.5f;
        if (dir.y() < 0)
          feature.center.y() = scanPoint.y();
        if (dir.y() > 0)
          feature.center.y() = (feature.center.y() + scanPoint.y())*0.5f;
        break;
      }
      lastY = newY;
    }
    if (!foundEdge)
      return false;
    dir.rotateRight();
  }
  // now check shape of feature (blob)
  dir = Vector2f(1.f, 0);
  maxSize = expectedSize*0.75f;
  for (int i = 0; i < 4; i++)
  {
    newY = lastY = image[static_cast<int>(feature.center.y())][static_cast<int>(feature.center.x())].y;
    scanPoint = feature.center;
    foundEdge = false;
    for (int i = 0; i < maxSize && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 2); i++)
    {
      scanPoint += dir;
      newY = image[static_cast<int>(scanPoint.y())][static_cast<int>(scanPoint.x())].y;
      if (newY - lastY > 30)
      {
        foundEdge = true;
        if (dir.x() > 0)
          scannedSizeX = scanPoint.x();
        if (dir.x() < 0)
        {
          scannedSizeX -= scanPoint.x();
          feature.center.x() = scanPoint.x() + scannedSizeX / 2;
        }
        if (dir.y() < 0)
          scannedSizeY = scanPoint.y();
        if (dir.y() > 0)
        {
          scannedSizeY = scanPoint.y() - scannedSizeY;
          feature.center.y() = scanPoint.y() - scannedSizeY / 2;
        }
        break;
      }
      lastY = newY;
    }
    if (!foundEdge)
      return false;
    dir.rotateRight();
  }
  float size = (scannedSizeX + scannedSizeY) / 2;
  feature.scannedSizeX = scannedSizeX;
  feature.scannedSizeY = scannedSizeY;
  CIRCLE("module:CLIPBallPerceptor:ballFeatures", 
    feature.center.x(), feature.center.y(), size/2, 1, 
    Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::noBrush, ColorRGBA(255, 0, 0));
  return true;
}

int CLIPBallPerceptor::getFittingPoints(const Vector2f &centerPoint, 
  const float &radius, float &distSum, float &maxDist, const bool &upper)
{
  // TODO: instead of return 0, check again if possible overlap..?
  // TODO: round trip not fully working right now
  // maybe go backwards and forwards from start point??
  // was 3.0*(imageHeight/240) (param originated from orange ball on 320x240 image)
  const float minAllowedRadiusDiff = 3.f*(float)(imageHeight / 240);
  float angleSumBetweenPoints = 0.f; //to verify roundness of ball
  int angleSumBetweenPointsCount = 0;
  float lastAngleBetweenPoints = 0.f;
  int count = 0;
  Vector2f firstGoodPoint(0, 0);
  int firstCount = -1;
  int maxCount = 0;
  int countSum = 0;
  int noFarPoints = 0;
  float distSumPositive = 0.f;
  const float anglePerPoint = pi2 / numberOfScanLines;
  distSum = 0;
  maxDist = 0;
  std::vector< Vector2f > &points = (ballPerceptState.ballOnFieldLine ? goodBallHullPoints : ballHullPoints);
  std::vector< Vector2f >::iterator point = points.begin();
  Vector2f lastPoint = *point;
  float angleSum = 0.f;
  bool roundTrip = false;
  size_t pointCount = 0;

  while (maxCount < static_cast<int>(points.size()))
  {
    pointCount++;
    DOT("module:CLIPBallPerceptor:fittingPoints", point->x(), point->y(), ColorRGBA(255, 0, 255), ColorRGBA(255, 0, 255));
    float lpToCenterAngle = (lastPoint - centerPoint).angle();
    float pToCenterAngle = ((*point) - centerPoint).angle();
    if (pointCount == 2)
    {
      lastAngleBetweenPoints = (*point - lastPoint).angle();
    }
    if (pointCount > 1 && radius >= minRadiusInImageForFeatures)
    {
      float angleBetweenPoints = (*point - lastPoint).angle();
      float newAngleDiff = Angle::normalize(lastAngleBetweenPoints - angleBetweenPoints);

      angleSumBetweenPointsCount++;

      if (angleSumBetweenPointsCount > 0)
      {
        angleSumBetweenPoints += newAngleDiff;
        if (std::abs(newAngleDiff) > pi_4 / 2 || std::abs(angleSumBetweenPoints) > pi_4)
        {
          angleSumBetweenPoints = 0.f;
          angleSumBetweenPointsCount = 0;
        }
      }
      
      if (angleSumBetweenPointsCount > 3 && std::abs(angleSumBetweenPoints) < pi_4)
      {
        ballPerceptState.hullState = HullCheckState::edgy;
        return 0;
      }
      lastAngleBetweenPoints = angleBetweenPoints;
    }
    float newAngle = std::min(std::abs(Angle::normalize(lpToCenterAngle - pToCenterAngle)), std::abs(Angle::normalize(pToCenterAngle - lpToCenterAngle)));
    angleSum += newAngle;
    Vector2f diff(point->x() - centerPoint.x(), point->y() - centerPoint.y());
    float dist = diff.norm()-radius;
    if (!roundTrip)
    {
      distSum += std::abs(dist);
      if (dist > 0)
        distSumPositive += dist;
      maxDist = std::max(maxDist, dist);
    }
    if (std::abs(dist) < std::max<float>(std::min<float>(radius / 5.f, minAllowedRadiusDiff), minRadiusInImage / 3)
      && !(newAngle > 0.1f && (newAngle < anglePerPoint / 2 || newAngle > anglePerPoint * 2)))
    {
      if (count == 0 && !roundTrip && firstGoodPoint.x() < 1)
        firstGoodPoint = *point;
      // TODO: check this!
      if (roundTrip && std::abs(firstGoodPoint.x() - point->x() + firstGoodPoint.y() - point->y()) < 0.5f)
      {
        countSum += (count + firstCount > 2) ? count : 0; 
        if (firstCount + count > maxCount)
          maxCount = firstCount + count;
        break;
      }
      lastPoint = *point;
      point++;
      count++;
      if (count > 3) //quarter circle
      {
        if (count > maxCount)
          maxCount = count;
        if ((angleSum < count*anglePerPoint - count*Angle::fromDegrees(10) || angleSum > count*anglePerPoint + count*Angle::fromDegrees(10)))
        {
          ballPerceptState.hullState = HullCheckState::edgy;
          return 0;
        }
      }
    }
    else
    {
      if (firstCount == -1)
        firstCount = count;
      if (count > 3) //quarter circle
        countSum += count - (roundTrip ? firstCount : 0);
      if (roundTrip)
        break;
      angleSum = 0.f;
      lastPoint = *point;
      point++;// = (ballObstacleOverlap ? goodBallHullPoints.erase(point) : ballHullPoints.erase(point));
      count = 0;
      if (dist > 0)
        noFarPoints++;
    }
    if (pointCount == points.size())
    {
      roundTrip = true;
      point = points.begin();
    }
  }
  int modMaxFarPointsOnHull = maxFarPointsOnHull;

  if (ballPerceptState.ballOnFieldLine || ballPerceptState.ballObstacleOverlap)
    modMaxFarPointsOnHull = 2;
  
  if (maxCount >= minFittingPointsForSafeBall && (distSumPositive < radius/2 || ballPerceptState.ballOnFieldLine || ballPerceptState.ballObstacleOverlap))
    return maxCount;
  if (noFarPoints > modMaxFarPointsOnHull && !(ballPerceptState.ballOnFieldLine && countSum > minFittingPointsForSafeBall && maxCount > 6))
    return 0;
  // try to catch small white patches (like penalty cross)
  if (distSum > radius*2 || (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && radius > 20 && (noFarPoints > modMaxFarPointsOnHull || distSum > radius*1.5f || distSumPositive > radius / 1.5f)))
    return 0;
  // TODO: check conditions
  // maxCount < 8 means at least half of ball hull points must be fitting on overlapping ball
  else if (ballPerceptState.ballObstacleOverlap && (distSum > 2 * radius || distSumPositive > radius / 2 || (maxCount < 8 && countSum < minFittingPointsForSafeBall)))
    return 0;
  else
    return maxCount;
}

bool CLIPBallPerceptor::isOnBallSpot(const Vector2i &pos)
{
  for (std::vector<BallSpot>::const_iterator spot = localBallSpots.ballSpots.begin(); spot != localBallSpots.ballSpots.end(); ++spot)
  {
    if ((spot->position-pos).cast<float>().norm() < (3*spot->radiusInImage)/2)
      return true;
  }
  for (std::vector<BallSpot>::const_iterator spot = noBallSpots.begin(); spot != noBallSpots.end(); ++spot)
  {
    if ((spot->position-pos).cast<float>().norm() < (3*spot->radiusInImage)/2)
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

  int maxStepSizeX = imageWidth/8;
  int imgXStart = 4 + maxStepSizeX/4 + random(maxStepSizeX/2);
  int imgXEnd = std::min(imgXStart + maxStepSizeX*2,imageWidth-4);
  int minImgY = std::max<int>(std::min<int>((int)Geometry::calculateHorizon(cameraMatrix,cameraInfo).base.y(),imageHeight/4),4);
  int imgYStart = imageHeight-4;
  int imgYEnd = minImgY;
  Vector2i scanPoint;

  int ballSize = 0;
  
  //int timeSinceLastSeen = (int)theFrameInfo.getTimeSince(std::max(theBallModel.timeWhenLastSeen,theBallModel.timeWhenLastSeenByTeamMate));
  Vector2f pImage = Vector2f::Zero();
  if (Transformation::robotToImage(
    theBallModel.estimate.position, cameraMatrix, cameraInfo, pImage)
    && !image.isOutOfImage(pImage.x(),pImage.y(),4))
  {
    ballSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius, theBallModel.estimate.position.norm());
    if (ballSize > 50 || ballSize < 3)
      return;
    int scanRadius = std::max(ballSize * 5, 50);
    imgXStart = std::max(4, (int)pImage.x() - scanRadius);
    imgXEnd = std::min(imgXStart + scanRadius * 2, imageWidth - 4);
    imgYStart = std::min(imageHeight - 4, (int)pImage.y() + scanRadius);
    imgYEnd = std::max(imgYStart - scanRadius*2, minImgY);
  }
  else 
    return;
  
  if (ballSize <= 0 || imgYStart < imgYEnd || imgXEnd < imgXStart)
    return;
  scanPoint.x() = imgXStart;
  scanPoint.y() = imgYStart;
  int xStepSize = std::min(ballSize*2,std::max((imgXEnd-imgXStart)/10,5));
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
  int expectedBallSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius*2, theBallModel.estimate.position.norm());
  ASSERT(step > 0);
  int lastY = image[y][x].y;

  bool onGreen = false;
  bool onBall = false;
  bool isEdge = false;
  int cb = 0;
  int cr = 0;
  int ballSegmentSize = 0;
  int lastBallSegmentSize = 0;
  while (stepNo < steps && !image.isOutOfImage(x,y,10))
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
        bs.cb = cb/lastBallSegmentSize;
        bs.cr = cr/lastBallSegmentSize;
        localBallSpots.ballSpotsUpper.push_back(bs);
        LINE("module:CLIPBallPerceptor:additionalScan", x, y, 
          x-lastBallSegmentSize*stepX, y-lastBallSegmentSize*stepY,
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

int CLIPBallPerceptor::countGreenOnBallSpot(BallSpot &spot, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const FieldColors &fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;

  if (spot.radiusInImage < 3)
    return 0; // TODO:check

  int greenCount = 0;
  Vector2i scanDir(1, 0);
  Vector2i center(spot.position);
  for (int i = 0; i < 4; i++)
  {
    for (int pointNo = 1; pointNo < spot.radiusInImage-spot.radiusInImage/5; pointNo++)
    {
      Vector2i checkPoint = center + scanDir*pointNo;
      if (image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3))
        continue;
      Image::Pixel p = image[checkPoint.y()][checkPoint.x()];
      if (std::abs(p.cb - spot.cb) > 20 || std::abs(p.cb - spot.cr) > 20)
      {
        greenCount += fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
      }
    }
    scanDir.rotateLeft();
  }
  return greenCount;
}

bool CLIPBallPerceptor::calcBallSpot2016(BallSpot &spot, const bool &upper)
{
  // TODO: shorten/split
  // TODO: far ball with obstacle overlap can be discarded? maybe only if tracking a good ball percept..!
  // TODO: improve ballObstacleOverlap stuff (determine region of overlap?)
  // TODO: too much ball spots are scanned.. check runtime! (improve speed?)
  // TODO: get ball on line.. at least if near
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
    static_cast<int>(expectedSizeInImage)+1 : static_cast<int>(expectedSizeInImage*1.5f));
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
  Vector2i lastBallColorPos(spot.position);
  scannedCenter = spot.position.cast<float>();
  scannedRadius = 0.f;
  int maxY = 0;
  if (spot.found) // when using all star like scanlines, collect y info for yAvg
  {
    for (int i = 0; i < 32; i++)
      ballPerceptState.yHistogram[i] = 0;
  }

  ballPerceptState.ballObstacleOverlap = false; // TODO: as of now, reset for every scan, check this
  bool leftOverlap = false;
  bool rightOverlap = false;
  bool onImageEdge = false;
  bool onShadow = false;
  const int expectedShadowBrightness = std::min(128,fieldColor.fieldColorArray[0].fieldColorOptY);
  Image::Pixel lastP;

  // run maxI scan lines
  for (int i = 0; i < maxI; i++)
  {
    scanPoint = spot.position;
    yJumpLine = 0;
    first = true;
    int divisor = std::max(std::abs(xDir), std::abs(yDir));
    lengthPerStep = std::sqrt(sqr(static_cast<float>(xDir)) + sqr(static_cast<float>(yDir))) / divisor;
    wrongColorCount = 0;
    greenCount = 0;
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
        onShadow = p.y < expectedShadowBrightness;
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
            /*DOT("module:CLIPBallPerceptor:ballFeatures", 
              scanPoint.x, scanPoint.y, ColorRGBA(255, 0, 0, spot.found*255), ColorRGBA(255, 0, 0, spot.found*255));*/
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
      scannedRadius = (scannedRadius + scannedCenter.x() - lastBallColorPos.x())/2;
      scannedCenter.x() = (scannedCenter.x() + lastBallColorPos.x())/2;
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
        if (yDir == 0 && xDir > 0)
          rightOverlap = true;
        if (yDir == 0 && xDir < 0)
          leftOverlap = true;
        ballPerceptState.ballObstacleOverlap = ballPerceptState.ballObstacleOverlap || !onShadow;
        xDir += xInc;
        yDir += yInc;
        continue; // do not add to ball hull points here
      }
      else
        return false;
    }
    if (!onImageEdge)
    {
      // good ball edge point, if green was found, ball color was found and scanline not too long
      if (foundBallColor && scanLength <= expectedSizeInImage && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), minDistFromImageBorder) && (wrongColorCount > 0 && foundGreen))
      {
        goodBallHullPoints.push_back(Vector2f(lastBallColorPos.cast<float>()));
        yJumpSum += yJumpLine;
      }
      if (pointNo > wrongColorCount)
        ballHullPoints.push_back(Vector2f(lastBallColorPos.cast<float>()));
    }
    xDir += xInc;
    yDir += yInc;

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
        Geometry::Circle(scannedCenter, scannedRadius/2),
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
    if (spot.found && yJumpSum < minNumberOfYJumps && scannedRadius/2 > minRadiusInImageForFeatures)
      return false;
    minPercent = 40;
  }
  if (theFieldDimensions.ballType != SimpleFieldDimensions::BallType::any && pointNoSum > 0 && (100 * ballColorCount) / pointNoSum < minPercent)
    return false;

  ColorRGBA centerColor = spot.found ? ColorRGBA(50,50,50) : (spot.centerFound ? ColorRGBA(25,25,25) : ColorRGBA(0,0,0));
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
      else if (ballPerceptState.ballObstacleOverlap)
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
          std::vector<Vector2f > &points = ballPerceptState.ballObstacleOverlap ? goodBallHullPoints : ballHullPoints;
          for (int i = 0; i < (int)points.size(); i++)
            LINE("module:CLIPBallPerceptor:ballScanLines",
            newBallSpot.position.x(), newBallSpot.position.y(),
            points[i].x(), points[i].y(), 1, Drawings::solidPen, ColorRGBA::yellow);
        }
        ballPerceptState.featureCheckNeeded = true;
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
  //const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix; unused
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo; unused
  // now find best fitting circle for all ballHullPoints
  int fittingPoints = 0;
  float maxDist = 0.f;
  float distSum = 0;
  if (ballPerceptState.ballOnFieldLine)
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
  }

  Geometry::Circle testCircle;
  if (Geometry::computeCircleOnFieldLevenbergMarquardt(goodBallHullPoints, testCircle) && testCircle.radius < imageHeight / 4)
  {
    if ((!image.isOutOfImage(testCircle.center.x(), testCircle.center.y(), (int)testCircle.radius)) &&
      (scannedCenter - testCircle.center).norm() > std::max<float>(testCircle.radius, (float)(imageHeight / 80)))
      return false;
    // clip with body contour
    int yClipped = static_cast<int>(testCircle.center.y());
    theBodyContour.clipBottom(static_cast<int>(testCircle.center.x()), yClipped);
    if (yClipped - 2 > testCircle.center.y())
      return false;
    CIRCLE("module:CLIPBallPerceptor:testCircles",
      testCircle.center.x(), testCircle.center.y(), testCircle.radius,
      2, Drawings::solidPen, ColorRGBA(0, 255, 0), Drawings::noBrush, ColorRGBA(0, 255, 0));

    int minFittingPointNo = minFittingPoints;
    if (testCircle.radius > 15 || ballPerceptState.ballObstacleOverlap)
      ballPerceptState.featureCheckNeeded = true;
    // watch out for goal posts on both upper and lower image top border (due to white ball vs goal post or robot feet)
    if ((int)testCircle.center.y() < (int)(lowerImageUpperBorderDistanceFactor*testCircle.radius))
    {
      ballPerceptState.detailedCheckNeeded = true;
      ballPerceptState.featureCheckNeeded = true;
      minFittingPointNo = minFittingPointsForSafeBall;
    }

    minDistOfCenterFromImageBorder = testCircle.radius*1.5f;
    // watch out for half white balls at left/right image border on upper image (lower border should be okay)
    if (upper && (testCircle.center.x() < minDistOfCenterFromImageBorder || testCircle.center.x() > imageWidth - minDistOfCenterFromImageBorder))
      return false;
    spot.position = testCircle.center.cast<int>();
    spot.radiusInImage = testCircle.radius;
    ballPerceptState.circleOK = true;

    fittingPoints = getFittingPoints(testCircle.center, testCircle.radius, distSum, maxDist, upper);
    if (fittingPoints >= minFittingPointNo)
    {
      Geometry::Circle fitCircle;
      // TODO: check this condition.. Why all ballhullpoints? 
      // should maybe just check if that circle is not larger than the other
      // but that should be handled by getFittingPoints()..
      std::vector<Vector2f > &points = ballPerceptState.ballObstacleOverlap ? goodBallHullPoints : ballHullPoints;
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
        if (fittingPoints >= minFittingPointsForSafeBall && (!useFeaturesForGoodLargeBalls ||
          (testCircle.radius < radiusForNoFeatureCheck && 
          !ballPerceptState.ballObstacleOverlap)))
        {
          ballPerceptState.featureCheckNeeded = false;
          ballPerceptState.detailedCheckNeeded = false;
        }
        else if (testCircle.radius < radiusForNoFeatureCheck && !ballPerceptState.ballObstacleOverlap)
        {
          ballPerceptState.featureCheckNeeded = false;
        }
        return true;
      }
      else if (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && !upper && fittingPoints >= minFittingPointsForSafeBall)
      {
        ballPerceptState.featureCheckNeeded = true;
        ballPerceptState.detailedCheckNeeded = true;
        ballPerceptState.ballObstacleOverlap = true;
      }
    }
    else if (ballPerceptState.ballOnFieldLine && !ballPerceptState.ballScannedOnce)
    {
      ballPerceptState.featureCheckNeeded = true;
      ballPerceptState.detailedCheckNeeded = true;
      spot.position = testCircle.center.cast<int>();
      ballPerceptState.ballScannedOnce = true;
      return calcBallSpot2016(spot, upper);
    }
    else 
    {
      if (fittingPoints == 0)
      {
        BallSpot newBallSpot;
        newBallSpot.position = testCircle.center.cast<int>();
        newBallSpot.radiusInImage = testCircle.radius;
        noBallSpots.push_back(newBallSpot);
      }
    }

  }
  return false;
}

MAKE_MODULE(CLIPBallPerceptor, perception)
