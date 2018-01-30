#include "RobotDetector.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"

const float leftSonarMaxAngle = Angle::fromDegrees(70);
const float leftCenterSonarMaxAngle = Angle::fromDegrees(30);
const float rightCenterSonarMaxAngle = Angle::fromDegrees(-30);
const float rightSonarMaxAngle = Angle::fromDegrees(-70);

RobotDetector::RobotDetector()
{
}
  
bool RobotDetector::checkForGreen(const float &x, const float &y, Vector2f scanDir, int maxSteps, int minGreen, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;
  
  int greenCount = 0;
  int step = 0;
  Vector2f checkPoint(x,y);
  while (!image.isOutOfImage(checkPoint.x(),checkPoint.y(),2) && step <= maxSteps && greenCount < minGreen)
  {
    Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
    avgY += p.y;
    avgCb += p.cb;
    avgCr += p.cr;
    pixelCount++;
    if (fieldColor.isPixelFieldColor(p.y,p.cb,p.cr))
      greenCount++;
    checkPoint += scanDir;
    step++;
  }
  if (image.isOutOfImage(checkPoint.x(),checkPoint.y(),2) && step > 0)
    return (100*greenCount)/step >= (50*minGreen)/maxSteps;
  else
  {
    LINE("module:RobotDetector:greenScan",x,y,checkPoint.x(),checkPoint.y(),2,Drawings::solidPen,ColorRGBA::violet);
    return greenCount >= minGreen;
  }
}

bool RobotDetector::scanForRobotWidth(Vector2f &basePoint, float &robotWidth,
  const bool &upper, const bool &imageEndOkLeft, const bool &imageEndOkRight)
{
    
  robotWidth = 0.f;
  Vector2f scanDir((float)(imageWidth / 160), 0.f);
  if (!findEndOfObstacle(basePoint,scanDir,robotWidth,upper, imageEndOkRight))
    return false;
  //LINE("module:RobotDetector:footScanner",basePoint.x,basePoint.y,basePoint.x+robotWidth,basePoint.y,2,Drawings::ps_solid,ColorClasses::orange);
  scanDir.x() = (float)(-imageWidth / 160);
  float widthLeft = 0.f;
  if (!findEndOfObstacle(basePoint,scanDir,widthLeft,upper, imageEndOkLeft))
    return false;
  if (robotWidth < 1 || widthLeft < 1)
    return false;
  //LINE("module:RobotDetector:footScanner",basePoint.x,basePoint.y,basePoint.x-widthLeft,basePoint.y,2,Drawings::ps_solid,ColorClasses::orange);
  basePoint.x() -= widthLeft;
  robotWidth += widthLeft;
  return true;
}

bool RobotDetector::findEndOfObstacle(const Vector2f &basePoint, const Vector2f &scanDir, float &width, const bool &upper, const bool &imageEndOk)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  Vector2f scanPoint = basePoint;
  Vector2f obstacleEnd(0,0);
  Vector2f firstGreen(0,0);
  width = 0.f;
  float stepSize = scanDir.norm();
  bool onObstacle = true;
  fieldColorBuffer.fill(0);
  if (image.isOutOfImage(scanPoint.x(),scanPoint.y(),3))
    return imageEndOk;

  Image::Pixel p = image[(unsigned int)scanPoint.y()][(unsigned int)scanPoint.x()];

  int cr = p.cr, lastCr = p.cr;
  int cb = p.cb, lastCb = p.cb;
  int y = p.y, lastY = p.y;
  
  // mini state machine to detect end of obstacle - only accept obstacles surrounded by green
  while (!image.isOutOfImage(scanPoint.x(),scanPoint.y(),3))
  {
    Image::Pixel p = image[(unsigned int)scanPoint.y()][(unsigned int)scanPoint.x()];

    y = p.y;
    cb = p.cb;
    cr = p.cr;
    fieldColorBuffer.push_front(
      fieldColor.isPixelFieldColor(y,cb,cr) ? 2 : 
      ((onObstacle || std::abs(fieldColor.fieldColorArray[0].fieldColorOptCr-cr)+std::abs(fieldColor.fieldColorArray[0].fieldColorOptCb-cb) > 20) ? 0 : 1));

    if (onObstacle)
    {
      avgY += y;
      avgCb += cb;
      avgCr += cr;
      pixelCount++;
      if (lastY - y > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold/2 && 
        y < fieldColor.fieldColorArray[0].fieldColorOptY + fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
      {
        onObstacle = false;
        fieldColorBuffer.fill(0);
        fieldColorBuffer.push_front(
          fieldColor.isPixelFieldColor(y,cb,cr) ? 2 : 
          ((onObstacle || std::abs(fieldColor.fieldColorArray[0].fieldColorOptCr-cr)+std::abs(fieldColor.fieldColorArray[0].fieldColorOptCb-cb) > 20) ? 0 : 1));
        obstacleEnd = scanPoint;
      }
      else if (fieldColorBuffer.sum() > 8)
      {
        if (obstacleEnd.x() < 1)
          obstacleEnd = basePoint;
        width = (obstacleEnd - basePoint).norm();
        return false;
      }
    }
    else
    {
      if (firstGreen.x() < 0.5f && fieldColorBuffer[0] == 2)
        firstGreen = scanPoint;
      if (fieldColorBuffer.sum() > 8)
      {
        if (firstGreen.x() < 0.5)
          width = (basePoint-firstGreen).norm();
        else
          width = (basePoint-obstacleEnd).norm();
        return true;
      }
      else if (std::abs(lastY-y) > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold/2)
      {
        //onObstacle = true;
        obstacleEnd = scanPoint;
      }
    }
    scanPoint += scanDir;
    width += stepSize;
    lastY = y;
    lastCb = cb;
    lastCr = cr;
  }
  if (image.isOutOfImage(scanPoint.x(),scanPoint.y(),3) && onObstacle == false && firstGreen.x() > 0.5f)
  {
    width = (basePoint-obstacleEnd).norm();
    return true;
  }
  return imageEndOk;
}

bool RobotDetector::lookForOtherLeg(Vector2f &basePoint, float &robotWidth, const float &expectedWidth, const bool &upper)
{
  float otherLegWidthLeft = 0.f;
  float otherLegWidthRight = 0.f;
  float widthComplete = robotWidth;
  Vector2f estimatedLegPosLeft(basePoint.x() - 2*robotWidth/3,basePoint.y()-imageHeight/12);
  Vector2f estimatedLegPosRight(basePoint.x() + 4*robotWidth/3,basePoint.y()-imageHeight/12);

  Vector2f scanDirLeft(-2.f,-0.5f);
  Vector2f scanDirRight(2.f,-0.5f);

  Vector2f newBasePoint(basePoint);

  if (findEndOfObstacle(estimatedLegPosLeft,scanDirLeft,otherLegWidthLeft,upper,false))
  {
    widthComplete = (basePoint.x() - estimatedLegPosLeft.x()) + otherLegWidthLeft + robotWidth;
    newBasePoint.x() = estimatedLegPosLeft.x() - otherLegWidthLeft;
  }
  else if (findEndOfObstacle(estimatedLegPosRight,scanDirRight,otherLegWidthRight,upper,false))
  {
    widthComplete = (estimatedLegPosRight.x() - basePoint.x()) + otherLegWidthRight;
    newBasePoint.x() = basePoint.x();
  }
  else
    return false;
  float widthRatio = widthComplete/(expectedWidth+1);
  if (widthRatio < maxFootWidthDiff &&
    widthRatio > 1/maxFootWidthDiff && 
    widthComplete - robotWidth > robotWidth/2)
  {
    basePoint.x() = newBasePoint.x();
    robotWidth = widthComplete;
    return true;
  }
  return false;
}

RobotEstimate::RobotType RobotDetector::scanForRobotColor(
  const Vector2f &from,
  const Vector2f &scanDir,
  const int &maxSteps,
  Vector2f wristPos,
  const bool &upper)
{
  // TODO: save color-center and scan around to verify
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  int numOwnColor = 0;
  int numOppColor = 0;
  Vector2f checkPoint(from);
  int steps = 0;
  int lastColorY = -1;
  int lastColorCb = -1;
  int lastColorCr = -1;
  bool colorFound = false;
  Vector2f lastColorPoint(-1, -1);
  int wrongColorCount = 0;
  int goodColorCount = 0;

  while (!image.isOutOfImage(checkPoint.x(), checkPoint.y(), 4) && steps < maxSteps && wrongColorCount < 5)
  {
    Image::Pixel p = image[(int)checkPoint.y()][(int)checkPoint.x()];
    if (!fieldColor.isPixelFieldColor(p.y, p.cb, p.cr))
      //&& (p.y < std::min(2 * fieldColor.fieldColorArray[0].fieldColorOptY / 3,80) || std::max(abs(p.cb - 128), abs(p.cr - 128)) > shirtMinColorDiff))
    {
      int diffToOwnColor = std::abs(p.cb - ownColor.y()) + std::abs(p.cr - ownColor.z());
      if (ownColor.x() != 128)
        diffToOwnColor += 60 * (sgn(p.y - 128) != sgn(ownColor.x() - 128));
      int diffToOppColor = std::abs(p.cb - oppColor.y()) + std::abs(p.cr - oppColor.z());
      if (oppColor.x() != 128)
        diffToOppColor += 60 * (sgn(p.y - 128) != sgn(oppColor.x() - 128));
      if (diffToOwnColor > 15 || diffToOppColor > 15)
      {
        if (diffToOwnColor < diffToOppColor)
          numOwnColor += (diffToOwnColor < 60);
        else
          numOppColor += (diffToOppColor < 60);

        colorFound = true;
        lastColorPoint = checkPoint;
        lastColorY = p.y;
        lastColorCb = p.cb;
        lastColorCr = p.cr;
        if (++goodColorCount > 4)
          wrongColorCount = 0;
      }
    }
    else if (colorFound)
    {
      goodColorCount = 0;
      wrongColorCount++;
    }
    checkPoint += scanDir;
    steps++;
  }

  LINE("module:RobotDetector:shirtScan",
    from.x(), from.y(), checkPoint.x(), checkPoint.y(), 3, Drawings::solidPen, ColorRGBA::black);

  if (numOwnColor > 6 && numOppColor < 3)
    return RobotEstimate::teammateRobot;
  else if (numOppColor > 6 && numOwnColor < 3)
    return RobotEstimate::opponentRobot;
  else
    return RobotEstimate::unknownRobot;
}

bool RobotDetector::verifyObstacle(Vector2f &footBase, float &robotWidth, RobotEstimate &robot, const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  Vector2f scanDir(0.f,std::max((float)(imageHeight/240),1.f));
  
  // <<< verify widthByScan to widthByDistance ratio >>>
  Vector2f pointOnField;
  if (!Transformation::imageToRobot(static_cast<int>(footBase.x()+robotWidth/2),static_cast<int>(footBase.y()),
      cameraMatrix,cameraInfo,pointOnField))
    return false;

  // width calculation seems wrong ; 230 pixel width if robot has distance of 300mm? (simulator)
  float obstacleHeightInImage = 1.f;
  Vector3f vectorFromCamera(cameraMatrix.translation);
  vectorFromCamera.x() = pointOnField.x() - vectorFromCamera.x();
	vectorFromCamera.y() = pointOnField.y() - vectorFromCamera.y();
  
  float obstacleWidthFeet = Geometry::getSizeByDistance(cameraInfo, robotFootWidth,vectorFromCamera.norm());
  obstacleHeightInImage = Geometry::getSizeByDistance(cameraInfo, robotHeight,vectorFromCamera.norm());
  float widthRatio = robotWidth/(obstacleWidthFeet+1);
  if (widthRatio > maxFootWidthDiff || (widthRatio < 1/maxFootWidthDiff && !lookForOtherLeg(footBase,robotWidth,obstacleWidthFeet,upper)))
  {
    return false;
  }
  else
  {
    if (!Transformation::imageToRobot(static_cast<int>(footBase.x()+robotWidth/2),static_cast<int>(footBase.y()),
      cameraMatrix,cameraInfo,pointOnField))
    return false;
  }

  LINE("module:RobotDetector:footScanner",
    footBase.x(),footBase.y(),footBase.x()+robotWidth,footBase.y(),2,Drawings::solidPen,ColorRGBA::orange);
  
  // <<< compare with goal percept image coordinates >>>
  bool perceptOnGoal = false;
  
  for (int i = 0; i < theCLIPGoalPercept.numberOfGoalPosts; i++)
  {
    if (upper != theCLIPGoalPercept.goalPosts[i].fromUpper)
      continue;
    int bottomWidth = (int)(theCLIPGoalPercept.goalPosts[i].bottomWidth);
    
    if (footBase.x() <= theCLIPGoalPercept.goalPosts[i].bottomInImage.x()-bottomWidth/2 &&
      footBase.x() + robotWidth >= theCLIPGoalPercept.goalPosts[i].bottomInImage.x()+bottomWidth/2)
    {
      perceptOnGoal = true;
      break;
    }
  }
  if (perceptOnGoal || pointOnField.norm() > maxObstacleDistField || pointOnField.norm() < 200.f)
    return false;

  // <<< check for shirt colors >>>
  // to determine obstacle type or discard obstacle if too much colors
  Vector2f wristPos(0,0);
  scanDir.x() = 0.f;
  scanDir.y() = -std::max((float)(imageHeight/240),1.f);
  robot.robotType = scanForRobotColor(Vector2f(footBase.x()+robotWidth/2.f,footBase.y()-obstacleHeightInImage/2),
                                         scanDir,(int)(obstacleHeightInImage/2.f),wristPos,upper);

  if (robot.robotType == RobotEstimate::noRobot)
    return false;

  // <<< check for green on obstacle by sampling a few pixels >>>
  int stepSize = imageHeight / 240;
  if (useGreenCheck)
  {
    int maxSteps = (int)(robotWidth / (2 * stepSize));
    scanDir.x() = 0.f;
    scanDir.y() = -std::max((float)(stepSize), 1.f);
    scanDir.rotate(pi_4);
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
    scanDir.rotate(-pi_2);
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - 3*obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
    scanDir.rotate(pi_2);
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - 3 * obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - 2 * obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
    scanDir.rotate(-pi_2);
    if (checkForGreen(footBase.x() + robotWidth / 2, footBase.y() - 2 * obstacleHeightInImage / 5, scanDir, maxSteps, maxSteps / 2, upper))
      return false;
  }

  // check for colored obstacle, accept only 'whitish'
  if (useWhiteCheck)
  {
    if (pixelCount > 0)
    {
      avgY /= pixelCount;
      avgCb /= pixelCount;
      avgCr /= pixelCount;
      if (avgY < fieldColor.fieldColorArray[0].fieldColorOptY + fieldColor.fieldColorArray[0].lineToFieldColorYThreshold / 2
        || std::abs(128 - avgCb) > 18
        || std::abs(128 - avgCr) > 18)
      {
        return false;
      }
    }
    else
      return false;
  }

  // are there lower white points? (eg when having upper body part of robot in image (arms..))
  float toBottom = 0.f;
  float toBottom2 = 0.f;
  scanDir.x() = 0.f;
  scanDir.y() = std::max((float)(stepSize),1.f);
  findEndOfObstacle(Vector2f(footBase.x()+robotWidth/4,footBase.y()),scanDir,toBottom,upper,false);
  findEndOfObstacle(Vector2f(footBase.x()+3*robotWidth/4,footBase.y()),scanDir,toBottom2,upper,false);
  if (toBottom > robotWidth/3 || toBottom2 > robotWidth/3)
    return false;

  // enough features on robot?
  int minX = (int)(footBase.x() + robotWidth / 5);
  int maxX = (int)(footBase.x() + 4 * robotWidth / 5);
  int minY = (int)(footBase.y() - obstacleHeightInImage / 2); // scan legs basically
  if (!checkForFeaturesOnRobot(minX, maxX, imageWidth / 80, static_cast<int>(footBase.y()), minY, upper))
    return false;

  // <<< fill RobotsPercept >>>
  robot.locationOnField = pointOnField;
  robot.distance = pointOnField.norm();  
  robot.imageLowerRight.y() = (int)footBase.y();
  robot.imageLowerRight.x() = (int)(footBase.x() + robotWidth);
  robot.imageUpperLeft.x() = (int)(footBase.x());
  robot.imageUpperLeft.y() = (int)(footBase.y() - obstacleHeightInImage);
  robot.validity = 1.f;
  robot.fromUpperImage = upper;
  
  return true;
}

bool RobotDetector::checkForFeaturesOnRobot(const int &minX, const int &maxX, const int &xStep, const int &fromY, const int &toY, const bool &upper)
{
  int scans = 0;
  int featureSum = 0;
  int features = 0;
  int noFeatureLines = 0;
  for (int scanAtX = minX; scanAtX <= maxX; scanAtX += xStep)
  {
    features = 0;
    scans++;
    features = scanForFeatures(Vector2i(scanAtX, fromY), Vector2i(0, -2), Vector2i(scanAtX, toY), upper);
    featureSum += features;
    if (features < 2)
      noFeatureLines++;
  }
  if (scans < 4 || noFeatureLines > 1 || (float)featureSum / (float)scans < featureFactor)
    return false;
  return true;
}

int RobotDetector::scanForFeatures(const Vector2i &basePoint, const Vector2i &scanDir, const Vector2i &stopAt, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  int noFeatures = 0;
  Vector2i scanPoint(basePoint);
  int stepAbs = std::abs(scanDir.y());
  int yDiffSum, cbDiffSum, crDiffSum, lastY, lastCb, lastCr;
  yDiffSum = cbDiffSum = crDiffSum = 0;
  if (image.isOutOfImage(scanPoint.x(), scanPoint.y(), 2))
    return 0;
  else
  {
    Image::Pixel p = image[scanPoint.y()][scanPoint.x()];
    lastY = p.y;
    lastCb = p.cb;
    lastCr = p.cr;
  }
  while (std::abs(scanPoint.y() - stopAt.y()) > stepAbs && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 2))
  {
    Image::Pixel p = image[scanPoint.y()][scanPoint.x()];
    int yDiff = p.y - lastY;
    int cbDiff = p.cb - lastCb;
    int crDiff = p.cr - lastCr;
    if (sgn(yDiffSum) != sgn(yDiff) && abs(yDiff) > 4)
    {
      if (abs(yDiffSum) > 20)
      {
        noFeatures++;
        CIRCLE("module:RobotDetector:features", 
          scanPoint.x(), scanPoint.y(), 2, 2, 
          Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, ColorRGBA(255, 0, 0));
      }
      yDiffSum = yDiff;
    }
    if (sgn(cbDiffSum) != sgn(cbDiff) && std::abs(cbDiff) > 4)
    {
      if (std::abs(cbDiffSum) > 20)
      {
        noFeatures++;
        CIRCLE("module:RobotDetector:features",
          scanPoint.x(), scanPoint.y(), 2, 2,
          Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, ColorRGBA(255, 0, 0));
      }
      cbDiffSum = cbDiff;
    }
    if (sgn(crDiffSum) != sgn(crDiff) && std::abs(crDiff) > 4)
    {
      if (std::abs(crDiffSum) > 20)
      {
        noFeatures++;
        CIRCLE("module:RobotDetector:features",
          scanPoint.x(), scanPoint.y(), 2, 2,
          Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, ColorRGBA(255, 0, 0));
      }
      crDiffSum = crDiff;
    }
      
    scanPoint += scanDir;
    lastY = p.y;
    lastCb = p.cb;
    lastCr = p.cr;
  }
  return noFeatures;
}

void RobotDetector::update(RobotsPercept &theRobotsPercept)
{
  DECLARE_DEBUG_DRAWING("module:RobotDetector:footScanner","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotDetector:shirtScan","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotDetector:outLineScans","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotDetector:greenScan","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotDetector:shirtSpots","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotDetector:features", "drawingOnImage");
  theRobotsPercept.robots.clear();
  localRobotsPercept.robots.clear();
  execute();
  theRobotsPercept = localRobotsPercept;
}

void RobotDetector::execute()
{
  std::vector<ObstacleBasePoints::ObstacleBasePoint>::const_iterator obstacle = theObstacleBasePoints.basePoints.begin();
  while (obstacle != theObstacleBasePoints.basePoints.end())
  {
    // do not scan if obstacle point is within existing robotspercept
    bool segmentOnRobot = false;
    for (std::vector<RobotEstimate>::const_iterator i = localRobotsPercept.robots.begin(); i != localRobotsPercept.robots.end(); ++i)
    {
      if (i->imageLowerRight.x() + 5 > obstacle->pointInImage.x() && i->imageLowerRight.y() + 5 > obstacle->pointInImage.y()
        && i->imageUpperLeft.x() - 5 < obstacle->pointInImage.x() && i->imageUpperLeft.y() - 5 < obstacle->pointInImage.y())
      {
        segmentOnRobot = true;
        break;
      }
    }
    if (!segmentOnRobot)
    {
      const Image &image = obstacle->upperCam ? (Image&)theImageUpper : theImage;
      imageWidth = image.width;
      imageHeight = image.height;
      switch (obstacle->direction)
      {
      case ObstacleBasePoints::ObstacleBasePoint::up:
        checkLowBasePoint(*obstacle);
        break;
      case ObstacleBasePoints::ObstacleBasePoint::right:
        if (allowInCompleteRobots)
          checkLeftBasePoint(*obstacle);
        break;
      case ObstacleBasePoints::ObstacleBasePoint::left:
        if (allowInCompleteRobots)
          checkRightBasePoint(*obstacle);
        break;
      default:
        break;
      }
    }
    obstacle++;
  }

  if (useBumpers)
  {
    addObstacleFromBumpers();
  }

  if (useTeamMatePoses)
  {
    addObstaclesFromTeamMateData();
  }

  //addObstaclesFromColorScan();

  // if parameter is set, add percepts coming from ObstacleModel_D only
  // (calculating image coords only to remove floats later on)
  // used for near obstacles
  /*
  if (params.useSonarWithoutVision)
  {
    RobotsPercept::RobotEstimate newRobot;
    newRobot.robotType = RobotsPercept::RobotEstimate::unknownRobot;
    float angleToRobot = 0.0;
      
    if (theObstacleModel_D.distanceToLeftObstacle < 800 && theObstacleModel_D.distanceToCenterLeftObstacle)
    {
      newRobot.distance = (theObstacleModel_D.distanceToLeftObstacle+theObstacleModel_D.distanceToCenterLeftObstacle)/2;
      angleToRobot = (leftSonarMaxAngle+leftCenterSonarMaxAngle)/2.0;
      newRobot.locationOnField = Pose2D(-pi_D,newRobot.distance*sin(angleToRobot),newRobot.distance*cos(angleToRobot));
      calcImageCoords(newRobot);
      localRobotsPercept.robots.push_back(newRobot);
    }
    else if (theObstacleModel_D.distanceToLeftObstacle < 800)
    {
      newRobot.distance = theObstacleModel_D.distanceToLeftObstacle;
      angleToRobot = (leftSonarMaxAngle);
      newRobot.locationOnField = Pose2D(-pi_D,newRobot.distance*sin(angleToRobot),newRobot.distance*cos(angleToRobot));
      calcImageCoords(newRobot);
      localRobotsPercept.robots.push_back(newRobot);
    }
    if (theObstacleModel_D.distanceToCenterLeftObstacle < 800 && theObstacleModel_D.distanceToCenterRightObstacle < 800)
    {
      newRobot.distance = (theObstacleModel_D.distanceToCenterRightObstacle+theObstacleModel_D.distanceToCenterLeftObstacle)/2;
      float angleToRobot = 0.0;
      newRobot.locationOnField = Pose2D(-pi_D,newRobot.distance*sin(angleToRobot),newRobot.distance*cos(angleToRobot));
      calcImageCoords(newRobot);
      localRobotsPercept.robots.push_back(newRobot);
    }
    if (theObstacleModel_D.distanceToCenterRightObstacle < 800 && theObstacleModel_D.distanceToRightObstacle < 800)
    {
      newRobot.distance = (theObstacleModel_D.distanceToCenterRightObstacle+theObstacleModel_D.distanceToRightObstacle)/2;
      float angleToRobot = (rightSonarMaxAngle+rightCenterSonarMaxAngle)/2;
      newRobot.locationOnField = Pose2D(-pi_D,newRobot.distance*sin(angleToRobot),newRobot.distance*cos(angleToRobot));
      calcImageCoords(newRobot);
      localRobotsPercept.robots.push_back(newRobot);
    }
    else if (theObstacleModel_D.distanceToCenterRightObstacle < 800)
    {
      newRobot.distance = (theObstacleModel_D.distanceToRightObstacle);
      float angleToRobot = rightSonarMaxAngle;
      newRobot.locationOnField = Pose2D(-pi_D,newRobot.distance*sin(angleToRobot),newRobot.distance*cos(angleToRobot));
      calcImageCoords(newRobot);
      localRobotsPercept.robots.push_back(newRobot);
    }
    
  }
  */

  int r1 = 0,r2 = 0;
  while (r1 < (int)localRobotsPercept.robots.size()-1)
  {
    r2 = r1+1;
    if (localRobotsPercept.robots[r1].fromUpperImage == localRobotsPercept.robots[r2].fromUpperImage)
    {
      while (r2 < (int)localRobotsPercept.robots.size() && r2 >= 0 && r1 >= 0 && r2 != r1)
      {
        if (localRobotsPercept.robots[r1].imageLowerRight.x()+25 > localRobotsPercept.robots[r2].imageLowerRight.x()
          && localRobotsPercept.robots[r1].imageUpperLeft.x()-25 < localRobotsPercept.robots[r2].imageUpperLeft.x())
        {
          localRobotsPercept.robots.erase(localRobotsPercept.robots.begin()+r2);
          r2--;
        }
        else if (localRobotsPercept.robots[r1].imageLowerRight.x() < localRobotsPercept.robots[r2].imageLowerRight.x()+25
          && localRobotsPercept.robots[r1].imageUpperLeft.x() > localRobotsPercept.robots[r2].imageUpperLeft.x()-25)
        {
          localRobotsPercept.robots.erase(localRobotsPercept.robots.begin()+r1);
          r1--;
          break;
        }
        r2++;
      }
    }
    r1++;
  }
}

void RobotDetector::checkLowBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle)
{
  bool upper = obstacle.upperCam;
  float obstacleWidthInImage = 1.f;

  const Image &image = upper ? (Image&)theImageUpper : theImage;

  imageWidth = image.width;
  imageHeight = image.height;

  avgY = 0;
  avgCb = 0;
  avgCr = 0;
  pixelCount = 0;
  Vector2f footBase = obstacle.pointInImage;
  int scanWidthHeightDiff = (int)(footBase.y()) / 10;
  footBase.y() -= scanWidthHeightDiff;
  // first, scan for width of robot a little above the given point in image
  if (image.isOutOfImage((int)footBase.x(), (int)footBase.y(), imageHeight / 24) || !scanForRobotWidth(footBase, obstacleWidthInImage, upper,false,false))
  {
    return;
  }

  // check for body contour
  if (!upper)
  {
    int clippedY = (int)obstacle.pointInImage.y();
    theBodyContour.clipBottom((int)footBase.x(), clippedY);
    if (clippedY < (int)obstacle.pointInImage.y())
    {
      return;
    }
  }

  RobotEstimate newRE;
  footBase.y() += scanWidthHeightDiff;
  if (!verifyObstacle(footBase, obstacleWidthInImage, newRE, upper))
  {
    return;
  }

  LINE("module:RobotDetector:feet",
    footBase.x() - obstacleWidthInImage / 2, footBase.y(), footBase.x() + obstacleWidthInImage / 2, footBase.y(),
    2, Drawings::dashedPen, ColorRGBA(127, 0, 255));
  Vector2f wristPos;

  // correct obstacle distance via sonar, if parameter useSonar is set
  /*
  float angleToEstimate = Geometry::angleTo(theRobotPose,newRE.locationOnField.translation);
  if (params.useSonar && newRE.distance < 800)
  {
  if ((angleToEstimate > leftCenterSonarMaxAngle && angleToEstimate < leftSonarMaxAngle && theObstacleModel_D.distanceToLeftObstacle >= 1000) ||
  (angleToEstimate < rightCenterSonarMaxAngle && angleToEstimate > rightSonarMaxAngle && theObstacleModel_D.distanceToRightObstacle >= 1000) ||
  (angleToEstimate <= leftCenterSonarMaxAngle && angleToEstimate > 0 && theObstacleModel_D.distanceToCenterLeftObstacle >= 1000) ||
  (angleToEstimate >= rightCenterSonarMaxAngle && angleToEstimate <= 0 && theObstacleModel_D.distanceToCenterRightObstacle >= 1000)
  )
  continue;
  else
  {
  if (angleToEstimate > leftCenterSonarMaxAngle && angleToEstimate < leftSonarMaxAngle) newRE.distance = theObstacleModel_D.distanceToLeftObstacle;
  else if (angleToEstimate < rightCenterSonarMaxAngle && angleToEstimate > rightSonarMaxAngle) newRE.distance = theObstacleModel_D.distanceToRightObstacle;
  else if (angleToEstimate > 0) newRE.distance = theObstacleModel_D.distanceToCenterLeftObstacle;
  else newRE.distance = theObstacleModel_D.distanceToCenterRightObstacle;
  }

  }
  */

  localRobotsPercept.robots.push_back(newRE);
}

void RobotDetector::checkHighBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle)
{

}

void RobotDetector::checkLeftBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle)
{
  const BodyContour &bodyContour = obstacle.upperCam ? (BodyContour&)theBodyContourUpper : theBodyContour;
  const float expectedRobotWidth = static_cast<float>(imageWidth / 10);
  if (obstacle.pointInImage.x() > imageWidth - imageWidth / 8) // not too far to the image border
    return;
  float widths[6] = { -1.f };
  float avgWidth = 0.f;
  Vector2f basePoint = obstacle.pointInImage;
  basePoint.y() = obstacle.upperCam ? imageHeight / 6.f : imageHeight/4.f;
  basePoint.x() += imageWidth / 10;
  float minX = static_cast<float>(imageWidth);
  float maxX = -1.f;
  float yStep = (imageHeight - basePoint.y()) / 7;
  int i = 0;
  int failedScans = 0;
  for (i = 0; i < 6; i++)
  {
    Vector2f scanPoint = basePoint;
    if (!scanForRobotWidth(scanPoint, widths[i], obstacle.upperCam, false, true))
    {
      failedScans++;
    }
    minX = std::min(scanPoint.x(), minX);
    maxX = std::max(scanPoint.x() + widths[i], maxX);
    int yClipped = static_cast<int>(basePoint.y());
    const int yBase = yClipped;
    bodyContour.clipBottom(static_cast<int>(basePoint.x() + avgWidth / 2), yClipped);
    if (yBase != yClipped)
      return;
    basePoint.y() += yStep;
    avgWidth += widths[i];
  }
  basePoint.y() -= yStep;
  avgWidth /= (i - failedScans);
  int badWidthCount = 0;
  for (int j = 0; j <= i; j++)
  {
    if (widths[j] < expectedRobotWidth)
      badWidthCount++;
  }
  Vector2f colorPoint(std::min(basePoint.x() + avgWidth / 2,imageWidth - imageWidth/12.f), 2 * imageHeight / 3);
  if (badWidthCount < 2 && !checkForGreen(colorPoint.x(),colorPoint.y(),Vector2f(0,-1.f),imageHeight/4,imageHeight/12,obstacle.upperCam))
  {
    minX = std::max(minX, 4.f);
    maxX = std::min(maxX, imageWidth - 4.f);
    RobotEstimate newRE;
    newRE.robotType = obstacle.upperCam ? scanForRobotColor(colorPoint, Vector2f(0.f, -1.f), imageHeight / 5, Vector2f(basePoint), true) : RobotEstimate::RobotType::unknownRobot;
    newRE.locationOnField.translation = Vector2f(1000, 0);
    const CameraMatrix &cm = obstacle.upperCam ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    const CameraInfo &ci = obstacle.upperCam ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    Vector2f angles = Vector2f::Zero();
    Geometry::calculateAnglesForPoint(Vector2f((minX+maxX)/2,colorPoint.y()), cm, ci, angles);
    newRE.locationOnField.translation.rotate(angles.x());
    float guessWidth = (maxX > imageWidth - 8) ? avgWidth + avgWidth / 5 : avgWidth;
    newRE.locationOnField.translation.normalize(Geometry::getDistanceBySize(ci, robotFootWidth, guessWidth));
    newRE.distance = newRE.locationOnField.translation.norm();
    if (newRE.distance < 200 || newRE.locationOnField.translation.x() < 100)
      return;
    newRE.locationOnField.rotation = 0.f;
    newRE.imageUpperLeft = obstacle.upperCam ? Vector2i((int)minX, imageHeight / 4) : Vector2i((int)minX, 4);
    newRE.imageLowerRight = Vector2i((int)maxX, imageHeight - 4);
    newRE.validity = 0.33f;
    newRE.fromUpperImage = obstacle.upperCam;
    localRobotsPercept.robots.push_back(newRE);
  }

}

void RobotDetector::checkRightBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle)
{
  const BodyContour &bodyContour = obstacle.upperCam ? (BodyContour&)theBodyContourUpper : theBodyContour;
  const float expectedRobotWidth = static_cast<float>(imageWidth / 10);
  if (obstacle.pointInImage.x() < imageWidth / 8) // not too far to the image border
    return;
  float widths[6] = { -1.f };
  float avgWidth = 0.f;
  Vector2f basePoint = obstacle.pointInImage;
  basePoint.y() = obstacle.upperCam ? imageHeight / 6.f : imageHeight / 4.f;
  basePoint.x() -= imageWidth / 10;
  float minX = static_cast<float>(imageWidth);
  float maxX = -1.f;
  float yStep = (imageHeight - basePoint.y()) / 7;
  int i = 0;
  int failedScans = 0;
  for (i = 0; i < 6; i++)
  {
    Vector2f scanPoint = basePoint;
    if (!scanForRobotWidth(scanPoint, widths[i], obstacle.upperCam, false, true))
    {
      failedScans++;
    }
    minX = std::min(scanPoint.x(), minX);
    maxX = std::max(scanPoint.x() + widths[i], maxX);
    int yClipped = static_cast<int>(basePoint.y());
    const int yBase = yClipped;
    bodyContour.clipBottom(static_cast<int>(basePoint.x() + avgWidth / 2), yClipped);
    if (yBase != yClipped)
      return;
    basePoint.y() += yStep;
    avgWidth += widths[i];
  }
  basePoint.y() -= yStep;
  avgWidth /= (i - failedScans);
  int badWidthCount = 0;
  for (int j = 0; j <= i; j++)
  {
    if (widths[j] < expectedRobotWidth)
      badWidthCount++;
    else if (widths[j] > 3 * expectedRobotWidth) // TODO: could still be robot on line..?
      return;
  }
  Vector2f colorPoint(std::min(basePoint.x() - avgWidth / 2, imageWidth / 12.f), 2 * imageHeight / 3);
  if (badWidthCount < 2 && !checkForGreen(colorPoint.x(), colorPoint.y(), Vector2f(0, -1.f), imageHeight / 4, imageHeight / 12, obstacle.upperCam))
  {
    minX = std::max(minX, 4.f);
    maxX = std::min(maxX, imageWidth - 4.f);
    RobotEstimate newRE;
    newRE.robotType = obstacle.upperCam ? scanForRobotColor(colorPoint, Vector2f(0.f, -1.f), imageHeight / 5, Vector2f(basePoint), true) : RobotEstimate::RobotType::unknownRobot;
    newRE.locationOnField.translation = Vector2f(1000, 0);
    const CameraMatrix &cm = obstacle.upperCam ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    const CameraInfo &ci = obstacle.upperCam ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    Vector2f angles = Vector2f::Zero();
    Geometry::calculateAnglesForPoint(Vector2f((minX + maxX) / 2, colorPoint.y()), cm, ci, angles);
    newRE.locationOnField.translation.rotate(angles.x());
    float guessWidth = (minX < 8) ? avgWidth + avgWidth / 5 : avgWidth;
    newRE.locationOnField.translation.normalize(Geometry::getDistanceBySize(ci, robotFootWidth, guessWidth));
    newRE.distance = newRE.locationOnField.translation.norm();
    if (newRE.distance < 200 || newRE.locationOnField.translation.x() < 100)
      return;
    newRE.locationOnField.rotation = 0.f;
    newRE.imageUpperLeft = obstacle.upperCam ? Vector2i((int)minX, imageHeight / 4) : Vector2i((int)minX, 4);
    newRE.imageLowerRight = Vector2i((int)maxX, imageHeight - 4);
    newRE.validity = 0.33f;
    newRE.fromUpperImage = obstacle.upperCam;
    localRobotsPercept.robots.push_back(newRE);
  }
}

void RobotDetector::addObstaclesFromGoodSegments()
{
  /*for (int i = 0; i < theObstaclesPercept.numberOfSegments; i++)
  {
    const RobotsPercept::RobotEstimate &obstacle = theObstaclesPercept.segments[i];
    RobotsPercept::RobotEstimate newRE;
  }*/
}

void RobotDetector::calcImageCoords(RobotEstimate &robot, bool upper)
{
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  float heightInImage = Geometry::getSizeByDistance(cameraInfo,550,robot.distance);
  Vector2f imageBase;
  bool ret = Transformation::robotToImage(robot.locationOnField.translation,
    cameraMatrix, cameraInfo, imageBase);

  if (ret == false)
  {
    robot.imageLowerRight.y() = 0;
    robot.imageLowerRight.x() = 0;
    robot.imageUpperLeft.x() = 0;
    robot.imageUpperLeft.y() = 0;
    return;
  }

  robot.imageLowerRight.y() = static_cast<int>(imageBase.y());
  robot.imageLowerRight.x() = static_cast<int>(imageBase.x()) + (int)(heightInImage/5.f);
  robot.imageUpperLeft.x() = static_cast<int>(robot.imageLowerRight.x() - heightInImage/2.5f);
  robot.imageUpperLeft.y() = static_cast<int>(robot.imageLowerRight.y() - heightInImage);

}

void RobotDetector::addObstacleFromBumpers()
{
  // Security check for broken bumpers
	if (theKeySymbols.obstacle_hit)
  {
    RobotEstimate robot;
    robot.fromUpperImage = false; // should cover the whole lower image
    robot.distance = 150; // directly in front of our feet
    robot.locationOnField.translation.x() = 150; // dito
    robot.locationOnField.translation.y() = 0; // don't know...
    robot.locationOnField.rotation = 0; // don't know...
    robot.robotType = RobotEstimate::unknownRobot;
    robot.validity = 0.5f;
    calcImageCoords(robot,false);
    localRobotsPercept.robots.push_back(robot);
  }
}

void RobotDetector::addObstaclesFromTeamMateData()
{
  for (auto &mate : theTeammateData.teammates)
  {
    if (theFrameInfo.getTimeSince(mate.timeWhenLastPacketReceived) < 1000
      && mate.pose.validity > 0.7)
    {
      Pose2f poseRel(Transformation::fieldToRobot(theRobotPose,mate.pose.translation));
      RobotEstimate robot;
      robot.fromUpperImage = poseRel.translation.norm() < 2000; // should be dynamic..eg calc by pose and head angles
      robot.distance = poseRel.translation.norm();
      robot.locationOnField = mate.pose;
      robot.robotType = RobotEstimate::teammateRobot;
      robot.validity = mate.pose.validity;
      calcImageCoords(robot,robot.fromUpperImage);
      localRobotsPercept.robots.push_back(robot);
    }
  }
}

void RobotDetector::addObstaclesFromColorScan()
{
  // TODO
}

MAKE_MODULE(RobotDetector, perception)
