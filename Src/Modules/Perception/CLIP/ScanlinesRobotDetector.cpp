#include "ScanlinesRobotDetector.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Tools/ColorModelConversions.h"
#include "Tools/Debugging/Stopwatch.h"

ScanlinesRobotDetector::ScanlinesRobotDetector()
{
  rejectionReason = RejectionReason::unknown;
}

RobotDetector::SideScanResult ScanlinesRobotDetector::scanToDirection(const Vector2i& start, Vector2f direction, const Vector2f& robotsize, const bool& upper)
{
  // Get correct image and FieldColors
  const Image& img = static_cast<const Image&>(*image);
  const FieldColors& colors = upper ? static_cast<const FieldColors&>(theFieldColorsUpper) : theFieldColors;

  RobotDetector::SideScanResult result;

  const float scanLength = std::max(1.f, std::abs(direction.x()) > std::abs(direction.y()) ? (robotsize.x() / downScanSteps) : (robotsize.y() / sideScanSteps));
  const float maxScanDistance = std::abs(direction.x()) > std::abs(direction.y()) ? 1.5f * robotsize.x() : 1.5f * robotsize.y();

  const float multiplikator =std::max(1.f, scanLength / static_cast<float>(directionScanBufferLength));
  direction *= multiplikator;

  // Bools for upcoming loop
  bool endReached = false;
  int  fieldColorCount = 0;

  // Initialize state
  SideScanState state;

  // Get first pixel and surroundings to check where we start
  for (int i = -1; i <= 1; ++i)
  {
    for (int j = -1; j <= 1; ++j)
    {
      Image::Pixel p;
      int x = start.y() + i;
      int y = start.x() + j;
      if (x > img.height || x < 0)
        continue;
      if (y > img.width || y < 0)
        continue;

      p = img[x][y];
      if (colors.isPixelFieldColor(p.y, p.cb, p.cr)) fieldColorCount++;
    }
  }
  if (fieldColorCount >= 5)
  {
    state = SideScanState::scanMaxDistForFurtherWhite;
    result.startedOnGreen = true;
  }
  else
  {
    state = SideScanState::scanForGreen;
    result.startedOnGreen = false;
  }


  RingBufferWithSum<int> fieldColorBuffer;
  fieldColorBuffer.reserve(directionScanBufferLength);
  fieldColorBuffer.fill(0);
  float maxDistForFurtherWhite = directionScanMaxDistForFurtherWhite * multiplikator;


  Vector2f currentPoint = start.cast<float>();
  Vector2i currentIntegerPoint = start;
  Vector2i lastIntegerPoint = start;

  bool outOfImage = false;

  while (!endReached)
  {
    currentPoint += direction;
    currentIntegerPoint.x() = static_cast<int>(currentPoint.x());
    currentIntegerPoint.y() = static_cast<int>(currentPoint.y());

    Image::Pixel pix;
    bool         curIsFieldColor;

    if (currentIntegerPoint.x() <= 0 || currentIntegerPoint.x() >= img.width ||
        currentIntegerPoint.y() <= 0 || currentIntegerPoint.y() >= img.height)
    {
      outOfImage = true;
    }
    else
    {
      pix = img[currentIntegerPoint.y()][currentIntegerPoint.x()];
      curIsFieldColor = colors.isPixelFieldColor(pix.y, pix.cb, pix.cr);
    }

    switch (state)
    {
      case SideScanState::scanForGreen:
      case SideScanState::scanForSecondGreen:
        if (outOfImage)
        {
          state = SideScanState::obstacleEndsAtImageBorder;
          break;
        }

        if (curIsFieldColor )//|| pix.y < colors.fieldColorArray[0].fieldColorOptY - 20)
        {
          fieldColorBuffer.push_front(1);
        }
        else
        {
          fieldColorBuffer.push_front(0);
        }

        // If of the last 5 scanned pixels at least 3 are field color, we assume that enough green
        // was found
        // Otherwise, continue scanning
        if (fieldColorBuffer.sum() >= directionScanBufferMajority || (currentIntegerPoint - start).norm() > maxScanDistance)
        {
          if (state == SideScanState::scanForGreen)
          {
            result.jumpsToGreen[0] = currentIntegerPoint;
            state = SideScanState::scanMaxDistForFurtherWhite;
          }
          else
          {
            result.jumpsToGreen[1] = currentIntegerPoint;
            state = SideScanState::obstacleEndsAtSecondGreen;
          }

          fieldColorBuffer.fill(0);
        }

        break;

      case SideScanState::scanMaxDistForFurtherWhite:
        if (maxDistForFurtherWhite <= 0)
        {
          state = SideScanState::obstacleEndsAtFirstGreen;
          break;
        }
        maxDistForFurtherWhite -= multiplikator;

        if (curIsFieldColor )//|| pix.y < colors.fieldColorArray[0].fieldColorOptY - 20)
        {
          fieldColorBuffer.push_front(0);
        }
        else
        {
          fieldColorBuffer.push_front(1);
        }

        if (fieldColorBuffer.sum() >= directionScanBufferMajority)
        {
          int whiteCount = 0;
          // Get first pixel and surroundings to check where we start
          for (int i = -1; i <= 1; ++i)
          {
            for (int j = -1; j <= 1; ++j)
            {
              Image::Pixel p;
              int x = currentIntegerPoint.y() + i;
              int y = currentIntegerPoint.x() + j;
              if (x > img.height || x < 0)
                continue;
              if (y > img.width || y < 0)
                continue;

              p = img[x][y];
              if (!colors.isPixelFieldColor(p.y, p.cb, p.cr)) 
                whiteCount++;
            }
          }
          if (whiteCount > 4) {
            result.jumpToWhite = currentIntegerPoint;
            fieldColorBuffer.fill(0);
            state = SideScanState::scanForSecondGreen;
          }
        }

        break;

      case SideScanState::obstacleEndsAtFirstGreen:
        endReached = true;

        if (result.jumpsToGreen[0].x() != -1)
        {
          result.obstacleEnd = result.jumpsToGreen[0];
        }
        else
        {
          result.obstacleEnd = start;
        }

        break;

      case SideScanState::obstacleEndsAtSecondGreen:
        endReached = true;

        if ((result.jumpsToGreen[1] - result.jumpsToGreen[0]).norm() > 3 * direction.norm()) {
          result.obstacleEnd = result.jumpsToGreen[1];
        }
        else {
          result.obstacleEnd = result.jumpsToGreen[0];
        }
        
        break;

      case SideScanState::obstacleEndsAtImageBorder:
        endReached = true;

        // Check where the scan will end
        Geometry::Line scanLine(start, direction);
        Geometry::Line topBorder(Vector2f(0, 0), Vector2f(img.width, 0));
        Geometry::Line bottomBorder(Vector2f(0, img.height), Vector2f(img.width, 0));
        Geometry::Line leftBorder(Vector2f(0, 0), Vector2f(0, img.height));
        Geometry::Line rightBorder(Vector2f(img.width, 0), Vector2f(0, img.height));
        Vector2f       inter(0, 0);
        Vector2i       realInterception(0, 0);

        // Can only be bottom or right border
        if (direction.x() >= 0 && direction.y() >= 0)
        {
          if (Geometry::getIntersectionOfLines(scanLine, rightBorder, inter) && inter.x() == img.width &&
            inter.y() < img.height && inter.y() >= 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
          else if (Geometry::getIntersectionOfLines(scanLine, bottomBorder, inter) &&
            inter.x() < img.width && inter.x() >= 0 && inter.y() == img.height)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
        }
        // Can only be right or top
        else if (direction.x() >= 0 && direction.y() < 0)
        {
          if (Geometry::getIntersectionOfLines(scanLine, rightBorder, inter) && inter.x() == img.width &&
            inter.y() < img.height && inter.y() >= 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
          else if (Geometry::getIntersectionOfLines(scanLine, topBorder, inter) &&
            inter.x() < img.width && inter.x() >= 0 && inter.y() == 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
        }
        // Can only be left or bottom
        else if (direction.x() < 0 && direction.y() >= 0)
        {
          if (Geometry::getIntersectionOfLines(scanLine, leftBorder, inter) && inter.x() == 0 &&
            inter.y() < img.height && inter.y() >= 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
          else if (Geometry::getIntersectionOfLines(scanLine, bottomBorder, inter) &&
            inter.x() < img.width && inter.x() >= 0 && inter.y() == img.height)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
        }
        // Can only be left or top
        else if (direction.x() < 0 && direction.y() < 0)
        {
          if (Geometry::getIntersectionOfLines(scanLine, leftBorder, inter) && inter.x() == 0 &&
            inter.y() < img.height && inter.y() >= 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
          else if (Geometry::getIntersectionOfLines(scanLine, topBorder, inter) &&
            inter.x() < img.width && inter.x() >= 0 && inter.y() == 0)
          {
            realInterception.x() = static_cast<int>(inter.x());
            realInterception.y() = static_cast<int>(inter.y());
          }
        }

        result.obstacleEnd = realInterception;
        break;
    }
  }

  return result;
}

bool ScanlinesRobotDetector::checkCameraMatrixRobotSize(Vector2f& realRobotSize, const Vector2f& basePointInImage, const bool& upper) {
  const CameraMatrix& cam_mat = upper ? theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo&   cam_info = upper ? theCameraInfoUpper : theCameraInfo;
  
  Vector2f rel_pos;
  bool success = Transformation::imageToRobot(basePointInImage, cam_mat, cam_info, rel_pos);
  if (!success) 
    return false;

  Vector3f rel_pos_to_cam(cam_mat.translation);
  rel_pos_to_cam.x() = rel_pos.x() - rel_pos_to_cam.x();
  rel_pos_to_cam.y() = rel_pos.y() - rel_pos_to_cam.y();
  float distance = rel_pos_to_cam.norm();

  realRobotSize.x() = Geometry::getSizeByDistance(cam_info, robotWidth,  distance);
  realRobotSize.y() = Geometry::getSizeByDistance(cam_info, robotHeight, distance);
  return true;
}

void ScanlinesRobotDetector::update(RobotsHypotheses& theRobotsHypotheses)
{
  DECLARE_DEBUG_DRAWING("module:ScanlinesRobotDetector:SideScans:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanlinesRobotDetector:DownScans:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanlinesRobotDetector:SideScans:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:ScanlinesRobotDetector:DownScans:Lower", "drawingOnImage");

  theRobotsHypotheses.robotsHypotheses.clear();
  theRobotsHypotheses.robotsHypothesesUpper.clear();

  STOPWATCH("ScanlinesRobotDetector:execute")
  {
    execute(theRobotsHypotheses);
  }
  
  STOPWATCH("ScanlinesRobotDetector:removeOverlappingRectangles") 
  {
    theRobotsHypotheses.robotsHypothesesUpper = removeOverlappingRectangles(theRobotsHypotheses.robotsHypothesesUpper);
    theRobotsHypotheses.robotsHypotheses = removeOverlappingRectangles(theRobotsHypotheses.robotsHypotheses);
  }

  if (fitHypotheses)
  {
    STOPWATCH("ScanlinesRobotDetector:fitHypothesesToWidthHeigtRatio")
    {
      fitHypothesesToWidthHeigtRatio(theRobotsHypotheses, 2.f, true);
      fitHypothesesToWidthHeigtRatio(theRobotsHypotheses, 2.f, false);
    }
  }
}

void ScanlinesRobotDetector::execute(RobotsHypotheses& theRobotsHypotheses) {
  std::vector<ObstacleBasePoints::ObstacleBasePoint>& localBasePoints = (std::vector<ObstacleBasePoints::ObstacleBasePoint>&)theObstacleBasePoints.basePoints;
  const std::size_t initialSize = localBasePoints.size();

  for (std::size_t runner = 0; runner < localBasePoints.size(); runner++)
  {
    ObstacleBasePoints::ObstacleBasePoint& base = localBasePoints.at(runner);
    rejectionReason = RejectionReason::unknown;

    Vector2f realRobotSize;
    if (!checkCameraMatrixRobotSize(realRobotSize, base.pointInImage, base.upperCam))
    {
      rejectionReason = RejectionReason::firstCameraMatrixCheck;
      drawRejectionReason(base);
      continue;
    }

    image = base.upperCam ? &theImageUpper : &theImage;
    const int imageWidth = image->width;
    const int imageHeight = image->height;
    Vector2i newStart;

    const int margin = 3;
    const int until = std::max(static_cast<int>(0.4 * realRobotSize.y()), (margin + 2 * sideScanSteps));
    const int step = std::max(2, until / sideScanSteps);      
    

    int sumLeft = 0;
    int sumRight = 0;

    int sumXLeft = 0;
    int sumXRight = 0;

    int numberOfScansLeft = 0;
    int numberOfScansRight = 0;

    std::vector<int> lengthes_left;
    std::vector<int> lengthes_right;

    float meanLeft = 0;
    float meanRight = 0;
    float meanXLeft = 0;
    float meanXRight = 0;

    int width = static_cast<int>(realRobotSize.x());
    int height = static_cast<int>(static_cast<float>(width) * robotRatio);

    float maxRobotSize = 1.5f;
    float factorLeft = 0.f;
    float factorRight = 0.f;

    int thresholdForNumberOfScansLeft = 3;
    int thresholdForNumberOfScansRight = 3;

    if (base.direction == ObstacleBasePoints::ObstacleBasePoint::Direction::left) {
      factorRight = (1.f / 6.f);
      factorLeft = (5.f / 6.f);
      thresholdForNumberOfScansRight = 0;
      thresholdForNumberOfScansLeft = static_cast<int>(sideScanSteps / 2.f);
    }
    else if (base.direction == ObstacleBasePoints::ObstacleBasePoint::Direction::right) {
      factorRight = (5.f / 6.f);
      factorLeft = (1.f / 6.f);
      thresholdForNumberOfScansRight = static_cast<int>(sideScanSteps / 2.f);
      thresholdForNumberOfScansLeft = 0;
    }
    else {
      factorRight = (3.f / 6.f);
      factorLeft = (3.f / 6.f);
      thresholdForNumberOfScansRight = static_cast<int>(sideScanSteps / 2.f);
      thresholdForNumberOfScansLeft = static_cast<int>(sideScanSteps / 2.f);
    }

    RobotDetector::SideScanResult right;
    RobotDetector::SideScanResult left;
    int lLeft;
    int lRight;

    for (int i = margin; i <= until; i += step)
    {
      if (base.pointInImage.y() - i < 0) 
        continue;

      newStart.x() = static_cast<int>(base.pointInImage.x());
      newStart.y() = static_cast<int>(base.pointInImage.y()) - i;

      right = scanToDirection(newStart, Vector2f(1, 0), realRobotSize * maxRobotSize * factorRight, base.upperCam);
      left = scanToDirection(newStart, Vector2f(-1, 0), realRobotSize * maxRobotSize * factorLeft, base.upperCam);

      lLeft = static_cast<int>((left.obstacleEnd.cast<float>() - newStart.cast<float>()).norm());
      lRight = static_cast<int>((right.obstacleEnd.cast<float>() - newStart.cast<float>()).norm());

      if (lLeft >= static_cast<int>(realRobotSize.y() / 10.f))
      {
        sumLeft += lLeft;
        lengthes_left.push_back(lLeft);
        sumXLeft += left.obstacleEnd.x();

        numberOfScansLeft++;
          
        COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Upper")
        {
          if (base.upperCam)
            drawSideScan(left, newStart, false, true);
        }

        COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Lower")
        {
          if (!base.upperCam)
            drawSideScan(left, newStart, false, false);
        }
      }
      if (lRight >= static_cast<int>(realRobotSize.y() / 10.f))
      {
        sumRight += lRight;
        lengthes_right.push_back(lRight);
        sumXRight += right.obstacleEnd.x();

        numberOfScansRight++;
        COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Upper")
        {
          if (base.upperCam)
            drawSideScan(right, newStart, true, true);
        }
        COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Lower")
        {
          if (!base.upperCam)
            drawSideScan(right, newStart, true, false);
        }
      }

      COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Upper")
      {
        if (base.upperCam)
          CROSS("module:ScanlinesRobotDetector:SideScans:Upper", newStart.x(), newStart.y(), 4, 1, Drawings::solidPen, ColorRGBA::cyan);
      }

      COMPLEX_DRAWING("module:ScanlinesRobotDetector:SideScans:Lower")
      {
        if (!base.upperCam)
          CROSS("module:ScanlinesRobotDetector:SideScans:Lower", newStart.x(), newStart.y(), 4, 1, Drawings::solidPen, ColorRGBA::cyan);
      }

    }

    if (numberOfScansLeft >= thresholdForNumberOfScansLeft && numberOfScansRight >= thresholdForNumberOfScansRight) {
      if (numberOfScansLeft >= static_cast<int>(sideScanSteps / 2.f)) {
        meanLeft = std::max(0.f, static_cast<float>(sumLeft) / numberOfScansLeft);
        meanXLeft = std::min(static_cast<float>(base.pointInImage.x()), static_cast<float>(sumXLeft) / numberOfScansLeft);
      }
      else {
        meanLeft = 0.f;
        meanXLeft = static_cast<float>(base.pointInImage.x());
      }

      if (numberOfScansRight >= static_cast<int>(sideScanSteps / 2.f)) {
        meanRight = std::max(0.f, static_cast<float>(sumRight) / numberOfScansRight);
        meanXRight = std::max(static_cast<float>(base.pointInImage.x()), static_cast<float>(sumXRight) / numberOfScansRight);
      }
      else {
        meanRight = 0.f;
        meanXRight = static_cast<float>(base.pointInImage.x());
      }

      width = static_cast<int>(meanLeft + meanRight);
      height = static_cast<int>(width * robotRatio);
    }
    else {
      rejectionReason = RejectionReason::tooFewValidSideScans;
      drawRejectionReason(base);
      continue;
    }

    int numberOfDownScans = 0;
    int numberOfZeroDownScans = 0;
    const int marginDown = static_cast<int>(0.25 * width);
    const int downStep = std::max(1, (width - (marginDown * 2)) / downScanSteps);
    std::vector<int> lengthes_down;
    int sumDown = 0;
    int sumYDown = 0;
    float meanDown = 0;
    float meanYDown = base.pointInImage.y();

    RobotDetector::SideScanResult down;
    int lDown;
    for (int i = marginDown; i <= width - marginDown; i += downStep)
    {
      if (base.pointInImage.x() - meanLeft + i < 0) continue;

      newStart.x() = static_cast<int>(base.pointInImage.x() - meanLeft + i);
      newStart.y() = static_cast<int>(base.pointInImage.y());

      down = scanToDirection(newStart, Vector2f(0, 1), realRobotSize, base.upperCam);

      lDown = static_cast<int>((down.obstacleEnd.cast<float>() - newStart.cast<float>()).norm());

      if (lDown > 0) {
        sumDown += lDown;
        lengthes_down.push_back(lDown);
        sumYDown += down.obstacleEnd.y();

        numberOfDownScans++;
      }
      else {
        numberOfZeroDownScans++;
      }
        
      if (base.upperCam)
      {
        COMPLEX_DRAWING("module:ScanlinesRobotDetector:DownScans:Upper")
        {
          drawSideScan(down, newStart, true, true);
          CROSS("module:ScanlinesRobotDetector:DownScans:Upper", newStart.x(), newStart.y(), 4, 1, Drawings::solidPen, ColorRGBA::blue);
        }
      }
      else
      {
        COMPLEX_DRAWING("module:ScanlinesRobotDetector:DownScans:Lower")
        {
          drawSideScan(down, newStart, true, false);
          CROSS("module:ScanlinesRobotDetector:DownScans:Lower", newStart.x(), newStart.y(), 4, 1, Drawings::solidPen, ColorRGBA::blue);
        }
      }

    }

    if (numberOfDownScans > 0) {
      if (numberOfDownScans < numberOfZeroDownScans) {
        numberOfDownScans += numberOfZeroDownScans;
      }

      meanDown = std::max(0.f, static_cast<float>(sumDown) / numberOfDownScans);
      meanYDown = std::max(base.pointInImage.y(), static_cast<float>(sumYDown) / numberOfDownScans);

    }

    if (!checkCameraMatrixRobotSize(realRobotSize, Vector2f(base.pointInImage.x(), meanYDown), base.upperCam)) {
      rejectionReason = RejectionReason::secondCameraMatrixCheck;
      drawRejectionReason(base);
      continue;
    }

    Vector2i upperLeft(0, 0);
    upperLeft.x() = static_cast<int>(meanXLeft);
    if (meanDown > 0.1*realRobotSize.y()) {
      upperLeft.y() = static_cast<int>(meanYDown) - static_cast<int>(realRobotSize.y());
    }
    else {
      upperLeft.y() = static_cast<int>(base.pointInImage.y()) - static_cast<int>(realRobotSize.y());
    }

    Vector2i lowerRight(0, 0);
    lowerRight.x() = upperLeft.x() + width;
    lowerRight.y() = upperLeft.y() + static_cast<int>(realRobotSize.y());

    if (upperLeft.y() < 0)
      upperLeft.y() = 0;
    if (upperLeft.x() < 0)
      upperLeft.x() = 0;

    float maximalX = realRobotSize.x() + (realRobotSize.x() * allowedDeviationFromRealSize);
    float minimalX = realRobotSize.x() - (realRobotSize.x() * allowedDeviationFromRealSize);

    if (!(width > minimalX || width < maximalX))
    {
      rejectionReason = RejectionReason::tooMuchSizeDeviation;
      drawRejectionReason(base);
      continue;
    }
    
    float updateWidth = (realRobotSize.x() - width) / 2.f;
    if (updateWidth > 0 && realRobotSize.x() > width && (upperLeft.x() - updateWidth) >= 0 && (lowerRight.x() + updateWidth) <= imageWidth) {
      upperLeft.x() -= static_cast<int>(updateWidth * factorLeft);
      lowerRight.x() += static_cast<int>(updateWidth * factorRight);
    } 

    RobotsHypotheses::RobotHypothesis a;
    a.upperLeftCorner = upperLeft;
    a.lowerRightCorner = lowerRight;

    ASSERT(a.upperLeftCorner.x() >= 0 && a.upperLeftCorner.x() <= imageWidth && a.upperLeftCorner.y() >= 0 && a.upperLeftCorner.y() <= imageHeight);

    if (base.upperCam)
    {
      rejectionReason = RejectionReason::notRejected;
      theRobotsHypotheses.robotsHypothesesUpper.push_back(a);
    }
    else
    {
      rejectionReason = RejectionReason::notRejected;
      theRobotsHypotheses.robotsHypotheses.push_back(a);
    }
    drawRejectionReason(base);
  }
}

void ScanlinesRobotDetector::drawRejectionReason(const ObstacleBasePoints::ObstacleBasePoint& base) {
  if (rejectionReason != RejectionReason::notRejected) {
    if (base.upperCam) {
      switch (rejectionReason) {
      case RejectionReason::firstCameraMatrixCheck:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Upper", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "firstCameraMatrixCheck");
        break;
      case RejectionReason::secondCameraMatrixCheck:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Upper", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "secondCameraMatrixCheck");
        break;
      case RejectionReason::tooFewValidSideScans:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Upper", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "tooFewValidSideScans");
        break;
      case RejectionReason::tooMuchSizeDeviation:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Upper", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "tooMuchSizeDeviation");
        break;

      case RejectionReason::unknown:
      default:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Upper", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "unknown");
        break;
      }
    }
    else {
      switch (rejectionReason) {
      case RejectionReason::firstCameraMatrixCheck:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Lower", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "firstCameraMatrixCheck");
        break;
      case RejectionReason::secondCameraMatrixCheck:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Lower", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "secondCameraMatrixCheck");
        break;
      case RejectionReason::tooFewValidSideScans:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Lower", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "tooFewValidSideScans");
        break;
      case RejectionReason::tooMuchSizeDeviation:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Lower", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "tooMuchSizeDeviation");
        break;

      case RejectionReason::unknown:
      default:
        DRAWTEXT("representation:ObstacleBasePoints:Image:Lower", base.pointInImage.x() + 10, base.pointInImage.y(), 10, ColorRGBA::black, "unknown");
        break;
      }
    }
  }
}

void ScanlinesRobotDetector::drawSideScan(const RobotDetector::SideScanResult& result, const Vector2i& start, const bool right, const bool upper)
{
  if (upper)
  {
    if (result.jumpsToGreen[0].x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Upper", result.jumpsToGreen[0].x(),
            result.jumpsToGreen[0].y(), 2, 1, Drawings::solidPen, ColorRGBA(0, 255, 127));
    }

    if (result.jumpsToGreen[1].x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Upper", result.jumpsToGreen[1].x(),
            result.jumpsToGreen[1].y(), 2, 1, Drawings::solidPen, ColorRGBA::green);
    }

    if (result.jumpToWhite.x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Upper", result.jumpToWhite.x(), result.jumpToWhite.y(),
            2, 1, Drawings::solidPen, ColorRGBA::white);
    }

    LINE("module:ScanlinesRobotDetector:SideScans:Upper", start.x(), start.y(), result.obstacleEnd.x(),
      result.obstacleEnd.y(), 1, Drawings::solidPen, right ? ColorRGBA::blue : ColorRGBA::red);

  }
  else
  {
    if (result.jumpsToGreen[0].x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Lower", result.jumpsToGreen[0].x(),
            result.jumpsToGreen[0].y(), 2, 1, Drawings::solidPen, ColorRGBA(0, 255, 127));
    }

    if (result.jumpsToGreen[1].x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Lower", result.jumpsToGreen[1].x(),
            result.jumpsToGreen[1].y(), 2, 1, Drawings::solidPen, ColorRGBA::green);
    }

    LINE("module:ScanlinesRobotDetector:SideScans:Lower", start.x(), start.y(), result.obstacleEnd.x(),
         result.obstacleEnd.y(), 1, Drawings::solidPen, right ? ColorRGBA::blue : ColorRGBA::red);

    if (result.jumpToWhite.x() != -1)
    {
      CROSS("module:ScanlinesRobotDetector:SideScans:Lower", result.jumpToWhite.x(), result.jumpToWhite.y(),
            2, 1, Drawings::solidPen, ColorRGBA::white);
    }
  }
}

int ScanlinesRobotDetector::overlap(RobotsHypotheses::RobotHypothesis r1, RobotsHypotheses::RobotHypothesis r2)
{
  using namespace std;

  int dx = min(r1.lowerRightCorner.x(), r2.lowerRightCorner.x()) - max(r1.upperLeftCorner.x(), r2.upperLeftCorner.x());
  int dy = min(r1.lowerRightCorner.y(), r2.lowerRightCorner.y()) - max(r1.upperLeftCorner.y(), r2.upperLeftCorner.y());

  if (dy >= 0 && dx >= 0)
  {
    return dx * dy;
  }

  return 0;

}

std::vector<RobotsHypotheses::RobotHypothesis> ScanlinesRobotDetector::removeOverlappingRectangles(const std::vector<RobotsHypotheses::RobotHypothesis>& input)
{
  std::vector<RobotsHypotheses::RobotHypothesis> tmp = input;
  std::vector<RobotsHypotheses::RobotHypothesis> result;

  auto sortRule = [](const RobotsHypotheses::RobotHypothesis & r1, const RobotsHypotheses::RobotHypothesis & r2) -> bool
  {
    return r1.lowerRightCorner.x() < r2.lowerRightCorner.x();
  };

  std::sort(tmp.begin(), tmp.end(), sortRule);
  std::vector<std::size_t> hasBeenMerged;

  bool hasChanges = true;

  while (hasChanges)
  {
    hasBeenMerged.clear();
    hasChanges = false;

    for (std::size_t i = 0; i < tmp.size(); i++)
    {
      if (std::find(hasBeenMerged.begin(), hasBeenMerged.end(), i) != hasBeenMerged.end())
      {
        continue;
      }

      RobotsHypotheses::RobotHypothesis tmpRect = tmp.at(i);
      int volCurrent = (tmpRect.lowerRightCorner.y() - tmpRect.upperLeftCorner.y()) * (tmpRect.lowerRightCorner.x() - tmpRect.upperLeftCorner.x());

      for (std::size_t j = i + 1; j < tmp.size(); j++)
      {
        if (std::find(hasBeenMerged.begin(), hasBeenMerged.end(), j) != hasBeenMerged.end())
        {
          continue;
        }

        if (tmpRect.lowerRightCorner.x() < tmp.at(j).upperLeftCorner.x())
        {
          break;
        }

        int volNew = (tmp.at(j).lowerRightCorner.y() - tmp.at(j).upperLeftCorner.y()) * (tmp.at(j).lowerRightCorner.x() - tmp.at(j).upperLeftCorner.x());

        int smallerVol = volCurrent < volNew ? volCurrent : volNew;

        if (overlap(tmpRect, tmp.at(j)) > overlappingRatio * smallerVol)
        {
          if (i == j + 1)
          {
            hasBeenMerged.push_back(i);
          }
          hasBeenMerged.push_back(j);
          hasChanges = true;
          tmpRect = volCurrent > volNew ? tmpRect : tmp.at(j);

          tmpRect.upperLeftCorner.x() = std::min(tmpRect.upperLeftCorner.x(), tmp.at(j).upperLeftCorner.x());
          tmpRect.upperLeftCorner.y() = std::min(tmpRect.upperLeftCorner.y(), tmp.at(j).upperLeftCorner.y());
          tmpRect.lowerRightCorner.x() = std::max(tmpRect.lowerRightCorner.x(), tmp.at(j).lowerRightCorner.x());
          tmpRect.lowerRightCorner.y() = std::max(tmpRect.lowerRightCorner.y(), tmp.at(j).lowerRightCorner.y());
        }

      }

      result.push_back(tmpRect);
    }

    tmp = result;
    result.clear();
  }

  return tmp;

}

void ScanlinesRobotDetector::fitHypothesesToWidthHeigtRatio(RobotsHypotheses& robotsHypotheses, float desiredRatio, const bool& upper) {
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  std::vector<RobotsHypotheses::RobotHypothesis>& hypotheses = (upper ? robotsHypotheses.robotsHypothesesUpper : robotsHypotheses.robotsHypotheses);
  for (auto& hypothesis : hypotheses)
  {
    int sizeX = std::abs(hypothesis.upperLeftCorner.x() - hypothesis.lowerRightCorner.x());
    int sizeY = std::abs(hypothesis.upperLeftCorner.y() - hypothesis.lowerRightCorner.y());
    float ratio = static_cast<float>(sizeY) / static_cast<float>(sizeX);
    if (ratio < desiredRatio) {
      hypothesis.upperLeftCorner.y() = hypothesis.lowerRightCorner.y() - static_cast<int>(desiredRatio * sizeX);
    }
    else if (ratio > desiredRatio)
    {
      int length = static_cast<int>(sizeY / desiredRatio);
      hypothesis.upperLeftCorner.x() = hypothesis.lowerRightCorner.x() - static_cast<int>((length + sizeX) / 2);
      hypothesis.lowerRightCorner.x() = hypothesis.upperLeftCorner.x() + length;
    }

    sizeX = std::abs(hypothesis.upperLeftCorner.x() - hypothesis.lowerRightCorner.x());
    sizeY = std::abs(hypothesis.upperLeftCorner.y() - hypothesis.lowerRightCorner.y());
    ASSERT(std::abs(sizeY - 2 * sizeX) < 2);
  }
}

MAKE_MODULE(ScanlinesRobotDetector, perception);
