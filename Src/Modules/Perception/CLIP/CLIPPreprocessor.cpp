/**
* @file CLIPPreprocessor.h
* Implementation of class CLIPPreprocessor
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#include "CLIPPreprocessor.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Tools/Math/Random.h"

CLIPPreprocessor::CLIPPreprocessor()
{
  timeStamp = 0;
  timeStampUpper = 0;
  obstaclePointsLow.reserve(100);
  obstaclePointsHigh.reserve(100);
  obstaclePointsLeft.reserve(100);
  obstaclePointsRight.reserve(100);
  scanLinePixelBuffer.reserve(1280);
  wasReset = false;
  initialized = false;
  lineSizes.resize(Image::maxResolutionHeight);
}

CLIPPreprocessor::~CLIPPreprocessor() {}

void CLIPPreprocessor::createScanLines()
{
  scanLinesHorizontalLower.clear();
  scanLinesHorizontalUpper.clear();
  scanLinesVerticalLower.clear();
  scanLinesVerticalUpper.clear();
  //const int imageSizeFactor = imageHeight / 240; unused
  const int lowerVDistance = theCameraInfo.height / 240 * vScanLineDistanceLower;
  const int upperVDistance = theCameraInfoUpper.height / 240 * vScanLineDistanceUpper;
  const int lowerHDistance = theCameraInfo.height / 240 * hScanLineDistanceLower;
  const int upperHDistance = theCameraInfoUpper.height / 240 * hScanLineDistanceUpper;
  for (int x = 4; x <= theCameraInfo.width - 4; x += lowerVDistance)
  {
    scanLinesVerticalLower.emplace_back(Vector2i(x, theCameraInfo.height - 4), Vector2i(x, 4), 1, true);
  }
  for (int x = 4; x <= theCameraInfoUpper.width - 4; x += upperVDistance)
  {
    scanLinesVerticalUpper.emplace_back(Vector2i(x, theCameraInfoUpper.height - 4), Vector2i(x, 4), 1, true);
  }
  // do not use all, start at horizon!
  for (int y = 4; y <= theCameraInfo.height - 4; y += lowerHDistance)
  {
    scanLinesHorizontalLower.emplace_back(Vector2i(4, y), Vector2i(theCameraInfo.width - 4, y), 1, true);
  }
  for (int y = 4; y <= theCameraInfoUpper.height - 4; y += upperHDistance)
  {
    scanLinesHorizontalUpper.emplace_back(Vector2i(4, y), Vector2i(theCameraInfoUpper.width - 4, y), 1, true);
  }
  scanLinePixelBuffer.clear();
  for (int i = 0; i < theCameraInfoUpper.width * 2; i++) //assuming imageupper width is max value of all image widths/heights
    scanLinePixelBuffer.emplace_back();
  initialized = true;
}

void CLIPPreprocessor::update(CLIPPointsPercept& theCLIPPointsPercept)
{
  wasReset = false;
  execute(false);
  execute(true);
  theCLIPPointsPercept = localCLIPPointsPercept;
}

void CLIPPreprocessor::update(ScanlinesBallSpots& ballSpots)
{
  wasReset = false;
  execute(false);
  execute(true);
  if (sortBallSpots)
  {
    auto sorting_criteria = [](const auto& a, const auto& b)
    {
      return a.position.y() > b.position.y();
    };
    std::sort(localBallSpots.ballSpots.begin(), localBallSpots.ballSpots.end(), sorting_criteria);
    std::sort(localBallSpots.ballSpotsUpper.begin(), localBallSpots.ballSpotsUpper.end(), sorting_criteria);
  }
  ballSpots = localBallSpots;
}

void CLIPPreprocessor::update(ObstacleBasePoints& obstacleBasePoints)
{
  wasReset = false;
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:fieldBorders", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:goalSegments", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:scanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:scanLineSegments:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:scanLineSegmentsRaw:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:scanLineSegments:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:scanLineSegmentsRaw:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:obstaclePoints:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:obstaclePoints:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:fieldHull:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPPreprocessor:fieldHull:Upper", "drawingOnImage");
  execute(false);
  execute(true);
  obstacleBasePoints = localObstacleBasePoints;
}

void CLIPPreprocessor::reset()
{
  localCLIPPointsPercept.reset();
  localBallSpots.ballSpots.clear();
  localBallSpots.ballSpotsUpper.clear();
  localObstacleBasePoints.basePoints.clear();
  obstaclePointsLow.clear();
  obstaclePointsHigh.clear();
  obstaclePointsLeft.clear();
  obstaclePointsRight.clear();
  fieldBorderRight.base = Vector2f::Zero();
  fieldBorderRight.direction = Vector2f::Zero();
  fieldBorderLeft.base = Vector2f::Zero();
  fieldBorderLeft.direction = Vector2f::Zero();
  fieldBorderFront.base = Vector2f::Zero();
  fieldBorderFront.direction = Vector2f::Zero();

  wasReset = true;
}

void CLIPPreprocessor::execute(const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  unsigned actualTimeStamp = upper ? timeStampUpper : timeStamp;
  if (!initialized)
    createScanLines();
  if (actualTimeStamp != image.timeStamp)
  { // Only process the image once!
    //OUTPUT(idText,text,"CLIPPreprocessor::execute(): Is actually processed for upper=" << upper);
    actualTimeStamp = image.timeStamp;
    if (upper)
    {
      timeStampUpper = actualTimeStamp;
    }
    else
    {
      timeStamp = actualTimeStamp;
    }

#ifdef USE_FULL_RESOLUTION
    imageWidth = image.resolutionWidth * 2;
    imageHeight = image.resolutionHeight * 2;
#else
    imageWidth = image.width;
    imageHeight = image.height;
#endif

    if (!image.shouldBeProcessed())
      return;

    // only horizon.base used (thereby assuming it is a straight line to save some calculations)
    horizon = Geometry::calculateHorizon(cameraMatrix, cameraInfo);

    obstaclePointsLow.clear();
    obstaclePointsHigh.clear();
    obstaclePointsLeft.clear();
    obstaclePointsRight.clear();
    if (!wasReset)
      reset();

    if (theFallDownState.state != FallDownState::upright || cameraMatrix.isValid == false)
      return;

    //if (wasReset)
    //  wasReset = false;

    STOPWATCH("CLIPPreprocessor:scanField")
    {
      scanField(upper);
    }

    // === DEBUG DRAWINGS ===

    // obstacle points
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:obstaclePoints:Upper")
    {
      if (upper)
      {
        for (int i = 0; i < (int)obstaclePointsLow.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Upper", obstaclePointsLow[i].x(), obstaclePointsLow[i].y(), ColorRGBA::orange, ColorRGBA::orange);
        for (int i = 0; i < (int)obstaclePointsLeft.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Upper", obstaclePointsLeft[i].x(), obstaclePointsLeft[i].y(), ColorRGBA(80, 200, 200), ColorRGBA(80, 200, 200));
        for (int i = 0; i < (int)obstaclePointsHigh.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Upper", obstaclePointsHigh[i].x(), obstaclePointsHigh[i].y(), ColorRGBA(80, 200, 200), ColorRGBA::yellow);
        for (int i = 0; i < (int)obstaclePointsRight.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Upper", obstaclePointsRight[i].x(), obstaclePointsRight[i].y(), ColorRGBA(80, 200, 200), ColorRGBA::magenta);
      }
    }

    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:obstaclePoints:Lower")
    {
      if (!upper)
      {
        for (int i = 0; i < (int)obstaclePointsLow.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Lower", obstaclePointsLow[i].x(), obstaclePointsLow[i].y(), ColorRGBA::orange, ColorRGBA::orange);
        for (int i = 0; i < (int)obstaclePointsLeft.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Lower", obstaclePointsLeft[i].x(), obstaclePointsLeft[i].y(), ColorRGBA(80, 200, 200), ColorRGBA(80, 200, 200));
        for (int i = 0; i < (int)obstaclePointsHigh.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Lower", obstaclePointsHigh[i].x(), obstaclePointsHigh[i].y(), ColorRGBA(80, 200, 200), ColorRGBA::yellow);
        for (int i = 0; i < (int)obstaclePointsRight.size(); i++)
          DOT("module:CLIPPreprocessor:obstaclePoints:Lower", obstaclePointsRight[i].x(), obstaclePointsRight[i].y(), ColorRGBA(80, 200, 200), ColorRGBA::magenta);
      }
    }

    // scan lines
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:scanLines")
    {
      std::vector<ScanLine>& scanLinesHorizontal = upper ? scanLinesHorizontalUpper : scanLinesHorizontalLower;
      std::vector<ScanLine>& scanLinesVertical = upper ? scanLinesVerticalUpper : scanLinesVerticalLower;
      for (std::vector<ScanLine>::const_iterator line = scanLinesHorizontal.begin(); line != scanLinesHorizontal.end(); ++line)
        LINE("module:CLIPPreprocessor:scanLines", line->from.x(), line->from.y(), line->to.x(), line->to.y(), 1, Drawings::solidPen, ColorRGBA::white);
      for (std::vector<ScanLine>::const_iterator line = scanLinesVertical.begin(); line != scanLinesVertical.end(); ++line)
        LINE("module:CLIPPreprocessor:scanLines", line->from.x(), line->from.y(), line->to.x(), line->to.y(), 1, Drawings::solidPen, ColorRGBA::white);
    }
    // scan line segments
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:scanLineSegments:Upper")
    {
      //y scan lines
      if (upper)
        drawScanLineSegments(true);
    }
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:scanLineSegments:Lower")
    {
      //y scan lines
      if (!upper)
        drawScanLineSegments(false);
    }
    // field hull
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:fieldHull:Lower")
    {
      if (!upper)
      {
        drawFieldHull(false);
      }
    }
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:fieldHull:Upper")
    {
      if (upper)
      {
        drawFieldHull(upper);
      }
    }
  }
}

void CLIPPreprocessor::scanField(const bool& upper)
{
  /*
  * TODO: remove color jump stuff at least if scanning for black-white ball
  * TODO: everything is line, field or obstacle -> merge obstacle segments together to check length -> possible balls
  * TODO: use expected ball size (derive from expected line size)
  */
  if (upper && fieldBorderFront.base.x() != 0)
    return;


  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const FieldColors& fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  const int imageSizeFactor = imageHeight / 240;
  const int hScanLineDistance = upper ? imageSizeFactor * hScanLineDistanceUpper : imageSizeFactor * hScanLineDistanceLower;
  const int vScanLineDistance = upper ? imageSizeFactor * vScanLineDistanceUpper : imageSizeFactor * vScanLineDistanceLower;
  std::vector<ScanLine>& scanLinesHorizontal = upper ? scanLinesHorizontalUpper : scanLinesHorizontalLower;
  std::vector<ScanLine>& scanLinesVertical = upper ? scanLinesVerticalUpper : scanLinesVerticalLower;

  //const int goalScanDistanceFromHorizon = std::max(hScanLineDistance*2,imageHeight/15);

  // compute relative ball max min values only once per image
  minBallCr = std::max(3 * fieldColor.fieldColorArray[0].fieldColorOptCr / 2, ballBaseCrValue);
  minBallCb = (2 * fieldColor.fieldColorArray[0].fieldColorOptCb) / 3;
  maxBallCb = (3 * fieldColor.fieldColorArray[0].fieldColorOptCb) / 2;

  // reset no of end points and field lines
  scanLineVNo = 0;
  scanLineHNo = 0;

  fieldEndPoints.clear();

  obstaclePointsLow.clear();
  obstaclePointsHigh.clear();
  obstaclePointsLeft.clear();
  obstaclePointsRight.clear();

  // create table of field line widths
  int yStart = std::min<int>(std::max<int>(static_cast<int>(horizon.base.y()), 4), imageHeight);
  float lineSizeMax = std::max(Geometry::calculateLineSizePrecise(Vector2i(imageWidth / 2, imageHeight), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth), 2.f);
  float lineSizeMin = std::max<float>(Geometry::calculateLineSizePrecise(Vector2i(imageWidth / 2, yStart), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth), 1.f);
  for (int y = 0; y < yStart; y++)
    lineSizes[y] = 0;
  for (int y = yStart; y < imageHeight; y++)
    lineSizes[y] = lineSizeMin + (lineSizeMax - lineSizeMin) * (static_cast<float>(y - yStart) / (imageHeight - yStart));

  // y scan lines, has to be the same as in initialization (createFieldLines)
  int scanLineNo = 0;
  int maxScanLineNo = (int)scanLinesVertical.size();
  for (int x = 4; x < imageWidth && scanLineNo < maxScanLineNo; x += vScanLineDistance)
  {
    scanLinesVertical[scanLineNo].scanLineSegments.clear();
    processScanLine(scanLinesVertical[scanLineNo], upper);
    scanLineNo++;
  }

  DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:scanLineSegmentsRaw:Upper")
  {
    if (upper)
      drawScanLineSegments(true);
  }

  DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:scanLineSegmentsRaw:Lower")
  {
    if (!upper)
      drawScanLineSegments(false);
  }

  for (int n = 0; n < (int)scanLinesVertical.size(); n++)
  {
    classifyScanLineSegments(scanLinesVertical[n], upper);
    postProcessScanLine(scanLinesVertical[n], upper);
    scanLineVNo++;
  }

  // find field border
  findFieldBorders();


  //goal scan lines (horizontal) not used anymore (since CLIPGoalPerceptor2015).
  /*int goalScanStartY = std::max((int)horizon.base.y-hScanLineDistance*3, 4);
  int goalScanEndY = std::min(std::max((int)horizon.base.y+goalScanDistanceFromHorizon*2, imageHeight/5), imageHeight-goalScanDistanceFromHorizon);
  for (int y = goalScanStartY; y < goalScanEndY; y += hScanLineDistance)
  {
    scanLinesHorizontal.push_back(ScanLine(Vector2<short>(4,y),Vector2<short>(imageWidth-4,y),1,true));
    processScanLine(scanLinesHorizontal.back(),upper,true);
  }*/


  // other horizontal field scan lines
  //for (int y = goalScanEndY+hScanLineDistance; y < imageHeight-hScanLineDistance; y += hScanLineDistance)
  scanLineNo = 0;
  scanLineNoYStart = 0;
  maxScanLineNo = (int)scanLinesHorizontal.size();
  for (int y = 4; y < imageHeight && scanLineNo < maxScanLineNo; y += hScanLineDistance)
  {
    scanLinesHorizontal[scanLineNo].scanLineSegments.clear();
    if (y >= yStart)
      processScanLine(scanLinesHorizontal[scanLineNo], upper);
    else
      scanLineNoYStart++;
    scanLineNo++;
  }

  for (int n = scanLineNoYStart; n < (int)scanLinesHorizontal.size(); n++)
  {
    classifyScanLineSegments(scanLinesHorizontal[n], upper);
    postProcessScanLine(scanLinesHorizontal[n], upper);
    scanLineHNo++;
  }

  if (useObstacleBasePoints)
  {
    //addObstaclePercepts(upper);
    createObstacleBasePoints(upper);
  }
}

void CLIPPreprocessor::createObstacleBasePoints(const bool& upper)
{
  // look for reasonably sized obstacle point clusters
  // TODO: lying obstacles covered? -> also check RobotDetector

  const int lowerVDistance = theCameraInfo.height / 240 * vScanLineDistanceLower;
  const int upperVDistance = theCameraInfoUpper.height / 240 * vScanLineDistanceUpper;
  const int lowerHDistance = theCameraInfo.height / 240 * hScanLineDistanceLower;
  const int upperHDistance = theCameraInfoUpper.height / 240 * hScanLineDistanceUpper;
  const int vScanLineDistance = upper ? upperVDistance : lowerVDistance;
  const int hScanLineDistance = upper ? upperHDistance : lowerHDistance;
  const int maxScanLineDistance = imageWidth / 20;

  // create base points from vertical scan lines
  const int sizeLow = static_cast<int>(obstaclePointsLow.size());
  const int maxYDistanceLow = maxScanLineDistance; // TODO
  const int maxScanLineNumberDistance = 2;
  const int maxXDistanceLow = vScanLineDistance * maxScanLineNumberDistance;
  std::vector<Vector2f> currentObstaclePoints(100);
  // get points on roughly a line close to each other
  for (int firstPoint = 0; firstPoint < sizeLow; firstPoint++)
  {
    int lastFittingSecondPoint = firstPoint;
    int maxY = obstaclePointsLow[firstPoint].y(); //TODO: what about oulier?(shadows..)
    currentObstaclePoints.clear();
    currentObstaclePoints.emplace_back(obstaclePointsLow[firstPoint].cast<float>());
    for (int secondPoint = firstPoint + 1; secondPoint < sizeLow; secondPoint++)
    {
      if (obstaclePointsLow[secondPoint].x() - obstaclePointsLow[lastFittingSecondPoint].x() > maxXDistanceLow)
        break;
      else if (std::abs(obstaclePointsLow[secondPoint].y() - obstaclePointsLow[lastFittingSecondPoint].y()) <= maxYDistanceLow)
      {
        lastFittingSecondPoint = secondPoint;
        currentObstaclePoints.emplace_back(obstaclePointsLow[secondPoint].cast<float>());
        maxY = std::max(maxY, obstaclePointsLow[secondPoint].y());
      }
    }
    firstPoint = lastFittingSecondPoint;
    if (currentObstaclePoints.size() >= obstacleMinPointsLow)
    {
      float avgError = 0.f; // TODO: use?
      float maxError = 0.f; // TODO: use?
      Geometry::Line obsLine = Geometry::calculateLineByLinearRegression(currentObstaclePoints, avgError, maxError);
      ObstacleBasePoints::ObstacleBasePoint obp;
      obp.pointInImage.x() = (currentObstaclePoints[0].x() + currentObstaclePoints.back().x()) / 2.f;
      obp.pointInImage.y() = static_cast<float>(maxY);
      obp.upperCam = upper;
      obp.certain = false;
      obp.direction = ObstacleBasePoints::ObstacleBasePoint::up;
      localObstacleBasePoints.basePoints.push_back(obp);
    }
  }

  // create base points from horizontal scan lines starting at obstacle
  const int sizeLeft = static_cast<int>(obstaclePointsLeft.size());
  const int maxYDistanceSide = hScanLineDistance * maxScanLineNumberDistance; // TODO
  const int maxXDistanceSide = maxScanLineDistance; // TODO
  currentObstaclePoints.clear();
  // get points on roughly a line close to each other
  for (int firstPoint = 0; firstPoint < sizeLeft; firstPoint++)
  {
    int lastFittingSecondPoint = firstPoint;
    int maxY = obstaclePointsLeft[firstPoint].y(); //TODO: what about oulier?(shadows..)
    currentObstaclePoints.clear();
    currentObstaclePoints.emplace_back(obstaclePointsLeft[firstPoint].cast<float>());
    for (int secondPoint = firstPoint + 1; secondPoint < sizeLeft; secondPoint++)
    {
      if (obstaclePointsLeft[secondPoint].y() - obstaclePointsLeft[lastFittingSecondPoint].y() > maxYDistanceSide)
        break;
      else if (std::abs(obstaclePointsLeft[secondPoint].x() - obstaclePointsLeft[lastFittingSecondPoint].x()) <= maxXDistanceSide)
      {
        lastFittingSecondPoint = secondPoint;
        currentObstaclePoints.emplace_back(obstaclePointsLeft[secondPoint].cast<float>());
        maxY = std::max(maxY, obstaclePointsLeft[secondPoint].y());
      }
    }
    firstPoint = lastFittingSecondPoint;
    if (currentObstaclePoints.size() >= obstacleMinPointsSide)
    {
      float avgError = 0.f; // TODO: use?
      float maxError = 0.f; // TODO: use?
      Geometry::Line obsLine = Geometry::calculateLineByLinearRegression(currentObstaclePoints, avgError, maxError);
      ObstacleBasePoints::ObstacleBasePoint obp;
      obp.pointInImage.x() = (currentObstaclePoints[0].x() + currentObstaclePoints.back().x()) / 2.f;
      obp.pointInImage.y() = static_cast<float>(maxY);
      obp.upperCam = upper;
      obp.certain = false;
      obp.direction = ObstacleBasePoints::ObstacleBasePoint::right;
      localObstacleBasePoints.basePoints.push_back(obp);
    }
  }

  // create base points from horizontal scan lines ending at obstacle
  const int sizeRight = static_cast<int>(obstaclePointsRight.size());
  currentObstaclePoints.clear();
  // get points on roughly a line close to each other
  for (int firstPoint = 0; firstPoint < sizeRight; firstPoint++)
  {
    int lastFittingSecondPoint = firstPoint;
    int maxY = obstaclePointsRight[firstPoint].y(); //TODO: what about oulier?(shadows..)
    currentObstaclePoints.clear();
    currentObstaclePoints.emplace_back(obstaclePointsRight[firstPoint].cast<float>());
    for (int secondPoint = firstPoint + 1; secondPoint < sizeRight; secondPoint++)
    {
      if (obstaclePointsRight[secondPoint].y() - obstaclePointsRight[lastFittingSecondPoint].y() > maxYDistanceSide)
        break;
      else if (std::abs(obstaclePointsRight[secondPoint].x() - obstaclePointsRight[lastFittingSecondPoint].x()) <= maxXDistanceSide)
      {
        lastFittingSecondPoint = secondPoint;
        currentObstaclePoints.emplace_back(obstaclePointsRight[secondPoint].cast<float>());
        maxY = std::max(maxY, obstaclePointsRight[secondPoint].y());
      }
    }
    firstPoint = lastFittingSecondPoint;
    if (currentObstaclePoints.size() >= obstacleMinPointsSide)
    {
      float avgError = 0.f; // TODO: use?
      float maxError = 0.f; // TODO: use?
      Geometry::Line obsLine = Geometry::calculateLineByLinearRegression(currentObstaclePoints, avgError, maxError);
      ObstacleBasePoints::ObstacleBasePoint obp;
      obp.pointInImage.x() = (currentObstaclePoints[0].x() + currentObstaclePoints.back().x()) / 2.f;
      obp.pointInImage.y() = static_cast<float>(maxY);
      obp.upperCam = upper;
      obp.certain = false;
      obp.direction = ObstacleBasePoints::ObstacleBasePoint::left;
      localObstacleBasePoints.basePoints.push_back(obp);
    }
  }
}

void CLIPPreprocessor::postProcessScanLine(const ScanLine& scanLine, const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  const bool isVertical = scanLine.from.x() == scanLine.to.x();

  int uncertainty = 0; // if high enough, probably unknown object (i.e. an obstacle)

  bool isUnknownFieldEnd = (isVertical && scanLine.fullScanLine);

  if (scanLine.scanLineSegments.size() == 1)
  {
    return;
  }

  int segNo = 0;
  Vector2f lastGreen(0, 0);
  int size = static_cast<int>(scanLine.scanLineSegments.size());
  float lastLineSize = 100; // just initializing
  bool foundLine = false;
  int lineSizeMultiplicator = isUnknownFieldEnd ? 2 : 3;
  while (segNo < size)
  {
    const ScanLineSegment& seg = scanLine.scanLineSegments.at(segNo);
    if (seg.segmentType == obstacleSegment)
    {
      if (isVertical && segNo + 1 < size)
        uncertainty += 2;
      else
      {
        uncertainty += 3;
        if (!isVertical)
        {
          if (segNo == 0)
            obstaclePointsRight.push_back(seg.endPointInImage.cast<int>());
          else
            obstaclePointsLeft.push_back(seg.startPointInImage.cast<int>());
        }
      }
    }
    else if (seg.segmentType == lineSegment && segNo > 0)
    {
      if (!foundLine)
        lastLineSize = Geometry::calculateLineSizePrecise(seg.startPointInImage.cast<int>(), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth);
      foundLine = true;
      const ScanLineSegment& prevSegment = scanLine.scanLineSegments.at(std::max(0, segNo - 1));
      const ScanLineSegment& nextSegment = scanLine.scanLineSegments.at(std::min(size - 1, segNo + 1));
      uncertainty++;
      if ((prevSegment.segmentType == fieldSegment && nextSegment.segmentType == fieldSegment)
          && (lastLineSize * lineSizeMultiplicator > (seg.endPointInImage - seg.startPointInImage).norm()))
      {
        Vector2f linePoint(((seg.endPointInImage.x() + seg.startPointInImage.x())) / 2.f, ((seg.endPointInImage.y() + seg.startPointInImage.y())) / 2.f);
        lastLineSize = (seg.endPointInImage - seg.startPointInImage).norm();

        addLinePoint(linePoint, lastLineSize, isVertical, upper);
      }
    }
    else if (seg.segmentType == ballSegment)
    {
      // do not add ball spots from horizontal scan that are probably too high (HACK, test!)
      if (((theFieldDimensions.ballType != SimpleFieldDimensions::BallType::orange) || isPixelBallColor(seg.avgY, seg.avgCb, seg.avgCr)) //seg.avgCr*3 > seg.avgCb*4)
          && (!(upper && (scanLine.from.y() == scanLine.to.y() && seg.startPointInImage.y() < imageHeight / 4))))
      {
        const ScanLineSegment& prevSegment = scanLine.scanLineSegments.at(std::max(0, segNo - 1));
        const ScanLineSegment& nextSegment = scanLine.scanLineSegments.at(std::min(size - 1, segNo + 1));
        if ((prevSegment.segmentType == fieldSegment
                && (nextSegment.segmentType == unknownSegment || nextSegment.segmentType == fieldSegment || nextSegment.segmentType == lineSegment || nextSegment.segmentType == obstacleSegment))
            || (nextSegment.segmentType == fieldSegment
                && (prevSegment.segmentType == unknownSegment || prevSegment.segmentType == fieldSegment || prevSegment.segmentType == lineSegment || prevSegment.segmentType == obstacleSegment))
            || (prevSegment.segmentType == ballSegment
                && (nextSegment.segmentType == unknownSegment || nextSegment.segmentType == fieldSegment || nextSegment.segmentType == lineSegment || nextSegment.segmentType == obstacleSegment))
            || (nextSegment.segmentType == ballSegment
                && (prevSegment.segmentType == unknownSegment || prevSegment.segmentType == fieldSegment || prevSegment.segmentType == lineSegment || prevSegment.segmentType == obstacleSegment))
            || (segNo == 0 && (nextSegment.segmentType == fieldSegment || nextSegment.segmentType == lineSegment || nextSegment.segmentType == obstacleSegment)))
        {
          addBallSpot(Vector2f(((seg.endPointInImage.x() + seg.startPointInImage.x())) / 2.f, ((seg.endPointInImage.y() + seg.startPointInImage.y())) / 2.f), seg.avgY, seg.avgCb, seg.avgCr, upper);
        }
      }
    }
    else if (seg.segmentType == fieldSegment)
    {
      lastGreen = seg.endPointInImage;
      uncertainty = std::max(0, uncertainty - 1);
    }
    if (seg.segmentType == unknownSegment && lastGreen.x() != 0)
    {
      uncertainty++;
    }
    if (uncertainty > 2 && lastGreen.x() > 0.5f)
    {
      uncertainty = 0;
      if (isVertical) // from vertical scan
      {
        // adding ballspots if field end next??
        if (seg.segmentType == ballSegment && scanLine.scanLineSegments.at(std::max(0, segNo - 1)).segmentType == fieldSegment)
        {
          addBallSpot(Vector2f(((seg.endPointInImage.x() + seg.startPointInImage.x())) / 2.f, ((seg.endPointInImage.y() + seg.startPointInImage.y())) / 2.f), seg.avgY, seg.avgCb, seg.avgCr, upper);
        }
        if (lastGreen.x() > 0.5f)
        {
          if (seg.segmentType == obstacleSegment && segNo + 1 == size)
          {
            FieldEndPoint newFEP;
            newFEP.imageCoordinates = lastGreen;
            newFEP.inlier = false;
            fieldEndPoints.push_back(newFEP);
          }
          obstaclePointsLow.emplace_back(lastGreen.cast<int>());
        }
        return;
      }
      /*else // from horizontal scan
      {
        obstaclePointsLeft.push_back(Vector2i(lastGreen.cast<int>()));
      }*/
    }
    segNo++;
  }
  if (isUnknownFieldEnd && lastGreen.x() > 0.5
      && (scanLine.scanLineSegments.back().endPointInImage.y() > scanLine.to.y() + 5
          || (scanLine.scanLineSegments.back().segmentType != fieldSegment
              && (scanLine.scanLineSegments.back().endPointInImage - scanLine.scanLineSegments.back().startPointInImage).norm() > lastLineSize)))
  {
    FieldEndPoint newFEP;
    newFEP.imageCoordinates = lastGreen;
    newFEP.inlier = false;
    fieldEndPoints.push_back(newFEP);
  }
}

void CLIPPreprocessor::classifyScanLineSegments(ScanLine& scanLine, const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  const bool isVertical = scanLine.from.x() == scanLine.to.x();

  float expectedBallSize = Geometry::calculateLineSizePrecise((scanLine.to + scanLine.from) / 2, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius * 2);

  int lastAvgY = -1000;
  int lastAvgCb = -1000;
  int lastAvgCr = -1000;
  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    ScanLineSegment& seg = scanLine.scanLineSegments[i];
    float segmentLength = std::max(1.f, (seg.startPointInImage - seg.endPointInImage).norm());
    if (((seg.segmentType == unknownSegment || seg.segmentType == ballSegment)
            && ((float)seg.fieldColorCount / segmentLength > minFieldColorForFieldSegment || abs(lastAvgY - seg.avgY) + abs(lastAvgCb - seg.avgCb) + abs(lastAvgCr - seg.avgCr) < 15))
        || (seg.segmentType == obstacleSegment && (float)seg.fieldColorCount / segmentLength > 0.7f))
    {
      seg.segmentType = fieldSegment;
      lastAvgY = seg.avgY;
      lastAvgCb = seg.avgCb;
      lastAvgCr = seg.avgCr;
    }
  }

  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    if (i != nextIndex && scanLine.scanLineSegments[i].segmentType == nextSegment.segmentType && !(scanLine.scanLineSegments[i].gradientColorEnd && nextSegment.gradientColorStart))
    {
      scanLine.scanLineSegments[nextIndex].startPointInImage = scanLine.scanLineSegments[i].startPointInImage;
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + i);
      i--;
      continue;
    }
  }

  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    ScanLineSegment& seg = scanLine.scanLineSegments[i];
    if (i != nextIndex && seg.segmentType == lineSegment && nextSegment.segmentType == lineSegment)
    {
      scanLine.scanLineSegments[nextIndex].startPointInImage = scanLine.scanLineSegments[i].startPointInImage;
      scanLine.scanLineSegments[nextIndex].segmentType = unknownSegment;
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + i);
      i--;
    }
  }

  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int prevIndex = std::max(0, i - 1);
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& prevSegment = scanLine.scanLineSegments[prevIndex];
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    ScanLineSegment& seg = scanLine.scanLineSegments[i];
    if ((seg.segmentType == unknownSegment) && prevSegment.segmentType == lineSegment && (nextSegment.segmentType == unknownSegment || nextSegment.segmentType == lineSegment))
    {
      scanLine.scanLineSegments[prevIndex].segmentType = unknownSegment;
    }
  }

  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    if (i != nextIndex && scanLine.scanLineSegments[i].segmentType == nextSegment.segmentType)
    {
      scanLine.scanLineSegments[nextIndex].startPointInImage = scanLine.scanLineSegments[i].startPointInImage;
      scanLine.scanLineSegments[nextIndex].gradientColorStart = scanLine.scanLineSegments[i].gradientColorStart;
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + i);
      i--;
      continue;
    }
  }

  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int prevIndex = std::max(0, i - 1);
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& prevSegment = scanLine.scanLineSegments[prevIndex];
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    float segmentLength = (scanLine.scanLineSegments[i].endPointInImage - scanLine.scanLineSegments[i].startPointInImage).norm();
    for (int j = i + 1; j < (int)scanLine.scanLineSegments.size(); j++)
    {
      if (scanLine.scanLineSegments[j].segmentType == unknownSegment || scanLine.scanLineSegments[j].segmentType == lineSegment)
        segmentLength += (scanLine.scanLineSegments[j].endPointInImage - scanLine.scanLineSegments[j].startPointInImage).norm();
      else
        break;
    }

    if (segmentLength == 0)
    {
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + i);
      i--;
      continue;
    }

    if (scanLine.scanLineSegments[i].segmentType == unknownSegment && scanLine.scanLineSegments[i].fieldColorCount < 2
        && (prevSegment.segmentType == fieldSegment || nextSegment.segmentType == fieldSegment))
    {
      Vector2i pImage(scanLine.scanLineSegments[i].startPointInImage.cast<int>());
      if (!isVertical)
        expectedBallSize = Geometry::calculateLineSizePrecise(pImage, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius * 2);

      if (segmentLength < expectedBallSize * 2)
        scanLine.scanLineSegments[i].segmentType = ballSegment;
      else if (segmentLength > expectedBallSize && useObstacleBasePoints)
        scanLine.scanLineSegments[i].segmentType = obstacleSegment;
    }
  }


  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    if (i != nextIndex && scanLine.scanLineSegments[i].segmentType == nextSegment.segmentType && !(scanLine.scanLineSegments[i].gradientColorEnd && nextSegment.gradientColorStart))
    {
      scanLine.scanLineSegments[nextIndex].startPointInImage = scanLine.scanLineSegments[i].startPointInImage;
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + i);
      i--;
      continue;
    }
  }

  // remove remaining unknown segments
  // TODO: check
  for (int i = 0; i < (int)scanLine.scanLineSegments.size(); i++)
  {
    const int prevIndex = std::max(0, i - 1);
    const int nextIndex = std::min((int)scanLine.scanLineSegments.size() - 1, i + 1);
    const ScanLineSegment& prevSegment = scanLine.scanLineSegments[prevIndex];
    const ScanLineSegment& nextSegment = scanLine.scanLineSegments[nextIndex];
    if (scanLine.scanLineSegments[i].segmentType == unknownSegment && (prevSegment.segmentType == fieldSegment && nextSegment.segmentType == fieldSegment))
    {
      scanLine.scanLineSegments[nextIndex].startPointInImage = scanLine.scanLineSegments[prevIndex].startPointInImage;
      scanLine.scanLineSegments.erase(scanLine.scanLineSegments.begin() + prevIndex, scanLine.scanLineSegments.begin() + i);
      i = prevIndex;
    }
  }
}

void CLIPPreprocessor::processScanLine(ScanLine& scanLine, const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors& fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  bool isVertical = (scanLine.from.x() == scanLine.to.x());

  int minY = std::max(scanLine.to.y(), std::min(scanLine.from.y(), static_cast<int>(horizon.base.y())));
  float scanLineLength = (float)std::max(scanLine.from.y() - minY, scanLine.to.x() - scanLine.from.x());

  int imageX = scanLine.from.x();
  int imageY = scanLine.from.y();
  int stepSize = scanLine.stepSize;
  int zero = 0;
  int& stepSizeY = isVertical ? stepSize : zero;
  int& stepSizeX = isVertical ? zero : stepSize;
  int xNorm = isVertical ? 0 : 1;
  int yNorm = isVertical ? 1 : 0;
  pixelCount = 0;

  bool isUnknownFieldEnd = (isVertical && scanLine.fullScanLine);
  bool foundField = false;
  float lastLineSize = (float)(imageWidth / 6);

#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel(imageX, imageY, &p);
#else
  Image::Pixel p = image[imageY][imageX];
#endif
  for (int i = 0; i < 3; i++)
    scanLinePixelBuffer[i] = p;
  pixelCount = 2; //0-2 have been filled above
  fieldColorBuffer.fill(0);

  int colorDiff = 0;
  int yDiff, yDiff2;
  yDiff = yDiff2 = 0;

  int segmentLength = 0;

  lastLineSize = lineSizes[scanLine.from.y()];
  float lineSizeMin = lineSizes[minY];
  const int stepSizeMin = std::max<int>(1, std::min<int>(imageHeight / 30, (int)lineSizeMin / 4));
  const int stepSizeMax = std::max<int>(stepSizeMin, std::min<int>(imageHeight / 30, (int)lastLineSize / 4));
  stepSize = stepSizeMax;

  int stepSizeLine = std::max(1, (int)lastLineSize / 20);
  int& stepSizeLineY = isVertical ? stepSizeLine : zero;
  int& stepSizeLineX = isVertical ? zero : stepSizeLine;

  scanLine.scanLineSegments.emplace_back(ScanLineSegment((float)imageX, (float)imageY, 0, lastColorDiff() > minColorDiff, unknownSegment));

  bool isGradientYUp = false;

  bool isGradientColor = false;

  int segmentNo = 0;

  while (scanLine.to.x() - imageX >= 0 && imageY >= minY)
  {
    if (scanLine.scanLineSegments.size() > 48) // 50 were reserved
    {
      //OUTPUT(idText,text,"too many scan line segments");
      scanLine.scanLineSegments.pop_back();
      segmentNo--;
      return;
    }
#ifdef USE_FULL_RESOLUTION
    image.getPixel(imageX, imageY, &p);
#else
    p = image[imageY][imageX];
#endif
    scanLinePixelBuffer[++pixelCount] = p;
    ASSERT(pixelCount > 0 && pixelCount < INT_MAX);
    ASSERT(static_cast<unsigned>(pixelCount) < scanLinePixelBuffer.size());

    float currentscannedRatio = (imageX - scanLine.from.x() + imageY - minY) / (scanLineLength + 1);
    stepSize = (int)(stepSizeMin + currentscannedRatio * (float)(stepSizeMax - stepSizeMin));

    ASSERT(pixelCount <= INT_MAX);
    yDiff = scanLinePixelBuffer[static_cast<int>(pixelCount) - 1].y - scanLinePixelBuffer[pixelCount].y;
    yDiff2 = scanLinePixelBuffer[static_cast<int>(pixelCount) - 2].y - scanLinePixelBuffer[pixelCount].y;
    colorDiff = std::max(lastColorDiff(), lastColorDiff2());
    fieldColorBuffer.push_front(fieldColor.isPixelFieldColor(scanLinePixelBuffer[pixelCount].y, scanLinePixelBuffer[pixelCount].cb, scanLinePixelBuffer[pixelCount].cr) ? 1 : 0);
    foundField = foundField || (fieldColorBuffer.sum() > 4 && (100 * scanLine.scanLineSegments[segmentNo].fieldColorCount) / (segmentLength + 1) > 50);

    // main state machine
    if (scanLine.scanLineSegments[segmentNo].segmentType == unknownSegment)
    {
      isGradientYUp = foundField && ((-yDiff) > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold || (-yDiff2) > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold);
      isGradientColor = (colorDiff > minColorDiff);

      if (isGradientYUp)
      {
        scanLine.scanLineSegments[segmentNo].gradientColorEnd = isGradientColor;
        // search for more precise line start
        // TODO: use full resolution here?
        int maxY = scanLinePixelBuffer[pixelCount].y;
        int localMinY = maxY;
        int maxScanWidth = std::max((3 * stepSize) / 2, 3);
        stepSizeLine = std::max(1, stepSize / 4);
        int counter = 0;
        bool foundHigh, foundLow;
        foundHigh = foundLow = false;
        bool foundEnd = false;
        int lastY = scanLinePixelBuffer[pixelCount].y;
        int linePosX, linePosY, linePosLowX, linePosLowY, linePosHighX, linePosHighY;

        linePosX = linePosLowX = linePosHighX = imageX - xNorm;
        linePosY = linePosLowY = linePosHighY = imageY + yNorm;

        while (counter <= maxScanWidth && !image.isOutOfImage(linePosX, linePosY, 2))
        {
#ifdef USE_FULL_RESOLUTION
          image.getPixel(linePosX, linePosY, &p);
#else
          p = image[linePosY][linePosX];
#endif
          lastY = p.y;
          if (foundHigh && foundLow)
          {
            if (localMinY + 5 >= lastY)
            {
              foundEnd = true;
              break;
            }
            else
            {
              localMinY = std::min(lastY, localMinY);
              linePosLowX = linePosX;
              linePosLowY = linePosY;
            }
          }
          if (lastY < maxY)
          {
            foundHigh = true;
          }
          else
          {
            maxY = lastY;
            foundHigh = false;
            foundLow = false;
            localMinY = maxY;
          }
          if (foundHigh && localMinY > lastY)
            localMinY = lastY;
          if (foundHigh && !foundLow && maxY - localMinY > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
          {
            foundLow = true;
            linePosHighX = linePosX + stepSizeLineX;
            linePosHighY = linePosY - stepSizeLineY;
            linePosLowX = linePosX;
            linePosLowY = linePosY;
          }

          linePosX -= stepSizeLineX;
          linePosY += stepSizeLineY;
          counter += stepSizeLine;
        }
        if (segmentLength > 4 && (scanLine.scanLineSegments[segmentNo].avgCr / segmentLength) > ballBaseCrValue)
        {
          scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
          scanLine.scanLineSegments[segmentNo].endPointInImage.x() = (float)imageX;
          scanLine.scanLineSegments[segmentNo].endPointInImage.y() = (float)imageY;
          scanLine.scanLineSegments.emplace_back((float)imageX, (float)imageY, 0, isGradientColor, unknownSegment);
          segmentNo++;
          segmentLength = 0;
          continue;
        }
        if (counter <= maxScanWidth)
        {
          scanLine.scanLineSegments[segmentNo].endPointInImage.x() = (float)linePosLowX;
          scanLine.scanLineSegments[segmentNo].endPointInImage.y() = (float)linePosLowY;
          if (segmentLength < stepSize * 2)
          {
            scanLine.scanLineSegments.pop_back();
            segmentNo--;
            if (segmentNo >= 0 && scanLine.scanLineSegments[segmentNo].gradientColorEnd)
              isGradientColor = true;
          }
          else if (segmentLength > 0)
          {
            scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
            scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
            scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
          }

          scanLine.scanLineSegments.emplace_back(((float)(linePosLowX + linePosHighX)) / 2.f, ((float)(linePosLowY + linePosHighY)) / 2.f, 0, isGradientColor, lineSegment);
          segmentNo++;
          segmentLength = 0;

          // scan for line end;
          maxScanWidth = std::max((int)(2.f * lastLineSize), 5);
          foundHigh = foundLow = foundEnd = false;
          linePosX = linePosLowX = linePosHighX = imageX;
          linePosY = linePosLowY = linePosHighY = imageY;
          counter = 0;
          localMinY = maxY;
          int fieldColorCount = 0;

          while (counter <= maxScanWidth && !image.isOutOfImage(linePosX, linePosY, 2))
          {
#ifdef USE_FULL_RESOLUTION
            image.getPixel(linePosX, linePosY, &p);
#else
            p = image[(int)linePosY][(int)linePosX];
#endif
            lastY = p.y;
            scanLinePixelBuffer[++pixelCount] = p;
            scanLine.scanLineSegments[segmentNo].gradientColorEnd = lastColorDiff() > minColorDiff;

            if (foundHigh && foundLow)
            {
              if (localMinY + 5 >= lastY)
              {
                foundEnd = true;
                break;
              }
              else
              {
                localMinY = std::min(lastY, localMinY);
                linePosLowX = linePosX;
                linePosLowY = linePosY;
              }
            }
            if (lastY < maxY)
            {
              foundHigh = true;
            }
            else
            {
              maxY = lastY;
              foundHigh = false;
              foundLow = false;
              localMinY = maxY;
            }
            if (foundHigh && localMinY > lastY)
              localMinY = lastY;
            if (foundHigh && !foundLow && maxY - localMinY > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
            {
              foundLow = true;
              linePosHighX = linePosX - stepSizeLineX;
              linePosHighY = linePosY + stepSizeLineY;
              linePosLowX = linePosX;
              linePosLowY = linePosY;
            }

            linePosX += stepSizeLineX;
            linePosY -= stepSizeLineY;
            counter += stepSizeLine;
            fieldColorCount += stepSizeLine * fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
          }
          if (foundEnd)
          {
            Vector2f endPoint(((float)(linePosLowX + linePosHighX)) / 2.f, ((float)(linePosLowY + linePosHighY)) / 2.f);
            scanLine.scanLineSegments[segmentNo].endPointInImage = endPoint;
            lastLineSize = lineSizes[(int)endPoint.y()];
            stepSizeLine = (int)std::max((lastLineSize / 20), 1.f);

            scanLine.scanLineSegments.emplace_back(ScanLineSegment(endPoint.x() + xNorm, endPoint.y() - yNorm, (fieldColorBuffer[0] == 1) * stepSize, isGradientColor, unknownSegment));
            segmentNo++;
            segmentLength = stepSize;

            imageX = (int)endPoint.x() + stepSizeX;
            imageY = (int)endPoint.y() - stepSizeY;
            if ((scanLine.to.x() - imageX) + (imageY - scanLine.to.y()) < stepSize || image.isOutOfImage(imageX, imageY, stepSize))
            {
              scanLine.scanLineSegments.pop_back();
              segmentNo--;
              return;
            }
#ifdef USE_FULL_RESOLUTION
            image.getPixel(imageX, imageY, &p);
#else
            p = image[imageY][imageX];
#endif
            scanLinePixelBuffer[++pixelCount] = p;
            scanLine.scanLineSegments[segmentNo].avgY = stepSize * scanLinePixelBuffer[pixelCount].y;
            scanLine.scanLineSegments[segmentNo].avgCb = stepSize * scanLinePixelBuffer[pixelCount].cb;
            scanLine.scanLineSegments[segmentNo].avgCr = stepSize * scanLinePixelBuffer[pixelCount].cr;
            continue;
          }
          else
          {
            scanLine.scanLineSegments[segmentNo].segmentType = unknownSegment;
            scanLine.scanLineSegments[segmentNo].fieldColorCount = fieldColorCount;
            if (isUnknownFieldEnd && foundField && fieldColorCount < std::max(5, segmentLength / 2))
            {
              if (!useObstacleBasePoints)
              {
                return;
              }
              scanLine.scanLineSegments[segmentNo].endPointInImage = Vector2f((float)(linePosX + xNorm), (float)(linePosY - yNorm));
              scanLine.scanLineSegments[segmentNo].gradientColorEnd = isGradientColor;
              if (segmentLength > 0)
              {
                scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
                scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
                scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
              }
              scanLine.scanLineSegments[segmentNo].segmentType = obstacleSegment;
              return;
            }
            imageX = linePosX;
            imageY = linePosY;
            // TODO: 'continue' here?
          }
        }
        else // did not find line start, should not happen..
        {
          scanLine.scanLineSegments[segmentNo].endPointInImage.x() = (float)imageX;
          scanLine.scanLineSegments[segmentNo].endPointInImage.y() = (float)imageY;
          imageX += stepSizeX;
          imageY += stepSizeY;
        }
      }
      else if (segmentLength < imageHeight / 80)
      {
      }
      else if (yDiff > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold / 2 || isGradientColor)
      {
        scanLine.scanLineSegments[segmentNo].endPointInImage = Vector2f((float)(imageX - xNorm), (float)(imageY + yNorm));
        scanLine.scanLineSegments[segmentNo].gradientColorEnd = isGradientColor;

        if (segmentLength < stepSize * 2)
        {
          scanLine.scanLineSegments.pop_back();
          segmentNo--;
          if (segmentNo >= 0 && scanLine.scanLineSegments[segmentNo].gradientColorEnd)
            isGradientColor = true;
        }
        else if (segmentLength > 0)
        {
          scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
          if ((100 * scanLine.scanLineSegments.back().fieldColorCount) / segmentLength < 25
              && !(scanLine.scanLineSegments.back().gradientColorStart && scanLine.scanLineSegments.back().gradientColorEnd))
          {
            scanLine.scanLineSegments.back().segmentType = unknownSegment;
            if (isUnknownFieldEnd && foundField && segmentLength > Geometry::calculateLineSizePrecise(Vector2i(imageX, imageY), cameraMatrix, cameraInfo, theFieldDimensions.ballRadius * 2))
            {
              if (useObstacleBasePoints)
              {
                scanLine.scanLineSegments.back().segmentType = obstacleSegment;
              }
              return;
            }
          }
        }
        scanLine.scanLineSegments.emplace_back(ScanLineSegment((float)imageX, (float)imageY, 0, isGradientColor, unknownSegment));
        segmentLength = 0;
        segmentNo++;
      }
    }
    else // possible obstacle segment..
    {
      if (fieldColorBuffer.sum() > 4)

      {
        scanLine.scanLineSegments[segmentNo].endPointInImage = Vector2f((float)(imageX - xNorm * stepSize * 2), (float)(imageY + yNorm * stepSize * 2));
        if (segmentLength > 0)
        {
          scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
          scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
        }
        scanLine.scanLineSegments.emplace_back(ScanLineSegment((float)imageX, (float)imageY, fieldColorBuffer.sum() * stepSize, isGradientColor, unknownSegment));
        segmentNo++;
        segmentLength = 0;
      }
    }

    segmentLength += stepSize;
    scanLine.scanLineSegments[segmentNo].avgY += stepSize * scanLinePixelBuffer[pixelCount].y;
    scanLine.scanLineSegments[segmentNo].avgCb += stepSize * scanLinePixelBuffer[pixelCount].cb;
    scanLine.scanLineSegments[segmentNo].avgCr += stepSize * scanLinePixelBuffer[pixelCount].cr;
    scanLine.scanLineSegments[segmentNo].fieldColorCount += fieldColorBuffer[0] * stepSize;
    imageX += stepSizeX;
    imageY -= stepSizeY;
  }
  scanLine.scanLineSegments[segmentNo].endPointInImage = Vector2f((float)imageX, (float)imageY);
  scanLine.scanLineSegments[segmentNo].gradientColorEnd = scanLine.scanLineSegments[segmentNo].gradientColorStart;
  if (segmentLength > 0)
  {
    scanLine.scanLineSegments[segmentNo].avgY /= segmentLength;
    scanLine.scanLineSegments[segmentNo].avgCb /= segmentLength;
    scanLine.scanLineSegments[segmentNo].avgCr /= segmentLength;
  }
}

void CLIPPreprocessor::addBallSpot(const Vector2f& point, const int& y, const int& cb, const int& cr, const bool& upper)
{
  ScanlinesBallSpot newBallSpot;
  newBallSpot.position.x() = (int)point.x();
  newBallSpot.position.y() = (int)point.y();
  //newBallSpot.y = y; // TODO BH0215 port: add y back in?
  newBallSpot.cb = cb;
  newBallSpot.cr = cr;
  newBallSpot.upper = upper;
  if (upper)
    localBallSpots.ballSpotsUpper.push_back(newBallSpot);
  else
    localBallSpots.ballSpots.push_back(newBallSpot);
}

void CLIPPreprocessor::addLinePoint(const Vector2f& linePoint, const float& lineSize, bool isVertical, const bool& upper)
{
  CLIPPointsPercept::Point point;
  point.inImage = linePoint;
  point.onField = Vector2f::Zero();
  point.lineSizeInImage = lineSize;
  //Geometry::calculatePointOnField(linePoint.x,linePoint.y,theCameraMatrix,theimage,point.onField);
  point.scanLineNoH = scanLineHNo;
  point.scanLineNoV = scanLineVNo;
  point.isVertical = isVertical;
  if (upper)
    localCLIPPointsPercept.pointsUpper.push_back(point);
  else
    localCLIPPointsPercept.points.push_back(point);
}

void CLIPPreprocessor::findFieldBorders()
{
  fieldBorderRight.base = Vector2f::Zero();
  fieldBorderRight.direction = Vector2f::Zero();
  fieldBorderLeft.base = Vector2f::Zero();
  fieldBorderLeft.direction = Vector2f::Zero();
  fieldBorderFront.base = Vector2f::Zero();
  fieldBorderFront.direction = Vector2f::Zero();

  // build upper convex hull of field end points
  fieldHull.clear();
  if ((int)fieldEndPoints.size() >= fieldBorderMinPoints)
  {
    fieldHull.push_back(fieldEndPoints[0].imageCoordinates);
    fieldHull.push_back(fieldEndPoints[1].imageCoordinates);
  }
  else
    return;
  int size = 0;
  for (int i = 2; i < (int)fieldEndPoints.size(); i++)
  {
    size = static_cast<int>(fieldHull.size());
    if ((fieldHull[size - 1] - fieldHull[size - 2]).angle() > (fieldEndPoints[i].imageCoordinates - fieldHull[size - 1]).angle())
    {
      fieldHull.pop_back();
      size = static_cast<int>(fieldHull.size());
      if (size > 1 && (fieldHull[size - 1] - fieldHull[size - 2]).angle() > (fieldEndPoints[i].imageCoordinates - fieldHull[size - 1]).angle())
        fieldHull.pop_back();
    }
    fieldHull.push_back(fieldEndPoints[i].imageCoordinates);
  }

  // try to create line from field end points
  int indexBase = 0;
  int indexDirection = 0;
  int inliers = 0;
  //int outliers = 0;
  bool foundLine = true;
  while ((int)fieldEndPoints.size() >= fieldBorderMinPoints && foundLine)
  {
    int fieldEndPointNo = (int)fieldEndPoints.size();
    foundLine = false;
    for (int round = 0; round < 20; round++)
    {
      inliers = 0;
      //outliers = 0;
      indexBase = random(fieldEndPointNo);
      int tries = 0;
      while (fieldEndPoints[indexBase].imageCoordinates.y() == 0 && tries < fieldEndPointNo / 2)
      {
        indexBase = random(fieldEndPointNo);
        tries++;
      }
      if (tries > 19)
        break;
      indexDirection = random(fieldEndPointNo);
      while (indexBase == indexDirection)
        indexDirection = random(fieldEndPointNo);
      if (tries > 24)
        break;
      Geometry::Line testLine;
      testLine.base = fieldEndPoints[indexBase].imageCoordinates;
      testLine.direction = fieldEndPoints[indexDirection].imageCoordinates - fieldEndPoints[indexBase].imageCoordinates;
      for (int i = 0; i < fieldEndPointNo; i++)
      {
        int distInImage = (int)Geometry::getDistanceToLine(testLine, fieldEndPoints[i].imageCoordinates);
        if (distInImage < fieldBorderMaxDistance)
        {
          inliers++;
          fieldEndPoints[i].inlier = true;
        }
        else
        {
          //outliers++;
          fieldEndPoints[i].inlier = false;
        }
      }
      if (inliers >= fieldBorderMinPoints)
      {
        foundLine = true;
        // side determined by direction angle and base
        if (std::abs(testLine.direction.angle()) < pi_4)
        {
          fieldBorderFront.base = testLine.base;
          fieldBorderFront.direction = testLine.direction;
        }
        else if (testLine.base.x() < imageWidth / 2)
        {
          fieldBorderLeft.base = testLine.base;
          fieldBorderLeft.direction = testLine.direction;
        }
        else
        {
          fieldBorderRight.base = testLine.base;
          fieldBorderRight.direction = testLine.direction;
        }

        // remove inliers from field end points and try to find next line
        std::vector<FieldEndPoint>::iterator j = fieldEndPoints.begin();

        while (j != fieldEndPoints.end())
        {
          if (j->inlier)
            j = fieldEndPoints.erase(j);
          else
            j++;
        }
        break;
      }
    }
  }
  if (fieldBorderFront.base.x() > 1 || fieldBorderRight.base.x() > 1 || fieldBorderLeft.base.x() > 1)
  {
    // draw the field border
    DEBUG_RESPONSE("debug drawing:module:CLIPPreprocessor:fieldBorders")
    {
      if (fieldBorderFront.base.x() > 1)
      {
        Vector2f fieldBorderFrontLeft = Vector2f::Zero();
        Vector2f fieldBorderFrontRight = Vector2f::Zero();
        if ((Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f(4.f, 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontLeft)
                || Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f((float)(imageWidth - 4), 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontRight))
            && fieldBorderFrontLeft.y() >= 4 && fieldBorderFrontRight.y() <= imageHeight - 4)
        {
          LINE("module:CLIPPreprocessor:fieldBorders", fieldBorderFrontLeft.x(), fieldBorderFrontLeft.y(), fieldBorderFrontRight.x(), fieldBorderFrontRight.y(), 3, Drawings::solidPen, ColorRGBA::yellow);
        }
      }
      if (fieldBorderLeft.base.x() > 1)
      {
        Vector2f fieldBorderFrontLeft = Vector2f::Zero();
        Vector2f fieldBorderFrontRight = Vector2f::Zero();
        if ((Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f(4.f, 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontLeft)
                || Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f((float)(imageWidth - 4), 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontRight))
            && fieldBorderFrontLeft.y() >= 4 && fieldBorderFrontRight.y() <= imageHeight - 4)
        {
          LINE("module:CLIPPreprocessor:fieldBorders", fieldBorderFrontLeft.x(), fieldBorderFrontLeft.y(), fieldBorderFrontRight.x(), fieldBorderFrontRight.y(), 3, Drawings::solidPen, ColorRGBA::yellow);
        }
      }
      if (fieldBorderRight.base.x() > 1)
      {
        Vector2f fieldBorderFrontLeft = Vector2f::Zero();
        Vector2f fieldBorderFrontRight = Vector2f::Zero();
        if ((Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f(4.f, 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontLeft)
                || Geometry::getIntersectionOfLines(fieldBorderFront, Geometry::Line(Vector2f((float)(imageWidth - 4), 4.f), Vector2f(0.f, 1.f)), fieldBorderFrontRight))
            && fieldBorderFrontLeft.y() >= 4 && fieldBorderFrontRight.y() <= imageHeight - 4)
        {
          LINE("module:CLIPPreprocessor:fieldBorders", fieldBorderFrontLeft.x(), fieldBorderFrontLeft.y(), fieldBorderFrontRight.x(), fieldBorderFrontRight.y(), 3, Drawings::solidPen, ColorRGBA::yellow);
        }
      }
    }
  }
}

void CLIPPreprocessor::drawScanLineSegments(const bool& upper)
{
  std::vector<ScanLine>& scanLinesHorizontal = upper ? scanLinesHorizontalUpper : scanLinesHorizontalLower;
  std::vector<ScanLine>& scanLinesVertical = upper ? scanLinesVerticalUpper : scanLinesVerticalLower;

  // vertical scan lines
  for (std::vector<ScanLine>::const_iterator i = scanLinesVertical.begin(); i != scanLinesVertical.end(); ++i)
  {
    for (std::vector<ScanLineSegment>::const_iterator seg = i->scanLineSegments.begin(); seg != i->scanLineSegments.end(); ++seg)
    {
      drawSegment(seg, upper);
    }
  }
  // horizontal scan lines
  for (std::vector<ScanLine>::const_iterator i = scanLinesHorizontal.begin(); i != scanLinesHorizontal.end(); ++i)
  {
    for (std::vector<ScanLineSegment>::const_iterator seg = i->scanLineSegments.begin(); seg != i->scanLineSegments.end(); ++seg)
    {
      drawSegment(seg, upper);
    }
  }
  //field End Points and connections, if close enough (only used for drawings)
  for (int i = 0; i < (int)fieldEndPoints.size() - 1; i++)
  {
    if ((fieldEndPoints[i].imageCoordinates - fieldEndPoints[i + 1].imageCoordinates).norm() < 40)
    {
      if (upper)
        LINE("module:CLIPPreprocessor:scanLineSegments:Upper",
            fieldEndPoints[i].imageCoordinates.x(),
            fieldEndPoints[i].imageCoordinates.y(),
            fieldEndPoints[i + 1].imageCoordinates.x(),
            fieldEndPoints[i + 1].imageCoordinates.y(),
            1,
            Drawings::solidPen,
            ColorRGBA(138, 043, 226));
      else
        LINE("module:CLIPPreprocessor:scanLineSegments:Lower",
            fieldEndPoints[i].imageCoordinates.x(),
            fieldEndPoints[i].imageCoordinates.y(),
            fieldEndPoints[i + 1].imageCoordinates.x(),
            fieldEndPoints[i + 1].imageCoordinates.y(),
            1,
            Drawings::solidPen,
            ColorRGBA(138, 043, 226));
    }
    else
    {
      if (upper)
      {
        DOT("module:CLIPPreprocessor:scanLineSegments:Upper", fieldEndPoints[i].imageCoordinates.x(), fieldEndPoints[i].imageCoordinates.y(), ColorRGBA(138, 043, 226), ColorRGBA(138, 043, 226));
        DOT("module:CLIPPreprocessor:scanLineSegments:Upper", fieldEndPoints[i + 1].imageCoordinates.x(), fieldEndPoints[i + 1].imageCoordinates.y(), ColorRGBA(138, 043, 226), ColorRGBA(138, 043, 226));
      }
      else
      {
        DOT("module:CLIPPreprocessor:scanLineSegments:Lower", fieldEndPoints[i].imageCoordinates.x(), fieldEndPoints[i].imageCoordinates.y(), ColorRGBA(138, 043, 226), ColorRGBA(138, 043, 226));
        DOT("module:CLIPPreprocessor:scanLineSegments:Lower", fieldEndPoints[i + 1].imageCoordinates.x(), fieldEndPoints[i + 1].imageCoordinates.y(), ColorRGBA(138, 043, 226), ColorRGBA(138, 043, 226));
      }
    }
  }
}

void CLIPPreprocessor::drawSegment(std::vector<ScanLineSegment>::const_iterator seg, const bool& upper)
{
  // mark new segments (could be same color/type!)
  if (upper)
  {
    if (seg->startPointInImage.x() == seg->endPointInImage.x())
    {
      LINE("module:CLIPPreprocessor:scanLineSegments:Upper", (seg->endPointInImage.x() - 2), seg->endPointInImage.y(), (seg->endPointInImage.x() + 2), seg->endPointInImage.y(), 1, Drawings::solidPen, ColorRGBA::black);
      LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Upper", (seg->endPointInImage.x() - 2), seg->endPointInImage.y(), (seg->endPointInImage.x() + 2), seg->endPointInImage.y(), 1, Drawings::solidPen, ColorRGBA::black);
    }
    else
    {
      LINE("module:CLIPPreprocessor:scanLineSegments:Upper", seg->endPointInImage.x(), (seg->endPointInImage.y() - 2), seg->endPointInImage.x(), (seg->endPointInImage.y() + 2), 1, Drawings::solidPen, ColorRGBA::black);
      LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Upper", seg->endPointInImage.x(), (seg->endPointInImage.y() - 2), seg->endPointInImage.x(), (seg->endPointInImage.y() + 2), 1, Drawings::solidPen, ColorRGBA::black);
    }
  }
  else
  {
    if (seg->startPointInImage.x() == seg->endPointInImage.x())
    {
      LINE("module:CLIPPreprocessor:scanLineSegments:Lower", (seg->endPointInImage.x() - 2), seg->endPointInImage.y(), (seg->endPointInImage.x() + 2), seg->endPointInImage.y(), 1, Drawings::solidPen, ColorRGBA::black);
      LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Lower", (seg->endPointInImage.x() - 2), seg->endPointInImage.y(), (seg->endPointInImage.x() + 2), seg->endPointInImage.y(), 1, Drawings::solidPen, ColorRGBA::black);
    }
    else
    {
      LINE("module:CLIPPreprocessor:scanLineSegments:Lower", seg->endPointInImage.x(), (seg->endPointInImage.y() - 2), seg->endPointInImage.x(), (seg->endPointInImage.y() + 2), 1, Drawings::solidPen, ColorRGBA::black);
      LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Lower", seg->endPointInImage.x(), (seg->endPointInImage.y() - 2), seg->endPointInImage.x(), (seg->endPointInImage.y() + 2), 1, Drawings::solidPen, ColorRGBA::black);
    }
  }
  // draw segments in different colors
  ColorRGBA color = ColorRGBA::black;
  switch (seg->segmentType)
  {
  case fieldSegment:
    color = ColorRGBA::green;
    break;
  case ballSegment:
    color = ColorRGBA::yellow;
    break;
  case lineSegment:
    // black
    break;
  case unknownSegment:
    color = ColorRGBA::gray;
    break;
  case obstacleSegment:
    color = ColorRGBA::red;
    break;
  default:
    break;
  }
  if (upper)
  {
    LINE("module:CLIPPreprocessor:scanLineSegments:Upper", seg->startPointInImage.x(), seg->startPointInImage.y(), seg->endPointInImage.x(), seg->endPointInImage.y(), 1, Drawings::solidPen, color);
    LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Upper", seg->startPointInImage.x(), seg->startPointInImage.y(), seg->endPointInImage.x(), seg->endPointInImage.y(), 1, Drawings::solidPen, color);
  }
  else
  {
    LINE("module:CLIPPreprocessor:scanLineSegments:Lower", seg->startPointInImage.x(), seg->startPointInImage.y(), seg->endPointInImage.x(), seg->endPointInImage.y(), 1, Drawings::solidPen, color);
    LINE("module:CLIPPreprocessor:scanLineSegmentsRaw:Lower", seg->startPointInImage.x(), seg->startPointInImage.y(), seg->endPointInImage.x(), seg->endPointInImage.y(), 1, Drawings::solidPen, color);
  }
}

void CLIPPreprocessor::drawFieldHull(const bool& upper)
{
  for (int i = 0; i < (int)fieldHull.size() - 1; i++)
  {
    if (upper)
      LINE("module:CLIPPreprocessor:fieldHull:Upper", fieldHull[i].x(), fieldHull[i].y(), fieldHull[i + 1].x(), fieldHull[i + 1].y(), 5, Drawings::solidPen, ColorRGBA::yellow);
    else
      LINE("module:CLIPPreprocessor:fieldHull:Lower", fieldHull[i].x(), fieldHull[i].y(), fieldHull[i + 1].x(), fieldHull[i + 1].y(), 5, Drawings::solidPen, ColorRGBA::yellow);
  }
}

MAKE_MODULE(CLIPPreprocessor, perception)
