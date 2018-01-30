#include "CLIPLineFinder.h"
#include "Tools/Math/Transformation.h"

// for drawings
#ifdef USE_FULL_RESOLUTION
  const int scaleFactor = 2;
#else
//const int scaleFactor = 1;
#endif

CLIPLineFinder::CLIPLineFinder()
{
  lastExecutionTimeStamp = 0;
  lastExecutionTimeStampUpper = 0;
  linePoints.reserve(500);
  lineSegments.reserve(100);
  wasReset = false;
  drawUpper = true;
}

void CLIPLineFinder::reset()
{
  localPenaltyCrossPercept.reset();
  foundLines.clear();
  foundLinesUpper.clear();
  localCenterCirclePercept.centerCircleWasSeen = false;
  centerCircles.clear();
  wasReset = true;
}

bool validityHigher(const CLIPFieldLinesPercept::FieldLine &first, const CLIPFieldLinesPercept::FieldLine &second)
{
  return first.validity > second.validity;
}

void CLIPLineFinder::execute(const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : (Image&)theImage;

  unsigned timeStamp = upper ? lastExecutionTimeStampUpper : lastExecutionTimeStamp;

  MODIFY("module:CLIPLineFinder:drawUpper",drawUpper);

  if (timeStamp != image.timeStamp)
  {
    if (upper)
    {
      lastExecutionTimeStampUpper = image.timeStamp;
    }
    else
    {
      lastExecutionTimeStamp = image.timeStamp;
    }
    if (!wasReset)
      reset();
    //else
    //  wasReset = false;

    lineSegments.clear();
    linePoints.clear();
     
#ifdef USE_FULL_RESOLUTION
  imageWidth = image.resolutionWidth*2;
  imageHeight = image.resolutionHeight*2;
#else
  imageWidth = image.width;
  imageHeight = image.height;
#endif

    parameterScale = (float)(imageWidth / 320);
	  int size = upper ? static_cast<int>(theCLIPPointsPercept.pointsUpper.size()) 
      : static_cast<int>(theCLIPPointsPercept.points.size());
    int pointNo = 0;

    while (pointNo < size)
    {
      LinePoint newLP;
	    newLP.point = upper ? &theCLIPPointsPercept.pointsUpper.at(pointNo) : &theCLIPPointsPercept.points.at(pointNo);
      newLP.onLine = false;
      newLP.predecessor = 0;
      newLP.successor = 0;
      linePoints.push_back(newLP);
      pointNo++;
    }

    connectPoints();

    createSegments(upper);

    connectSegments(upper);

    removeCenterCircleTangents(upper);

    enhanceFieldLines(upper);

    // remove the field lines that are still too short
    // also check for gradient on short lines (might lie on circle)
    int lineNo = 0;
    std::vector<CLIPFieldLinesPercept::FieldLine> &lineVector = upper ? (foundLinesUpper) : (foundLines);
    std::vector<CLIPFieldLinesPercept::FieldLine>::iterator line = lineVector.begin();
    while (line != lineVector.end())
    {
      if (localPenaltyCrossPercept.penaltyCrossWasSeen 
        && getPoint2LineDistance(line->startOnField, line->endOnField, localPenaltyCrossPercept.pointOnField.cast<float>()) < 500)
      {
        localPenaltyCrossPercept.penaltyCrossWasSeen = false;
      }
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesEnhanced:Image")
      {
        if (upper == drawUpper)
          LINE("module:CLIPLineFinder:LinesEnhanced:Image", 
            line->startInImage.x(), line->startInImage.y(),
            line->endInImage.x(), line->endInImage.y(), 3, Drawings::solidPen, ColorRGBA(255, 255, 0));
      }
      // TODO: check if minDefiniteLineLengthByDistance always makes sense
      float lineLength = (line->endOnField - line->startOnField).norm();
      float lineDistance = ((line->endOnField + line->startOnField) / 2).norm();
      float minDefiniteLineLengthByDistance = std::min(minDefiniteFieldLineLength, std::max(minFieldLineLength, lineDistance));
      if (lineLength < minDefiniteLineLengthByDistance
        && ((line->lineWidthEnd - line->lineWidthStart) > (line->lineWidthEnd + line->lineWidthStart)/3.0
        || lineLength < minFieldLineLength || !verifyLine(*line,upper)))
      {
        line = lineVector.erase(line);
      }
      else
        line++;
    }

    // remove float lines
    Vector2f lineDir(0,0);
    Vector2f lineDirOther(0,0);
    float minDist = (float)(theFieldDimensions.xPosOpponentGroundline * 4);
    bool outerErase = false;
    lineNo = 0;
    line = lineVector.begin();
    std::vector<CLIPFieldLinesPercept::FieldLine>::iterator otherLine = lineVector.begin();
    while (line != lineVector.end())
    {
      outerErase = false;
      lineDir = (line->endOnField-line->startOnField);
      otherLine = line;
      otherLine++;
      while (otherLine != lineVector.end())
      {
        minDist = getPoint2LineDistance(
          otherLine->startOnField,
          otherLine->endOnField,
          line->startOnField);
        minDist = std::min<float>(getPoint2LineDistance(
          otherLine->startOnField,
          otherLine->endOnField,
          line->endOnField),minDist);
        lineDirOther = (otherLine->endOnField-otherLine->startOnField);
        if (minDist < maxDistSumMergeLinesField && pi_2 - std::abs(std::abs(Angle::normalize(lineDirOther.angle() - lineDir.angle())) - pi_2) < 0.8f)
        {
          // <<<<<<<<<<<<<<<<< MERGE LINES HERE ??? >>>>>>>>>>>>>>>>>>
          if (lineDirOther.norm() < lineDir.norm())
          {
            otherLine = lineVector.erase(otherLine);
          }
          else
          {
            outerErase = true;
            break;
          }
        }
        else
          otherLine++;
      }
      if (outerErase)
        line = lineVector.erase(line);
      else
      {
        float maxLength = static_cast<float>(std::sqrt(imageWidth*imageWidth + imageHeight*imageHeight) / maxValidityDenominator);
        line->validity = std::min<float>(1.f, (line->endInImage - line->startInImage).cast<float>().norm() / maxLength);
        line++;
      }
    }

    // TODO?
    correctCenterCircle();
  }
}

void CLIPLineFinder::connectPoints()
{
  int pointNo = 0,pointNoOther = 1;
  int size = static_cast<int>(linePoints.size());
  LinePoint* closestPoint = 0;
  int closestDistance = 0, dist = 0;
  float distOther = 0.f;
  float closestDistanceOther = 0.f;
  float maxLineSizeDiff = (float)(imageHeight / 120);
  while (pointNo < size - 1)
  {
    pointNoOther = pointNo+1;
    closestPoint = 0;
    closestDistance = imageWidth;
    closestDistanceOther = (float)imageWidth;
    while (pointNoOther < size)
    {
      const CLIPPointsPercept::Point *p = linePoints[pointNo].point;
      const CLIPPointsPercept::Point *pOther = linePoints[pointNoOther].point;
      if (p->isVertical == pOther->isVertical)
      {
        int scanLineDist = p->isVertical ? pOther->scanLineNoY - p->scanLineNoY : pOther->scanLineNoX - p->scanLineNoX;
        if (p->isVertical && scanLineDist > maxNoDistLinesImage)
          break;
        if ((!p->isVertical) && scanLineDist > maxNoDistLinesImage)
          break;
        if (((p->isVertical && p->scanLineNoY != pOther->scanLineNoY && scanLineDist <= maxNoDistLinesImage && std::abs(p->inImage.y() - pOther->inImage.y()) < 3*std::abs(p->inImage.x() - pOther->inImage.x()))
            || ((!p->isVertical) && p->scanLineNoX != pOther->scanLineNoX && scanLineDist <= maxNoDistLinesImage && std::abs(p->inImage.x() - pOther->inImage.x()) < 3*std::abs(p->inImage.y() - pOther->inImage.y())))
            && std::abs(p->lineSizeInImage - pOther->lineSizeInImage) < std::max<float>((p->lineSizeInImage+pOther->lineSizeInImage)/12.f,maxLineSizeDiff))
        {
          dist = scanLineDist;
          distOther = p->isVertical ? std::abs(p->inImage.y() - pOther->inImage.y()) : std::abs(p->inImage.x() - pOther->inImage.x());
          /*linePoints[pointNoOther].predecessor ? imageWidth : 
            (p->isVertical ? abs(pOther->scanLineNoY-linePoints[pointNoOther].predecessor->point->scanLineNoY) :
              abs(pOther->scanLineNoX-linePoints[pointNoOther].predecessor->point->scanLineNoX));*/
          if (dist < closestDistance)
          {
            closestPoint = &linePoints[pointNoOther];
            closestDistance = dist;
            closestDistanceOther = distOther;
          }
          else if (dist == closestDistance && distOther < closestDistanceOther)
          {
            closestPoint = &linePoints[pointNoOther];
            closestDistanceOther = distOther;
          }
        }
      }
      
      pointNoOther++;
    }
    if (closestPoint)
    {
      linePoints[pointNo].successor = closestPoint;
      closestPoint->predecessor = &linePoints[pointNo];
    }
    pointNo++;
  }
}

void CLIPLineFinder::createSegments(const bool &upper)
{
  LinePoint* prevPoint = 0;
  LinePoint* nextPoint = 0;
  int size = (int)linePoints.size();
  const int baseLineError = imageHeight/120;
  std::vector<Vector2f > testPoints;
  float lastAngle = 0.f, angle = 0.f;
  float angleSum = 0.f, lastAngleSum;
  float lastMaxError = 0.f, lastAvgError = 0.f;
  float distSum = 0.f;
  float lastDistSum = 0.f;

  for (int pointNo = 0; pointNo < size; pointNo++)
  {
    if (!linePoints[pointNo].onLine)
    {
      testPoints.clear();
      distSum = 0;
      angleSum = 0;
      lastDistSum = 0;
      LineSegment testSeg;
      testSeg.startPoint = &linePoints[pointNo];
      testSeg.endPoint = &linePoints[pointNo];
      testPoints.push_back(linePoints[pointNo].point->inImage);
      testSeg.avgError = 0.f;
      testSeg.maxError = 0.f;
      testSeg.onCircle = false;
      testSeg.avgWidth = linePoints[pointNo].point->lineSizeInImage;
      prevPoint = &linePoints[pointNo];
      nextPoint = prevPoint->successor;
      if (nextPoint)
      {
        lastAngle = (nextPoint->point->inImage-prevPoint->point->inImage).angle();
      }
      else
      {
        continue;
      }
      testSeg.pointNo = 1;
      prevPoint->onLine = true;
      Geometry::Line testLine;
      testLine.base = prevPoint->point->inImage;
      testLine.direction.x() = testLine.direction.y() = 0;

      while (nextPoint && !nextPoint->onLine)
      {
        lastMaxError = testSeg.maxError;
        lastAvgError = testSeg.avgError;
        lastAngleSum = angleSum;
        angle = (nextPoint->point->inImage-prevPoint->point->inImage).angle();
        angleSum += angle-lastAngle;
        nextPoint->onLine = true;
        testPoints.push_back(nextPoint->point->inImage);
        testSeg.maxError = std::max<float>(testSeg.maxError, std::abs(Geometry::getDistanceToLine(testLine,nextPoint->point->inImage)));
        testLine.direction = Vector2f(testPoints.back()-testLine.base);
        if (testPoints.size() >= static_cast<unsigned>(minPointsForLine))
        {
          testLine = Geometry::calculateLineByLinearRegression(testPoints,testSeg.avgError,testSeg.maxError);
        }
        
        if (testPoints.size() > 2 &&
          (((testSeg.maxError > (maxTotalLineErrorImage*parameterScale)
          || (lastMaxError > baseLineError && testSeg.maxError > 4*lastMaxError)) && std::abs(angleSum) < maxAngleSumLineImage)
          || std::abs(angle-lastAngle) > maxSegmentAngleImage))
        {
          testPoints.pop_back();
          
          LineSegment newSeg;
          newSeg.avgWidth = testSeg.avgWidth;
          testLine = Geometry::calculateLineByLinearRegression(testPoints,newSeg.avgError,newSeg.maxError);
          newSeg.endPoint = prevPoint;
          newSeg.pointNo = testSeg.pointNo;
          newSeg.startPoint = testSeg.startPoint;
          newSeg.angleSum = std::abs(lastAngleSum);
          newSeg.onCircle = false;
          lineSegments.push_back(newSeg);
          DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Image")
          {
            if (upper == drawUpper)
            {
              float y = (newSeg.endPoint->point->inImage.y() > 30) ? newSeg.endPoint->point->inImage.y() - 1 : newSeg.endPoint->point->inImage.y() + 7;
              int nr = (int)lineSegments.size();
              DRAWTEXT("module:CLIPLineFinder:Connections:Image",
                newSeg.endPoint->point->inImage.x(), y, 10, ColorRGBA::red, nr);
            }
          }
          nextPoint->onLine = false;
          testSeg.pointNo = 0;
          break;
        }
        testSeg.pointNo++;
        testSeg.avgWidth = (testSeg.avgWidth + prevPoint->point->lineSizeInImage)/2.f;
        testSeg.angleSum = angleSum;
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Image")
        {
          if (upper == drawUpper)
            LINE("module:CLIPLineFinder:Connections:Image",
              prevPoint->point->inImage.x(), prevPoint->point->inImage.y(),
              nextPoint->point->inImage.x(), nextPoint->point->inImage.y(),
              1, Drawings::solidPen, ColorRGBA::red);
        }
        prevPoint = nextPoint;
        nextPoint = prevPoint->successor;
        lastAngle = angle;
      }

      testSeg.endPoint = prevPoint;
      if (testSeg.pointNo > 1)
      {
        lineSegments.push_back(testSeg);
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Image")
        {
          if (upper == drawUpper)
          {
            float y = (testSeg.endPoint->point->inImage.y() > 30) ? testSeg.endPoint->point->inImage.y() - 1 : testSeg.endPoint->point->inImage.y() + 7;
            DRAWTEXT("module:CLIPLineFinder:Connections:Image", testSeg.endPoint->point->inImage.x(), y, 10, ColorRGBA::red, (unsigned)lineSegments.size());
          }
        }
      }
    }
  }
}

void CLIPLineFinder::connectSegments(const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  int segNo = 0, segNoOther = 0;
  bool verified = false;
  Vector2f baseCenter(0,0);
  // first connect small segments of similar curvature that may lie on a circle
  while (segNo < (int)lineSegments.size()-1)
  {
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Image")
    {
      if (upper == drawUpper)
        LINE("module:CLIPLineFinder:LineSegments:Image",
          lineSegments[segNo].startPoint->point->inImage.x(),
          lineSegments[segNo].startPoint->point->inImage.y(),
          lineSegments[segNo].endPoint->point->inImage.x(),
          lineSegments[segNo].endPoint->point->inImage.y(),
          2, Drawings::solidPen, ColorRGBA::blue);
    }
    if ((lineSegments[segNo].pointNo > minPointsForLine || lineSegments[segNo].pointNo > minPointsForCenterCircle/2)
      && std::abs(lineSegments[segNo].angleSum) > minAngleSumCircleImage)
    {
      CenterCircle newCC;
      newCC.circle.radius = 0.0;
      newCC.pointsOnCircle.clear();
      newCC.upper = upper;
      if (!addSegmentPointsToCircle(lineSegments[segNo],newCC,upper))
      {
        segNo++;
        continue;
      }
      if (!Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle,newCC.circle))
      {
        segNo++;
        continue;
      }
      if (std::abs(newCC.circle.radius-theFieldDimensions.centerCircleRadius) > maxCenterCircleRadiusDiffField)
      {
        segNo++;
        continue;
      }
      baseCenter = newCC.circle.center;

      segNoOther = 0;
      verified = false;
      while (segNoOther < (int)lineSegments.size())
      {
        if (segNoOther == segNo || lineSegments[segNoOther].pointNo < 2 
          || (lineSegments[segNoOther].pointNo >= minPointsForLine && lineSegments[segNoOther].angleSum <= maxAngleSumLineImage))
        {
          segNoOther++;
          continue;
        }
        if (addSegmentPointsToCircle(lineSegments[segNoOther],newCC,upper))
        {
          if (!Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle,newCC.circle))
            removeSegmentPointsFromCircle(lineSegments[segNoOther],newCC);
          else if (std::abs(newCC.circle.radius-theFieldDimensions.centerCircleRadius) > maxCenterCircleRadiusDiffField 
            || std::abs((newCC.circle.center-baseCenter).norm()) > maxCenterCircleRadiusDiffField 
            || !verifyCircle(newCC,false))
            removeSegmentPointsFromCircle(lineSegments[segNoOther],newCC);
          else
          {
            lineSegments[segNoOther].onCircle = true;
            verified = true;
          }
          // <<<<<<<<<<<< add a check for white between line Segments ?? >>>>>>>>>>>

        
          // draw test circle and test points !!!
        }
        else
          removeSegmentPointsFromCircle(lineSegments[segNoOther],newCC);

        segNoOther++;
      }
      
      // create center circle from start/end and one middle point
      if ((int)newCC.pointsOnCircle.size() > minPointsForCenterCircle)
      {
        if (Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle,newCC.circle)
          && std::abs(newCC.circle.radius-theFieldDimensions.centerCircleRadius) < maxCenterCircleRadiusDiffField
          && (verified || verifyCircle(newCC,true))
          //&& verifyCenterCircle(newCC, upper)
          )
        {
          centerCircles.push_back(newCC);
          lineSegments[segNo].onCircle = true;
        }
      }
    }
    segNo++;
  }

  segNo = 0;
  
  // use best circle
  int maxID = 0, maxPointNo = 0;
  for (int id = 0; id < (int)centerCircles.size(); id++)
  {
    if ((int)centerCircles[id].pointsOnCircle.size() > maxPointNo)
    {
      maxID = id;
      maxPointNo = (int)centerCircles[id].pointsOnCircle.size();
    }
  }

  // use the best circle as center circle percept if existent
  if (maxPointNo >= minPointsForCenterCircle
    &&centerCircles[maxID].upper == upper) // otherwise the circle has already been added
  {
    Vector2f inImage;
    Vector2f onField(centerCircles[maxID].circle.center);
    if (!Transformation::robotToImage(onField, cameraMatrix, cameraInfo, inImage))
      inImage = Vector2f::Zero();
    // TODO : verify final circle here by scanning for green at some points
    localCenterCirclePercept.centerCircleWasSeen = true;
    localCenterCirclePercept.centerCircle.locationOnField = onField.cast<int>();
    localCenterCirclePercept.centerCircle.orientationKnown = false;
    localCenterCirclePercept.centerCircle.locationInImage = inImage.cast<int>();
    localCenterCirclePercept.fromUpper = centerCircles[maxID].upper;
    // no /RES_DIV_FACTOR here!
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCircle:Image")
    {
      if (upper == drawUpper)
      {
        const float rotateAngle = pi2 / 64;
        Vector2f centerToCirclePoint((int)centerCircles[maxID].circle.radius, 0);
        Vector2f pField;
        Vector2f pImage;
        for (int i = 0; i < 64; i++)
        {
          centerToCirclePoint.rotate(rotateAngle);
          pField = onField + centerToCirclePoint;
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage)
           && !image.isOutOfImage(pImage.x(), pImage.y(), 3))
          {
            CIRCLE("module:CLIPLineFinder:CenterCircle:Image",
              pImage.x(),
              pImage.y(),
              4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
          }
        }
      }
    }
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCirclePoints:Image")
    {
      if (upper == drawUpper)
      {
        for (int i = 0; i < (int)centerCircles[maxID].pointsOnCircle.size(); i++)
        {
          Vector2f pImage;
          Vector2f pField(centerCircles[maxID].pointsOnCircle[i]);
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage))
            CIRCLE("module:CLIPLineFinder:CenterCirclePoints:Image",
              pImage.x(),
              pImage.y(),
              4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
        }
      }
    }
  }

  
  // then create field lines and penalty cross from remaining line segments
  segNo = 0;
  bool foundConnection = false;
  while (segNo < (int)lineSegments.size())
  {
    if (lineSegments[segNo].pointNo <= 3 && createPenaltyCross(lineSegments[segNo],upper))
    {
      Vector2f pImage((lineSegments[segNo].endPoint->point->inImage+lineSegments[segNo].endPoint->point->inImage)*0.5f);
      Vector2f pField;
      if (Transformation::imageToRobot(pImage, 
        cameraMatrix, cameraInfo, pField)
        && (!localPenaltyCrossPercept.penaltyCrossWasSeen 
        || pField.norm() < localPenaltyCrossPercept.pointOnField.cast<float>().norm()))
      {
        // TODO: check if image point has to be scaled with /RES_DIV_FACTOR
        localPenaltyCrossPercept.pointInImage = pImage.cast<int>();
        localPenaltyCrossPercept.pointOnField = pField.cast<int>();
        localPenaltyCrossPercept.penaltyCrossWasSeen = true;
        localPenaltyCrossPercept.fromUpper = upper;
      }
    }
    if (lineSegments[segNo].onCircle || lineSegments[segNo].pointNo == 1
      || std::abs(lineSegments[segNo].avgError) > (maxAvgLineErrorImage*parameterScale) 
      || std::abs(lineSegments[segNo].maxError) > (maxTotalLineErrorImage*parameterScale))
    {
      lineSegments.erase(lineSegments.begin()+segNo);
      continue;
    }
    foundConnection = false;
    std::vector<CLIPFieldLinesPercept::FieldLine> &lineVector = upper ? (foundLinesUpper) : (foundLines);
    
    for (int lineNo = 0; lineNo < (int)lineVector.size(); lineNo++)
    {
      if (connectSegmentToFieldLine(
        lineSegments[segNo].startPoint->point->inImage,
        lineSegments[segNo].endPoint->point->inImage,
        lineVector[lineNo],
        upper))
      {
        lineSegments.erase(lineSegments.begin()+segNo);
        foundConnection = true;
        break;
      }
    }
    if (!foundConnection)
    {
      if (lineSegments[segNo].pointNo < minPointsForLine)
      {
        segNoOther = segNo+1;
        while (segNoOther < (int)lineSegments.size())
        {
          if (lineSegments[segNoOther].pointNo > 1 && connect2Segments(lineSegments[segNo],lineSegments[segNoOther],upper))
          {
            lineSegments[segNo].pointNo += lineSegments[segNoOther].pointNo;
            lineSegments[segNo].endPoint->successor = lineSegments[segNoOther].startPoint;
            lineSegments[segNo].endPoint = lineSegments[segNoOther].endPoint;
            lineSegments.erase(lineSegments.begin()+segNoOther);
            segNoOther--;
          }
          segNoOther++;
        }
      }
      if (lineSegments[segNo].pointNo > minPointsForLine/2 && createLineFromSingleSegment(lineSegments[segNo],upper))
      {
        lineSegments.erase(lineSegments.begin()+segNo);
        segNo--;
      }
    }
    else
      segNo--;
    segNo++;
  }
}

void CLIPLineFinder::correctCenterCircle()
{
  
}

void CLIPLineFinder::enhanceFieldLines(const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  float lineWidth,maxLineWidth,minLineWidth,lastLineWidth;
  float lineLength;
  bool lineStartWide = false;

  std::vector< Vector2f > checkPoints;
  checkPoints.clear();

  std::vector<CLIPFieldLinesPercept::FieldLine> &lineVector = upper ? (foundLinesUpper) : (foundLines);
  std::vector<CLIPFieldLinesPercept::FieldLine>::iterator line = lineVector.begin();

  while (line != lineVector.end())
  {
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesRaw:Image")
    {
      if (upper == drawUpper)
        LINE("module:CLIPLineFinder:LinesRaw:Image",
          line->startInImage.x(), line->startInImage.y(),
          line->endInImage.x(), line->endInImage.y(),
          3, Drawings::solidPen, ColorRGBA(153, 0, 255));
    }
    Vector2f lineStart = line->startInImage.cast<float>();
    Vector2f lineEnd = line->endInImage.cast<float>();
    Vector2f lineDir = lineEnd-lineStart;
    Vector2f lineDirField = (line->endOnField-line->startOnField);
    lineLength = lineDirField.norm();
    if (lineLength < 100)
    {
      line = lineVector.erase(line);
      continue;
    }
    if (line->lineWidthStart > line->lineWidthEnd)
      lineStartWide = true;
    else
      lineStartWide = false;
    minLineWidth = line->lineWidthEnd;
    maxLineWidth = line->lineWidthEnd;
    lineWidth = line->lineWidthEnd;
    lineDir.normalize(std::max(std::min(lineLength/5,(float)(imageHeight/12)),1.f));
    
    checkPoints.clear();
    Vector2f scanPoint(lineEnd.x()+lineDir.x(),lineEnd.y()+lineDir.y());
    Vector2f scanDir = lineDir;
    scanDir.rotateLeft();
    Vector2f scanDirNormal(scanDir);
    scanDirNormal.normalize();
    scanDir.normalize(std::max<float>(1.f,line->lineWidthEnd/10.f));
    scanPoint -= scanDirNormal*(std::max<float>(line->lineWidthEnd,2.5f));
    Vector2f lastCenter(scanPoint);
    lastLineWidth = lineWidth;

    // from end of line to infinity - aaaaand beyond!
    while (getLineCenterAndWidth(scanPoint,scanDir,lastLineWidth,lastCenter,upper))
    {
      // if lineWidth seems wrong, stop enhancing
      if (lastLineWidth > lineWidth + std::max<float>(1.f,lineWidth/10)
        || lastLineWidth < lineWidth - std::max<float>(1.f,lineWidth/10))
      {
        lastLineWidth = lineWidth;
        break;
      }
      else
      {
        if (lineStartWide)
          lineWidth = std::min<float>(lineWidth,lastLineWidth);
        else
          lineWidth = std::max<float>(lineWidth, lastLineWidth);
      }
      
      lastLineWidth = lineWidth;
      Vector2f newCenter(lastCenter);
      checkPoints.push_back(newCenter);
      scanPoint = lastCenter + lineDir;
    }

    float distSum = 0.0;
    for (int i = 0; i < (int)checkPoints.size(); i++)
    {
      Geometry::Line l;
      l.base = lineEnd;
      l.direction = lineDir;
      distSum += std::abs(Geometry::getDistanceToLine(l, checkPoints[i]));
    }
    if (distSum > std::max(3.f,(line->lineWidthStart+line->lineWidthEnd)/5))
    {
      if (lineLength < 1500.f)
        line = lineVector.erase(line);
      else
        line++;
      continue;
    }
    else if (!checkPoints.empty() && 
      Transformation::imageToRobot(checkPoints.back(),
        cameraMatrix,cameraInfo,line->endOnField))
    {
      line->endInImage = checkPoints.back().cast<int>();
      lineEnd = checkPoints.back();
      line->lineWidthEnd = lastLineWidth;
    }

    lineDir = lineStart-lineEnd;
      
    lineWidth = line->lineWidthStart;
    lineDir.normalize(std::max(std::min(lineLength/5,(float)(imageHeight/12)),1.f));
    
    checkPoints.clear();
    scanPoint = lineStart+lineDir;
    scanDir = lineDir;
    scanDir.rotateLeft();
    scanDirNormal = scanDir;
    scanDirNormal.normalize();
    scanDir.normalize(std::max<float>(1.f,line->lineWidthStart/10.f));
    scanPoint -= scanDirNormal*line->lineWidthStart;
    lastCenter = scanPoint;
    lastLineWidth = lineWidth;

    // from start of line to infinity - aaaaand beyond!
    while (getLineCenterAndWidth(scanPoint,scanDir,lastLineWidth,lastCenter,upper))
    {
      // if lineWidth seems wrong, stop enhancing
      if (lastLineWidth > lineWidth + std::max<float>(1.f,lineWidth/10)
        || lastLineWidth < lineWidth - std::max<float>(1.f, lineWidth / 10))
      {
        lastLineWidth = lineWidth;
        break;
      }
      else
      {
        if (lineStartWide)
          lineWidth = std::min(lineWidth,lastLineWidth);
        else
          lineWidth = std::max(lineWidth,lastLineWidth);
      }
      lastLineWidth = lineWidth;
      Vector2f newCenter(lastCenter);
      checkPoints.push_back(newCenter);
      scanPoint = lastCenter + lineDir;
    }

    distSum = 0.f;
    for (int i = 0; i < (int)checkPoints.size(); i++)
    {
      Geometry::Line l;
      l.base = lineEnd;
      l.direction = lineDir;
      distSum += std::abs(Geometry::getDistanceToLine(l,checkPoints[i]));
    }
    if (distSum > std::max<float>(3.f,(line->lineWidthStart+line->lineWidthEnd)/5))
    {
      if (lineLength < 1500.f)
        line = lineVector.erase(line);
      else
        line++;
    }
    else
    {
      if (!checkPoints.empty() && 
        Transformation::imageToRobot(checkPoints.back(),
          cameraMatrix,cameraInfo,line->startOnField))
      {
        line->startInImage = checkPoints.back().cast<int>();
        line->lineWidthStart = lastLineWidth;
      }
      line++;
    }

  }

  // now try to enhance the small lines (kind of hack..does only work on current field..to test)
  // if a small line is betweens to long, perpendicular ones - enhance it up to them
  // maybe : enhance lines perpendicular to others in general, if the others are long, safe ones?
  // ..TODO?

}

bool CLIPLineFinder::getLineCenterAndWidth(const Vector2f scanPoint, const Vector2f scanDir, float &width, Vector2f &center, const bool &upper)
{
  // TODO: use full image here
  const Image &image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  Vector2f checkPoint(scanPoint.x(),scanPoint.y());
  if (image.isOutOfImage(checkPoint.x(),checkPoint.y(),3))
    return false;

  center = scanPoint;
  float localWidth = 0;
  float lineWidth = 0;
#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
  Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
  int y = p.y;
  int lastY = y;
  int minY = y;
  int maxY = y;
  bool foundLineStart = false;
  float stepSize = scanDir.norm();
  float maxWidth = std::max<float>(width*2,6.f);

  while (!image.isOutOfImage(checkPoint.x(),checkPoint.y(),3) && localWidth < maxWidth)
  {
#ifdef USE_FULL_RESOLUTION
  image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
  p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    y = p.y;
    if (!foundLineStart && y - minY > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
      foundLineStart = true;
    if (!foundLineStart && minY > y)
      minY = y;
    if (foundLineStart && maxY < y)
      maxY = y;
    if (foundLineStart)
      lineWidth += stepSize;
    if (foundLineStart && maxY - y > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
    {
      width = lineWidth;
      center = checkPoint - scanDir*(lineWidth/(2*stepSize));
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesEnhanced:Image")
      {
        if (upper == drawUpper)
          LINE("module:CLIPLineFinder:LinesEnhanced:Image",
            scanPoint.x(), scanPoint.y(), checkPoint.x(),
            checkPoint.y(), 1, Drawings::solidPen, ColorRGBA(153, 0, 255));
      }
      return true;
    }
    localWidth += stepSize;
    lastY = y;
    checkPoint += scanDir;
  }
  return false;
}

bool CLIPLineFinder::addSegmentPointsToCircle(LineSegment &seg, CenterCircle &circle, const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  LinePoint *nextPoint = seg.startPoint;
  LinePoint *prevPoint = seg.startPoint;
  bool onField = true;
  float angleSum = 0;
  float angle = 0, lastAngle = 0;
  float distSum = 0.f;
  int count = 0;
  int sizeOffset = (int)circle.pointsOnCircle.size();
  float angleOfCircleCoveredBySegment = 0.f;
  float minAngle = 2*pi2;
  float maxAngle = -pi;

  while (nextPoint && nextPoint != seg.endPoint->successor)
  {
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Image")
    {
      if (upper == drawUpper)
        DOT("module:CLIPLineFinder:LineSegments:Image",
          nextPoint->point->inImage.x(), nextPoint->point->inImage.y(),
          ColorRGBA::green, ColorRGBA::green);
    }
    Vector2f pointOnField;
    if (Transformation::imageToRobot(nextPoint->point->inImage, cameraMatrix, cameraInfo, pointOnField))
    {
      circle.pointsOnCircle.push_back(pointOnField);
      if (onField && count > 1)
      {
        angle = (circle.pointsOnCircle[sizeOffset+count]-circle.pointsOnCircle[sizeOffset+count-1]).angle();
        angleSum += (angle-lastAngle);
      }
      else if (onField && count == 1)
      {
        lastAngle = (circle.pointsOnCircle[sizeOffset+1]-circle.pointsOnCircle[sizeOffset]).angle();
        angle = lastAngle;
      }
      count++;
      if (sizeOffset > 0)
        distSum += getDistancePointToCircle(pointOnField,circle.circle);
    }
    else
    {
      circle.pointsOnCircle.push_back(pointOnField);
      onField = false;
    }
    prevPoint = nextPoint;
    nextPoint = nextPoint->successor;
    lastAngle = angle;
  }
  Geometry::Circle testCircle;
  if (Geometry::computeCircleOnFieldLevenbergMarquardt(circle.pointsOnCircle, testCircle))
  {
    float leftAngle = (circle.pointsOnCircle[sizeOffset] - testCircle.center).angle();
    float rightAngle = (circle.pointsOnCircle.back() - testCircle.center).angle();
    minAngle = std::min(leftAngle, rightAngle);
    maxAngle = std::max(leftAngle, rightAngle);
    angleOfCircleCoveredBySegment = maxAngle - minAngle;
    if (angleOfCircleCoveredBySegment > pi)
      angleOfCircleCoveredBySegment = std::abs(angleOfCircleCoveredBySegment - pi2);
    if (std::abs(angleOfCircleCoveredBySegment - std::abs(angleSum)) > std::max<float>(std::abs(angleOfCircleCoveredBySegment) / 5, 0.4f))
      return false;
    return onField && distSum < 150 && (count < minPointsForLine || std::abs(angleSum) > maxAngleSumLineField);
  }
  return false;
}

void CLIPLineFinder::removeSegmentPointsFromCircle(LineSegment &seg, CenterCircle &circle)
{
  int size = (int)circle.pointsOnCircle.size();
  ASSERT(size >= seg.pointNo);
  circle.pointsOnCircle.erase(circle.pointsOnCircle.begin()+size-seg.pointNo,circle.pointsOnCircle.end());
}

bool CLIPLineFinder::verifyCircle(const CenterCircle &circle, const bool checkSeenAngle)
{
  float distSum = 0.f;
  float dist = 0.f;
  float maxDist = 0.f;
  //float minAngle = pi_D;
  //float maxAngle = -pi_D;
  int pointNo = (int)circle.pointsOnCircle.size();
  if (pointNo < 1)
    return false;
  for (int i = 0; i < pointNo; i++)
  {
    dist = getDistancePointToCircle(circle.pointsOnCircle[i],circle.circle);
    //float angle = (circle.pointsOnCircle[i]-circle.circle.center).angle();
    maxDist = std::max(maxDist,dist);
    //minAngle = std::min(minAngle, angle);
    //maxAngle = std::max(maxAngle, angle);
    distSum += dist;
  }
  return distSum / pointNo < maxAvgPointDistToCircleField &&
    maxDist < maxDistPointsToCircleField &&
    pointNo > minPointsForCenterCircle / 2;// && (!checkSeenAngle || (maxAngle-minAngle < 0.7));
}

bool CLIPLineFinder::verifyCenterCircle(const CenterCircle &circle, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  
  const float pi_16 = pi_4/4;
  const Vector2f pFieldBase(circle.circle.center);
  
  Vector2f pImageLeft, pImageMiddle, pImageRight;
  
  Vector2f pFieldCenterToCircle(circle.circle.radius,0);
  Vector2f pFieldRight = pFieldBase+pFieldCenterToCircle;
  pFieldCenterToCircle.rotate(pi_16);
  Vector2f pFieldMiddle = pFieldBase+pFieldCenterToCircle;
  pFieldCenterToCircle.rotate(pi_16);
  Vector2f pFieldLeft = pFieldBase+pFieldCenterToCircle;
  if (!Transformation::robotToImage(pFieldLeft,
    cameraMatrix,
    cameraInfo,
    pImageLeft))
    pImageLeft.x() = -1;
  if (!Transformation::robotToImage(pFieldMiddle,
    cameraMatrix,
    cameraInfo,
    pImageMiddle))
    pImageMiddle.x() = -1;
  if (!Transformation::robotToImage(pFieldRight,
    cameraMatrix,
    cameraInfo,
    pImageRight))
    pImageRight.x() = -1;

  unsigned int i = 0;
  unsigned int fails = 0;
  unsigned int oks = 0;
  int tries = 16;
    
  while ((image.isOutOfImage(pImageLeft.x(),pImageLeft.y(),3)
    || image.isOutOfImage(pImageMiddle.x(),pImageMiddle.y(),3)
    || image.isOutOfImage(pImageRight.x(),pImageRight.y(),3))
    && i < 15)
  {
    Vector2f pFieldRight = pFieldLeft;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldMiddle = pFieldBase+pFieldCenterToCircle;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldLeft = pFieldBase+pFieldCenterToCircle;
    if (!Transformation::robotToImage(pFieldLeft,
      cameraMatrix,
      cameraInfo,
      pImageLeft))
      pImageLeft.x() = -1;
    if (!Transformation::robotToImage(pFieldMiddle,
      cameraMatrix,
      cameraInfo,
      pImageMiddle))
      pImageMiddle.x() = -1;
    if (!Transformation::robotToImage(pFieldRight,
      cameraMatrix,
      cameraInfo,
      pImageRight))
      pImageRight.x() = -1;
    i++;
  }
  for (; i < 16; i++)
  {
    Vector2f pFieldRight = pFieldLeft;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldMiddle = pFieldBase + pFieldCenterToCircle;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldLeft = pFieldBase + pFieldCenterToCircle;
    if (!Transformation::robotToImage(pFieldLeft,
      cameraMatrix,
      cameraInfo,
      pImageLeft) ||
      !Transformation::robotToImage(pFieldMiddle,
        cameraMatrix,
        cameraInfo,
        pImageMiddle) ||
      !Transformation::robotToImage(pFieldRight,
        cameraMatrix,
        cameraInfo,
        pImageRight) ||
      image.isOutOfImage(pImageLeft.x(),pImageLeft.y(),3) ||
      image.isOutOfImage(pImageMiddle.x(),pImageMiddle.y(),3) ||
      image.isOutOfImage(pImageRight.x(),pImageRight.y(),3))
      continue;
    else
    {
      // TODO: drawing!
      // TODO: sanity check
      // TODO: check condition - should be between -normal and + normal?
      float lineSize = Geometry::calculateLineSizePrecise(pImageMiddle.cast<int>(),cameraMatrix,cameraInfo, theFieldDimensions.fieldLinesWidth);
      Vector2f checkPoint((pImageLeft.x() + pImageRight.x()) / 2, (pImageLeft.y() + pImageRight.y()) / 2);
      Vector2f normal((pImageLeft.x() - pImageRight.x()), (pImageLeft.y() - pImageRight.y()));
      normal.rotateRight().normalize(lineSize * 3);
      if (!checkForWhiteBetween(checkPoint, checkPoint + normal, lineSize, upper))
        fails++;
      else
        oks++;
    }
    tries--;
  }
  return (fails < 2 && oks > 2 && tries < 10);
}

bool CLIPLineFinder::connect2Segments(LineSegment &segA, LineSegment &segB, const bool &upper)
{
  // TODO: check for sensible line width diffs
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  if (segA.startPoint->point->isVertical != segB.startPoint->point->isVertical)
    return false;
  float segAAngle = (segA.endPoint->point->inImage-segA.startPoint->point->inImage).angle();
  float segBAngle = (segB.endPoint->point->inImage-segB.startPoint->point->inImage).angle();
  if (std::abs(Angle::normalize(segAAngle-segBAngle)) > maxSegmentAngleDiffImage)
    return false;

  bool segAFirst = (segA.startPoint->point->inImage-segB.endPoint->point->inImage).norm() > (segA.startPoint->point->inImage-segB.startPoint->point->inImage).norm();
  Vector2f segDir = segAFirst ? segB.endPoint->point->inImage-segA.startPoint->point->inImage :
    segA.endPoint->point->inImage-segB.startPoint->point->inImage;
  if (std::abs(Angle::normalize(segAAngle-segDir.angle())) > maxSegmentAngleDiffImage
    || std::abs(Angle::normalize(segBAngle-segDir.angle())) > maxSegmentAngleDiffImage)
    return false;
  std::vector< Vector2f > pointsOnLine;
  LinePoint *point = segAFirst ? segA.startPoint : segB.startPoint;
  while (point != (segAFirst ? segA.endPoint->successor : segB.endPoint->successor))
  {
    pointsOnLine.push_back(point->point->inImage);
    point = point->successor;
  }
  point = segAFirst ? segB.startPoint : segA.startPoint;;
  while (point != (segAFirst ? segB.endPoint->successor : segA.endPoint->successor))
  {
    pointsOnLine.push_back(point->point->inImage);
    point = point->successor;
  }

  // calculate line with linear regression
  float biggestError = 0.0, avgError = 0.0;
  Geometry::Line regLine = Geometry::calculateLineByLinearRegression(pointsOnLine,avgError,biggestError);
  if (avgError > (maxAvgLineErrorImage*parameterScale) || biggestError > (maxTotalLineErrorImage*parameterScale))
    return false;

  // get start/end point
  Vector2f startImage,endImage;
  if (!Geometry::getIntersectionOfLines(
    regLine,
    Geometry::Line(segAFirst ? segA.startPoint->point->inImage : segB.startPoint->point->inImage, Vector2f(0.f, 1.f)),
    startImage)
    || !Geometry::getIntersectionOfLines(
      regLine,
      Geometry::Line(segAFirst ? segB.endPoint->point->inImage : segA.endPoint->point->inImage, Vector2f(0.f, 1.f)), endImage))
    return false;
  
  // build Field Line
  CLIPFieldLinesPercept::FieldLine newFL;
  newFL.startInImage = startImage.cast<int>();
  newFL.endInImage = endImage.cast<int>();
  newFL.fromUpper = upper;
  newFL.validity = 0.f;
  newFL.isPlausible = false;
  
  // project lineSize (not perpendicular to line direction, rather either vertical or horizontal)
  newFL.lineWidthStart = segAFirst ? projectLineSize(regLine.direction,segA.startPoint->point->lineSizeInImage,segA.startPoint->point->isVertical)
    : projectLineSize(regLine.direction,segB.startPoint->point->lineSizeInImage,segB.startPoint->point->isVertical);
  newFL.lineWidthEnd = segAFirst ? projectLineSize(regLine.direction,segB.endPoint->point->lineSizeInImage,segB.endPoint->point->isVertical)
    : projectLineSize(regLine.direction,segA.endPoint->point->lineSizeInImage,segA.endPoint->point->isVertical);

  // check if line widths match
  if (segAFirst)
  {
    const float &sizeA = segA.endPoint->point->lineSizeInImage;
    const float &sizeB = segB.startPoint->point->lineSizeInImage;
    if (std::abs(sizeA - sizeB) > std::max(3.f, std::min(sizeA, sizeB) / 4))
      return false;
  }
  else
  {
    const float &sizeA = segA.startPoint->point->lineSizeInImage;
    const float &sizeB = segB.endPoint->point->lineSizeInImage;
    if (std::abs(sizeA - sizeB) > std::max(3.f, std::min(sizeA, sizeB) / 4))
      return false;
  }
  
  // check distance of segment start/end points to line (SHOULD BE REDUNDANT AFTER ANGLE CHECK AT OF METHOD)
  float distSum = getPoint2LineDistance(segA.startPoint->point->inImage,segA.endPoint->point->inImage,
    segB.endPoint->point->inImage);
  distSum += getPoint2LineDistance(segA.startPoint->point->inImage,segA.endPoint->point->inImage,
    segB.startPoint->point->inImage);
  distSum += getPoint2LineDistance(segB.startPoint->point->inImage,segB.endPoint->point->inImage,
    segA.startPoint->point->inImage);
  distSum += getPoint2LineDistance(segB.startPoint->point->inImage,segB.endPoint->point->inImage,
    segA.startPoint->point->inImage);
  if (distSum < (maxDistSumPointsToLineImage*parameterScale)
    && Transformation::imageToRobot(newFL.endInImage,
    cameraMatrix,cameraInfo,newFL.endOnField)
    && Transformation::imageToRobot(newFL.startInImage,
    cameraMatrix,cameraInfo,newFL.startOnField))
  {
    Vector2f onFieldEndFirst,onFieldStartSecond;
    if (segAFirst && !checkForGreenBetween(segA.endPoint->point->inImage,segB.startPoint->point->inImage,upper))
    {
      if (!Transformation::imageToRobot(segA.endPoint->point->inImage,
        cameraMatrix, cameraInfo, onFieldEndFirst) ||
        !Transformation::imageToRobot(segB.startPoint->point->inImage,
          cameraMatrix, cameraInfo, onFieldStartSecond))
        OUTPUT_WARNING("CLIPLineFinder : imageToRobot failed in connect2Segments");
    }
    else if (!segAFirst && !checkForGreenBetween(segB.endPoint->point->inImage,segA.startPoint->point->inImage,upper))
    {
      if (!Transformation::imageToRobot(segB.endPoint->point->inImage,
        cameraMatrix,cameraInfo,onFieldEndFirst) ||
      !Transformation::imageToRobot(segA.startPoint->point->inImage,
        cameraMatrix,cameraInfo,onFieldStartSecond))
        OUTPUT_WARNING("CLIPLineFinder : imageToRobot failed in connect2Segments");
    }
    else
      return false;

    // connect segments, if distance on field is close enough and build line if line points is enough
    if ((onFieldEndFirst-onFieldStartSecond).norm() < maxSegConnectDistField)
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Image")
      {
        if (upper == drawUpper)
          LINE("module:CLIPLineFinder:LineSegments:Image",
            segA.endPoint->point->inImage.x(),
            segA.endPoint->point->inImage.y(),
            segB.startPoint->point->inImage.x(),
            segB.startPoint->point->inImage.y(),
            1, Drawings::dottedPen, ColorRGBA::magenta);
      }
      if (segA.pointNo+segB.pointNo >= minPointsForLine)
      {
        if (upper)
          foundLinesUpper.push_back(newFL);
        else
          foundLines.push_back(newFL);
      }
      return true;
    }
  }
  return false;
}

bool CLIPLineFinder::createLineFromSingleSegment(const LineSegment &seg, const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  std::vector< Vector2f > pointsOnLine;

  LinePoint *point = seg.startPoint;
  float angleSum = 0;
  float angle = 0, lastAngle = 0;
  int count = 0;
  Vector2f lastPField(0,0);

  while (point && point != seg.endPoint->successor)
  {
    Vector2f pField(0,0);
    if (seg.pointNo < 2*minPointsForLine)
    {
      if (!Transformation::imageToRobot(point->point->inImage, cameraMatrix, cameraInfo, pField))
        return false;
      if (count > 1)
      {
        angle = (pField-lastPField).angle();
        angleSum += (lastAngle-angle);
      }
      else if (count == 1)
      {
        lastAngle = angle = (pField-lastPField).angle();
      }
    }
    pointsOnLine.push_back(point->point->inImage);
    point = point->successor;
    count++;
    lastAngle = angle;
    lastPField = pField;
  }
  if (std::abs(angleSum) > maxAngleSumLineField)
    return false;
  Vector2f dir(seg.endPoint->point->inImage-seg.startPoint->point->inImage);
  return createFieldLine(
    pointsOnLine,
    projectLineSize(dir,seg.startPoint->point->lineSizeInImage,seg.startPoint->point->isVertical),
    projectLineSize(dir,seg.endPoint->point->lineSizeInImage,seg.endPoint->point->isVertical),
    upper);
}

bool CLIPLineFinder::createPenaltyCross(const LineSegment &seg, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  const Vector2f basePoint(
    (seg.endPoint->point->inImage.x()+seg.startPoint->point->inImage.x())/2.f,
    (seg.endPoint->point->inImage.y()+seg.startPoint->point->inImage.y())/2.f);
  Vector2f checkPoint = basePoint;
  float lineSize = Geometry::calculateLineSizePrecise(
    Vector2i((int)checkPoint.x(), (int)checkPoint.y()), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth);
  if (lineSize < imageHeight/80)
    return false;
  Vector2f scanDir(1.f,0.f);
  float count = 0.f;
  float whiteCount[8];
  float avgWhiteCount = 0.f;
  float minWC = (theFieldDimensions.xPosOpponentGroundline * 4);
  float maxWC = 0.f;
  for (int i = 0; i < 8; i++)
  {
    count = 0.f;
    whiteCount[i] = 0.f;
    checkPoint = basePoint;
    while (count < lineSize*3)
    {
      count++;
      checkPoint += scanDir;
      if (image.isOutOfImage((checkPoint+scanDir).x(),(checkPoint+scanDir).y(),3))
        return false;
#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
  Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
      whiteCount[i] += (p.y > fieldColor.fieldColorArray[0].fieldColorOptY+fieldColor.fieldColorArray[0].lineToFieldColorYThreshold
        && !fieldColor.isPixelFieldColor(p.y,p.cb,p.cr));
    }
    scanDir.rotate(pi_4);
    avgWhiteCount+=whiteCount[i];
    minWC = std::min(minWC,whiteCount[i]);
    maxWC = std::max(maxWC,whiteCount[i]);
  }
  avgWhiteCount /= 4;
  bool result = avgWhiteCount > lineSize/2.f && maxWC < lineSize*1.5f;
  if (result)
  {
    // verify by checking for white around potential cross
    scanDir.x() = lineSize*3;
    scanDir.y() = 0;
    for (int i = 0; i < 32; i++)
    {
      checkPoint = basePoint + scanDir;
      if (image.isOutOfImage(checkPoint.x(),checkPoint.y(),3))
      {
        result = false;
        break;
      }
#ifdef USE_FULL_RESOLUTION
      Image::YUVPixel p;
      image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
      Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
      if (p.y > fieldColor.fieldColorArray[0].fieldColorOptY+fieldColor.fieldColorArray[0].lineToFieldColorYThreshold
        && !fieldColor.isPixelFieldColor(p.y,p.cb,p.cr))
      {
        result = false;
        break;
      }
      scanDir.rotate(pi/16);
    }
  }
  CROSS("module:CLIPLineFinder:penaltyCross",basePoint.x(),basePoint.y(),lineSize,5,Drawings::solidPen,result ? ColorRGBA::blue : ColorRGBA::red);
  return result;
}

void CLIPLineFinder::update(CLIPCenterCirclePercept &theCenterCirclePercept)
{
  wasReset = false;
  execute(false);
  execute(true);
  theCenterCirclePercept = localCenterCirclePercept;
}

void CLIPLineFinder::update(CLIPFieldLinesPercept &theCLIPFieldLinesPercept)
{
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCircle:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCirclePoints:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LineSegments:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesRaw:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesEnhanced:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:Connections:Image","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:penaltyCross","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:checkForLineBetween","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:checkForWhiteBetween","drawingOnImage");
  wasReset = false;
  execute(false);
  execute(true);
  theCLIPFieldLinesPercept.lines.clear();
  theCLIPFieldLinesPercept.lines.insert(theCLIPFieldLinesPercept.lines.end(),foundLines.begin(),foundLines.end());
  theCLIPFieldLinesPercept.lines.insert(theCLIPFieldLinesPercept.lines.end(), foundLinesUpper.begin(), foundLinesUpper.end());

  std::sort(theCLIPFieldLinesPercept.lines.begin(), theCLIPFieldLinesPercept.lines.end(), validityHigher);

  checkForPlausability(theCLIPFieldLinesPercept.lines);
}

void CLIPLineFinder::update(PenaltyCrossPercept &thePenaltyCrossPercept)
{
  wasReset = false;
  execute(false);
  execute(true);
  thePenaltyCrossPercept = localPenaltyCrossPercept;
}

bool CLIPLineFinder::connectSegmentToFieldLine(
  const Vector2f &imgStart,
  const Vector2f &imgEnd,
  CLIPFieldLinesPercept::FieldLine &line,
  const bool &upper)
{
  // TODO: check for sensible line width diffs
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  Vector2f startPointField,endPointField;
  if (!Transformation::imageToRobot(imgStart, cameraMatrix, cameraInfo, startPointField)
    || !Transformation::imageToRobot(imgEnd, cameraMatrix, cameraInfo, endPointField))
    return false;
  
  bool segFirst = (startPointField-line.endOnField).norm() > (startPointField-line.startOnField).norm();
  
  // check for distance between segment and line, also check for distance of points to new line
  float distSum = 0.f;
  if ((segFirst && (line.startOnField-endPointField).norm() > maxSegConnectDistField)
    || ((line.endOnField-startPointField).norm() > maxSegConnectDistField))
    return false;

  distSum += getPoint2LineDistance(line.startInImage,line.endInImage,imgStart);
  distSum += getPoint2LineDistance(line.startInImage,line.endInImage,imgEnd);
  if (distSum > (maxDistSumPointsToLineImage*parameterScale))
    return false;

  float segAngle = (imgEnd-imgStart).angle();
  float lineAngle = (line.endInImage-line.startInImage).cast<float>().angle();
  if (std::abs(Angle::normalize(segAngle-lineAngle)) > maxSegmentAngleDiffImage)
    return false;

  Vector2f segDir = segFirst ? Vector2f(line.endInImage.x()-imgStart.x(),line.endInImage.y()-imgStart.y()) :
    Vector2f(imgEnd.x()-line.startInImage.x(),imgEnd.y()-line.startInImage.y());
  
  if (std::abs(Angle::normalize(segAngle-segDir.angle())) > maxSegmentAngleDiffImage
    || std::abs(Angle::normalize(lineAngle-segDir.angle())) > maxSegmentAngleDiffImage)
    return false;
  
  
  // all seems ok.. connect it
  if (segFirst && !checkForGreenBetween(imgEnd, Vector2f(line.startInImage.cast<float>()), upper))
  {
    line.startInImage = imgStart.cast<int>();
    line.startOnField = startPointField;
  }
  else if (!segFirst && !checkForGreenBetween(Vector2f(line.endInImage.cast<float>()), imgStart, upper))
  {
    line.endInImage = imgEnd.cast<int>();
    line.endOnField = endPointField;
  }
  else
    return false;

  return true;
}

bool CLIPLineFinder::createFieldLine(
  const std::vector< Vector2f > &pointsOnLine,
  const float &lineWidthStart,
  const float &lineWidthEnd,
  const bool &upper)
{
  const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  float biggestError = 0.f, avgError = 0.f;
  Geometry::Line regLine = Geometry::calculateLineByLinearRegression(pointsOnLine,avgError,biggestError);
  if (avgError > (maxAvgLineErrorImage*parameterScale) || biggestError > (maxTotalLineErrorImage*parameterScale))
    return false;

  CLIPFieldLinesPercept::FieldLine newFL;
  
  Vector2f startImage,endImage;
  if (pointsOnLine.front().x() < pointsOnLine.back().x())
  {
    startImage = pointsOnLine.front();
    endImage = pointsOnLine.back();
  }
  else
  {
    startImage = pointsOnLine.back();
    endImage = pointsOnLine.front();
  }
  
  newFL.startInImage = startImage.cast<int>();
  newFL.endInImage = endImage.cast<int>();
  newFL.lineWidthStart = lineWidthStart;
  newFL.lineWidthEnd = lineWidthEnd;
  newFL.validity = 0.f;
  newFL.isPlausible = false;
  newFL.fromUpper = upper;
  if (!Transformation::imageToRobot(startImage,cameraMatrix,cameraInfo,newFL.startOnField)
    || !Transformation::imageToRobot(endImage,cameraMatrix,cameraInfo,newFL.endOnField))
    return false;
  float distSum = 0.f;
  Vector2f newFLDir = (newFL.endOnField-newFL.startOnField);
  Vector2f lineDir;
  std::vector<CLIPFieldLinesPercept::FieldLine> &lineVector = upper ? (foundLinesUpper) : (foundLines);

  for (int i = 0; i < (int)lineVector.size(); i++)
  {
    distSum = getPoint2LineDistance(
      lineVector[i].startOnField,
      lineVector[i].endOnField,
      newFL.startOnField);
    distSum += getPoint2LineDistance(
      lineVector[i].startOnField,
      lineVector[i].endOnField,
      newFL.endOnField);
    lineDir = (lineVector[i].endOnField-lineVector[i].startOnField);
    if (distSum < maxDistSumMergeLinesField && std::abs(newFLDir.angle() - lineDir.angle()) < 0.8f)
    {
      // <<<<<<<<<<<<<<<<< MERGE LINES HERE ??? >>>>>>>>>>>>>>>>>>
      if (newFLDir.norm() > lineDir.norm())
      {
        lineVector.erase(lineVector.begin()+i);
        lineVector.push_back(newFL);
        return true;
      }
      else
        return false;
    }
  }
  lineVector.push_back(newFL);
  //LINE("module:CLIPLineFinder:LineSegments:Image",newFL.startInImage.x,newFL.startInImage.y,newFL.endInImage.x,newFL.endInImage.y,2,Drawings::ps_solid,ColorClasses::yellow);
  return true;
}

bool CLIPLineFinder::checkForLineBetween(
  const Vector2f &imgStart,
  const Vector2f &imgEnd,
  const float &lineSizeInImage,
  const bool &upper
  )
{
  LINE("module:CLIPLineFinder:checkForLineBetween",
    imgStart.x(),imgStart.y(),
    imgEnd.x(),imgEnd.y(),
    3,Drawings::solidPen,ColorRGBA(138,43,226));
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  // first check for 'white' on field line (2 (even 1?) check(s) should be enough)
  float safetyBuffer = (float)(imageHeight / 40);
  Vector2f lineDir = imgEnd-imgStart;
  Vector2f lineDirNormal = lineDir;
  lineDirNormal.rotateLeft();
  lineDirNormal.normalize(1);
  Vector2f checkPoint = imgStart+lineDir/2-lineDirNormal*(safetyBuffer/2+lineSizeInImage/2);
  int lineStart = 0, pointNo = 0, lineEnd = 0;
  if (image.isOutOfImage(checkPoint.x(),checkPoint.y(),3))
    return false;
#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
  Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
  int lastY = p.y;
  int y = lastY;
  
  while (pointNo <= lineSizeInImage+safetyBuffer && !image.isOutOfImage(checkPoint.x(),checkPoint.y(),3))
  {
#ifdef USE_FULL_RESOLUTION
    image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
    p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    y = p.y;
    if (lineEnd == 0 && y - lastY > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold/2)
      lineStart = pointNo;
    if (lastY - y > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold/2)
      lineEnd = pointNo;
    
    checkPoint += lineDirNormal;
    lastY = y;
    pointNo++;
  }
  if (lineStart == 0 || lineEnd == 0 || lineStart >= lineEnd
    || std::abs(lineSizeInImage - (float)(lineEnd - lineStart)) >= std::max<float>(std::min<float>(4.5f, lineSizeInImage / 6), 1.5f)
    || (lineStart - safetyBuffer) >= safetyBuffer/3)
    return false;
  return true;
}


bool CLIPLineFinder::verifyLine(const CLIPFieldLinesPercept::FieldLine &line, const bool &upper)
{
  return checkForLineBetween(
    Vector2f(line.startInImage.cast<float>()),
    Vector2f(line.endInImage.cast<float>()),
    0.5f*(line.lineWidthStart+line.lineWidthEnd),
    upper);
}

bool CLIPLineFinder::checkForGreenBetween(const Vector2f &startInImage, const Vector2f &endInImage, const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;
  
  int length = static_cast<int>((endInImage-startInImage).norm());
  int sampleDistance = std::max(length/10,3);
  int sampleCount = length/sampleDistance;
  int steps = 0;
  int greenCount = 0;
  if (sampleCount < 5)
    return true;
  Vector2f checkPoint(startInImage);
  Vector2f scanDir(endInImage-startInImage);
  scanDir.normalize((float)sampleDistance);
  while (steps < sampleCount && !image.isOutOfImage(checkPoint.x(),checkPoint.y(),2))
  {
#ifdef USE_FULL_RESOLUTION
    Image::YUVPixel p;
    image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
    Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    greenCount += fieldColor.isPixelFieldColor(p.y,p.cb,p.cr);
    steps++;
    checkPoint += scanDir;
  }
  if (greenCount >= steps/2 || steps < 5)
    return true;
  return false;
}

bool CLIPLineFinder::checkForWhiteBetween(const Vector2f &startInImage, const Vector2f &endInImage, const float &lineSize, const bool &upper)
{
  LINE("module:CLIPLineFinder:checkForWhiteBetween",
    startInImage.x(),startInImage.y(),
    endInImage.x(),endInImage.y(),
    3,Drawings::solidPen,ColorRGBA(138,43,226));
  
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;
  
  int length = (int)(endInImage-startInImage).norm();
  int sampleDistance = std::max(length/20,1);
  int sampleCount = length/sampleDistance;
  int steps = 0;
  int greenCount = 0;
  if (sampleCount < 5)
    return true;
  Vector2f checkPoint(startInImage);
  Vector2f scanDir(endInImage-startInImage);
  scanDir.normalize((float)sampleDistance);
  int greenWhiteY = fieldColor.fieldColorArray[0].fieldColorOptY;
  int whiteCount = 0;
  while (steps < sampleCount && !image.isOutOfImage(checkPoint.x(),checkPoint.y(),2))
  {
#ifdef USE_FULL_RESOLUTION
    Image::YUVPixel p;
    image.getPixel((unsigned int)checkPoint.x,(unsigned int)checkPoint.y,&p);
#else
    Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    if (fieldColor.isPixelFieldColor(p.y,p.cb,p.cr))
    {
      greenCount++;
      greenWhiteY += p.y;
    }
    else if (p.y > (greenWhiteY/(greenCount+1))+fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
      whiteCount++;
    steps++;
    checkPoint += scanDir;
  }
  if (greenCount >= steps/4 && whiteCount >= std::max(lineSize*0.75f,1.f))
    return true;
  return false;
}

void CLIPLineFinder::removeCenterCircleTangents(const bool &upper)
{
  if (!localCenterCirclePercept.centerCircleWasSeen)
    return;

  std::vector<CLIPFieldLinesPercept::FieldLine> &lineVector = upper ? (foundLinesUpper) : (foundLines);
  std::vector<CLIPFieldLinesPercept::FieldLine>::iterator i = lineVector.begin();
  float distSum = 0.f;

  while (i != lineVector.end())
  {
    distSum = 0.f;
    Vector2f endToCircle(localCenterCirclePercept.centerCircle.locationOnField.x()-i->endOnField.x(),
      localCenterCirclePercept.centerCircle.locationOnField.y()-i->endOnField.y());
    distSum += std::abs((endToCircle.norm() - theFieldDimensions.centerCircleRadius));
    Vector2f startToCircle(localCenterCirclePercept.centerCircle.locationOnField.x()-i->startOnField.x(),
      localCenterCirclePercept.centerCircle.locationOnField.y()-i->startOnField.y());
    distSum += std::abs(startToCircle.norm() - theFieldDimensions.centerCircleRadius);
    Vector2f middleToCircle(
      localCenterCirclePercept.centerCircle.locationOnField.x()-(i->endOnField.x()+i->startOnField.x())/2,
      localCenterCirclePercept.centerCircle.locationOnField.y()-(i->endOnField.y()+i->startOnField.y())/2);
    distSum += std::abs(middleToCircle.norm() - theFieldDimensions.centerCircleRadius);
    if (distSum < maxDistSegToCenterCircleField)
      i = lineVector.erase(i);
    else
      i++;
  }
}

MAKE_MODULE(CLIPLineFinder, perception)
