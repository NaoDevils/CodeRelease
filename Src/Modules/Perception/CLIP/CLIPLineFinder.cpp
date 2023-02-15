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

bool validityHigher(const CLIPFieldLinesPercept::FieldLine& first, const CLIPFieldLinesPercept::FieldLine& second)
{
  return first.validity > second.validity;
}

void CLIPLineFinder::execute(const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : (Image&)theImage;

  unsigned timeStamp = upper ? lastExecutionTimeStampUpper : lastExecutionTimeStamp;

  MODIFY("module:CLIPLineFinder:drawUpper", drawUpper);

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

    if (!image.shouldBeProcessed())
      return;

    if (!wasReset)
      reset();
    //else
    //  wasReset = false;

    lineSegments.clear();
    linePoints.clear();

#ifdef USE_FULL_RESOLUTION
    imageWidth = image.resolutionWidth * 2;
    imageHeight = image.resolutionHeight * 2;
#else
    imageWidth = image.width;
    imageHeight = image.height;
#endif

    parameterScale = (float)(imageWidth / 320);
    int size = upper ? static_cast<int>(theCLIPPointsPercept.pointsUpper.size()) : static_cast<int>(theCLIPPointsPercept.points.size());
    int pointNo = 0;

    while (pointNo < size)
    {
      LinePoint newLP;
      newLP.point = upper ? &theCLIPPointsPercept.pointsUpper.at(pointNo) : &theCLIPPointsPercept.points.at(pointNo);
      newLP.onLine = false;
      newLP.predecessor = -1;
      newLP.successor = -1;
      linePoints.push_back(newLP);
      pointNo++;
    }

    checkYoloPenaltyCross(upper);

    connectPoints();

    createSegments(upper);

    connectSegments(upper);

    removeCenterCircleTangents(upper);

    enhanceFieldLines(upper);

    // remove the field lines that are still too short
    // also check for gradient on short lines (might lie on circle)
    std::vector<CLIPFieldLinesPercept::FieldLine>& lineVector = upper ? (foundLinesUpper) : (foundLines);
    std::vector<CLIPFieldLinesPercept::FieldLine>::iterator line = lineVector.begin();
    while (line != lineVector.end())
    {
      if (localPenaltyCrossPercept.penaltyCrossWasSeen && localPenaltyCrossPercept.detectionType == PenaltyCrossPercept::scanlines
          && getPoint2LineDistance(line->startOnField, line->endOnField, localPenaltyCrossPercept.pointOnField.cast<float>()) < 500)
      {
        localPenaltyCrossPercept.penaltyCrossWasSeen = false;
      }
      if (upper == drawUpper)
      {
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesEnhanced:Upper")
        {
          LINE("module:CLIPLineFinder:LinesEnhanced:Upper", line->startInImage.x(), line->startInImage.y(), line->endInImage.x(), line->endInImage.y(), 3, Drawings::solidPen, ColorRGBA(255, 255, 0));
        }
      }
      else
      {
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesEnhanced:Lower")
        {
          LINE("module:CLIPLineFinder:LinesEnhanced:Lower", line->startInImage.x(), line->startInImage.y(), line->endInImage.x(), line->endInImage.y(), 3, Drawings::solidPen, ColorRGBA(255, 255, 0));
        }
      }
      // TODO: check if minDefiniteLineLengthByDistance always makes sense
      float lineLength = (line->endOnField - line->startOnField).norm();
      float lineDistance = ((line->endOnField + line->startOnField) / 2).norm();
      float minDefiniteLineLengthByDistance = std::min(minDefiniteFieldLineLength, std::max(minFieldLineLength, lineDistance));
      if (lineLength < minDefiniteLineLengthByDistance && (line->endInImage - line->startInImage).cast<float>().norm() < imageWidth / 4
          && ((line->lineWidthEnd - line->lineWidthStart) > (line->lineWidthEnd + line->lineWidthStart) / 3.f || lineLength < minFieldLineLength || !verifyLine(*line, upper)))
      {
        line = lineVector.erase(line);
      }
      else
        line++;
    }

    // remove float lines
    Vector2f lineDir(0, 0);
    Vector2f lineDirOther(0, 0);
    float minDist = (float)(theFieldDimensions.xPosOpponentGroundline * 4);
    bool outerErase = false;
    line = lineVector.begin();
    std::vector<CLIPFieldLinesPercept::FieldLine>::iterator otherLine = lineVector.begin();
    while (line != lineVector.end())
    {
      outerErase = false;
      lineDir = (line->endOnField - line->startOnField);
      otherLine = line;
      otherLine++;
      while (otherLine != lineVector.end())
      {
        minDist = getPoint2LineDistance(otherLine->startOnField, otherLine->endOnField, line->startOnField);
        minDist = std::min<float>(getPoint2LineDistance(otherLine->startOnField, otherLine->endOnField, line->endOnField), minDist);
        lineDirOther = (otherLine->endOnField - otherLine->startOnField);
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
        float maxLength = static_cast<float>(std::sqrt(imageWidth * imageWidth + imageHeight * imageHeight) / maxValidityDenominator);
        line->validity = std::min<float>(1.f, (line->endInImage - line->startInImage).cast<float>().norm() / maxLength);
        line++;
      }
    }

    // TODO?
    correctCenterCircle();
  }
}

void CLIPLineFinder::checkYoloPenaltyCross(bool upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  std::vector<PenaltyCross> pcs = upper ? thePenaltyCrossHypothesesYolo.penaltyCrossesUpper : thePenaltyCrossHypothesesYolo.penaltyCrosses;

  if (pcs.size() > 0)
  {
    std::sort(pcs.begin(), pcs.end());
    PenaltyCross pc = pcs[0];
    Vector2f pImage(pc.positionInImage);
    Vector2f pField;
    if (Transformation::imageToRobot(pImage, cameraMatrix, cameraInfo, pField))
    {
      localPenaltyCrossPercept.pointInImage = pImage.cast<int>();
      localPenaltyCrossPercept.pointOnField = pField.cast<int>();
      localPenaltyCrossPercept.penaltyCrossWasSeen = true;
      localPenaltyCrossPercept.fromUpper = upper;
      localPenaltyCrossPercept.detectionType = PenaltyCrossPercept::yolo;
    }
  }
}

void CLIPLineFinder::connectPoints()
{
  int pointNo = 0, pointNoOther = 1;
  int size = static_cast<int>(linePoints.size());
  int closestPoint = -1;
  int closestDistance = 0, dist = 0;
  float distOther = 0.f;
  float closestDistanceOther = 0.f;
  float maxLineSizeDiff = (float)(imageHeight / 120);
  while (pointNo < size - 1)
  {
    pointNoOther = pointNo + 1;
    closestPoint = -1;
    closestDistance = imageWidth;
    closestDistanceOther = (float)imageWidth;
    while (pointNoOther < size)
    {
      const CLIPPointsPercept::Point* p = linePoints[pointNo].point;
      const CLIPPointsPercept::Point* pOther = linePoints[pointNoOther].point;
      int scanLineDist = p->isVertical ? pOther->scanLineNoV - p->scanLineNoV : pOther->scanLineNoH - p->scanLineNoH;
      if (p->isVertical == pOther->isVertical && scanLineDist > 0)
      {
        if (scanLineDist > maxNoDistLinesImage) // test points are too far away, stop search for near points to p
          break;
        // angle to the next point in image should not be too steep, the orthogonal scan lines should get that line
        if (((p->isVertical && std::abs(p->inImage.y() - pOther->inImage.y()) < 3 * std::abs(p->inImage.x() - pOther->inImage.x()))
                || ((!p->isVertical) && std::abs(p->inImage.x() - pOther->inImage.x()) < 3 * std::abs(p->inImage.y() - pOther->inImage.y())))
            && std::abs(p->lineSizeInImage - pOther->lineSizeInImage) < std::max<float>((p->lineSizeInImage + pOther->lineSizeInImage) / 12.f, maxLineSizeDiff))
        {
          dist = scanLineDist;
          distOther = p->isVertical ? std::abs(p->inImage.y() - pOther->inImage.y()) : std::abs(p->inImage.x() - pOther->inImage.x());
          /*linePoints[pointNoOther].predecessor ? imageWidth :
            (p->isVertical ? abs(pOther->scanLineNoY-linePoints[pointNoOther].predecessor->point->scanLineNoY) :
              abs(pOther->scanLineNoX-linePoints[pointNoOther].predecessor->point->scanLineNoX));*/
          if (dist < closestDistance)
          {
            closestPoint = pointNoOther;
            closestDistance = dist;
            closestDistanceOther = distOther;
          }
          else if (dist == closestDistance && distOther < closestDistanceOther)
          {
            closestPoint = pointNoOther;
            closestDistanceOther = distOther;
          }
        }
      }

      pointNoOther++;
    }
    if (closestPoint >= 0 && linePoints[closestPoint].predecessor == -1) //closest point was set to not zero -> connect the 2 points up
    {
      linePoints[pointNo].successor = closestPoint;
      linePoints[closestPoint].predecessor = pointNo;
    }
    pointNo++;
  }
}

void CLIPLineFinder::createSegments(const bool& upper)
{
  int prevPoint = -1;
  int nextPoint = -1;
  int size = (int)linePoints.size();
  std::vector<Vector2f> testPoints;
  float lastAngle = 0.f, angle = 0.f;

  for (int pointNo = 0; pointNo < size; pointNo++)
  {
    if (!linePoints[pointNo].onLine)
    {
      prevPoint = pointNo;
      nextPoint = linePoints[pointNo].successor;
      if (nextPoint >= 0)
      {
        lastAngle = (linePoints[nextPoint].point->inImage - linePoints[prevPoint].point->inImage).angle();
      }
      else
      {
        continue;
      }
      testPoints.clear();
      LineSegment testSeg;
      testSeg.startPoint = pointNo;
      testSeg.endPoint = nextPoint;
      testPoints.push_back(linePoints[pointNo].point->inImage);
      testSeg.avgError = 0.f;
      testSeg.maxError = 0.f;
      testSeg.angleSum = 0.f;
      testSeg.onCircle = false;
      testSeg.avgWidth = linePoints[pointNo].point->lineSizeInImage;

      testSeg.pointNo = 1;
      linePoints[prevPoint].onLine = true;
      bool segmentFinished = false;

      while (nextPoint >= 0 && !linePoints[nextPoint].onLine)
      {
        angle = (linePoints[nextPoint].point->inImage - linePoints[prevPoint].point->inImage).angle();
        linePoints[nextPoint].onLine = true;
        testPoints.push_back(linePoints[nextPoint].point->inImage);

        // check if next point not on this line -> finish and verify segment, then add to lineSegments
        if (std::abs(angle - lastAngle) > maxSegmentAngleImage)
        {
          if (finishLineSegment(testSeg, prevPoint, testPoints, upper))
            lineSegments.push_back(testSeg);
          else
            linePoints[nextPoint].onLine = false;
          segmentFinished = true;
          break;
        }

        // next point is on line, go on
        testSeg.pointNo++;
        testSeg.avgWidth += linePoints[nextPoint].point->lineSizeInImage;
        testSeg.angleSum += angle - lastAngle;
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Upper")
        {
          if (upper == drawUpper)
            LINE("module:CLIPLineFinder:Connections:Upper",
                linePoints[prevPoint].point->inImage.x(),
                linePoints[prevPoint].point->inImage.y(),
                linePoints[nextPoint].point->inImage.x(),
                linePoints[nextPoint].point->inImage.y(),
                1,
                Drawings::solidPen,
                ColorRGBA::red);
        }
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Lower")
        {
          if (upper != drawUpper)
            LINE("module:CLIPLineFinder:Connections:Lower",
                linePoints[prevPoint].point->inImage.x(),
                linePoints[prevPoint].point->inImage.y(),
                linePoints[nextPoint].point->inImage.x(),
                linePoints[nextPoint].point->inImage.y(),
                1,
                Drawings::solidPen,
                ColorRGBA::red);
        }
        prevPoint = nextPoint;
        nextPoint = linePoints[nextPoint].successor;
        lastAngle = angle;
      }
      // if all connections are on one line, we land here
      if (!segmentFinished && finishLineSegment(testSeg, prevPoint, testPoints, upper))
        lineSegments.push_back(testSeg);
    }
  }
}

void CLIPLineFinder::connectSegments(const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  int segNo = 0, segNoOther = 0;
  bool verified = false;
  Vector2f baseCenter(0, 0);
  // first connect small segments of similar curvature that may lie on a circle
  while (segNo < (int)lineSegments.size() - 1)
  {
    if (upper == drawUpper)
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Upper")
      {
        LINE("module:CLIPLineFinder:LineSegments:Upper",
            linePoints[lineSegments[segNo].startPoint].point->inImage.x(),
            linePoints[lineSegments[segNo].startPoint].point->inImage.y(),
            linePoints[lineSegments[segNo].endPoint].point->inImage.x(),
            linePoints[lineSegments[segNo].endPoint].point->inImage.y(),
            2,
            Drawings::solidPen,
            ColorRGBA::blue);
      }
    }
    else
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Lower")
      {
        LINE("module:CLIPLineFinder:LineSegments:Lower",
            linePoints[lineSegments[segNo].startPoint].point->inImage.x(),
            linePoints[lineSegments[segNo].startPoint].point->inImage.y(),
            linePoints[lineSegments[segNo].endPoint].point->inImage.x(),
            linePoints[lineSegments[segNo].endPoint].point->inImage.y(),
            2,
            Drawings::solidPen,
            ColorRGBA::blue);
      }
    }
    if ((lineSegments[segNo].pointNo > minPointsForLine || lineSegments[segNo].pointNo > minPointsForCenterCircle / 2)
        && (std::abs(lineSegments[segNo].angleSum) > minAngleSumCircleImage || lineSegments[segNo].onCircle))
    {
      CenterCircle newCC;
      newCC.circle.radius = 0.0;
      newCC.pointsOnCircle.clear();
      newCC.upper = upper;
      if (!addSegmentPointsToCircle(lineSegments[segNo], newCC, upper))
      {
        segNo++;
        continue;
      }
      if (!Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle, newCC.circle))
      {
        segNo++;
        continue;
      }
      if (std::abs(newCC.circle.radius - theFieldDimensions.centerCircleRadius) > maxCenterCircleRadiusDiffField)
      {
        segNo++;
        continue;
      }
      baseCenter = newCC.circle.center;

      segNoOther = 0;
      verified = false;
      while (segNoOther < (int)lineSegments.size())
      {
        if (segNoOther == segNo || lineSegments[segNoOther].pointNo < 2 || (lineSegments[segNoOther].pointNo >= minPointsForLine && lineSegments[segNoOther].angleSum <= maxAngleSumLineImage))
        {
          segNoOther++;
          continue;
        }
        if (addSegmentPointsToCircle(lineSegments[segNoOther], newCC, upper))
        {
          if (!Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle, newCC.circle))
            removeSegmentPointsFromCircle(lineSegments[segNoOther], newCC);
          else if (std::abs(newCC.circle.radius - theFieldDimensions.centerCircleRadius) > maxCenterCircleRadiusDiffField
              || std::abs((newCC.circle.center - baseCenter).norm()) > maxCenterCircleRadiusDiffField || !verifyCircle(newCC, false))
            removeSegmentPointsFromCircle(lineSegments[segNoOther], newCC);
          else
          {
            lineSegments[segNoOther].onCircle = true;
            verified = true;
          }
          // <<<<<<<<<<<< add a check for white between line Segments ?? >>>>>>>>>>>


          // draw test circle and test points !!!
        }
        else
          removeSegmentPointsFromCircle(lineSegments[segNoOther], newCC);

        segNoOther++;
      }

      // create center circle from start/end and one middle point
      if ((int)newCC.pointsOnCircle.size() > minPointsForCenterCircle)
      {
        if (Geometry::computeCircleOnFieldLevenbergMarquardt(newCC.pointsOnCircle, newCC.circle)
            && std::abs(newCC.circle.radius - theFieldDimensions.centerCircleRadius) < maxCenterCircleRadiusDiffField && (verified || verifyCircle(newCC, true))
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
  if (maxPointNo >= minPointsForCenterCircle && centerCircles[maxID].upper == upper) // otherwise the circle has already been added
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
    if (upper == drawUpper)
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCircle:Upper")
      {
        const float rotateAngle = pi2 / 64;
        Vector2f centerToCirclePoint((int)centerCircles[maxID].circle.radius, 0);
        Vector2f pField;
        Vector2f pImage;
        for (int i = 0; i < 64; i++)
        {
          centerToCirclePoint.rotate(rotateAngle);
          pField = onField + centerToCirclePoint;
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage) && !image.isOutOfImage(pImage.x(), pImage.y(), 3))
          {
            CIRCLE("module:CLIPLineFinder:CenterCircle:Upper", pImage.x(), pImage.y(), 4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
          }
        }
      }
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCirclePoints:Upper")
      {
        for (int i = 0; i < (int)centerCircles[maxID].pointsOnCircle.size(); i++)
        {
          Vector2f pImage;
          Vector2f pField(centerCircles[maxID].pointsOnCircle[i]);
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage))
            CIRCLE("module:CLIPLineFinder:CenterCirclePoints:Upper", pImage.x(), pImage.y(), 4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
        }
      }
    }
    else
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCircle:Lower")
      {
        const float rotateAngle = pi2 / 64;
        Vector2f centerToCirclePoint((int)centerCircles[maxID].circle.radius, 0);
        Vector2f pField;
        Vector2f pImage;
        for (int i = 0; i < 64; i++)
        {
          centerToCirclePoint.rotate(rotateAngle);
          pField = onField + centerToCirclePoint;
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage) && !image.isOutOfImage(pImage.x(), pImage.y(), 3))
          {
            CIRCLE("module:CLIPLineFinder:CenterCircle:Lower", pImage.x(), pImage.y(), 4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
          }
        }
      }
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:CenterCirclePoints:Lower")
      {
        for (int i = 0; i < (int)centerCircles[maxID].pointsOnCircle.size(); i++)
        {
          Vector2f pImage;
          Vector2f pField(centerCircles[maxID].pointsOnCircle[i]);
          if (Transformation::robotToImage(pField, cameraMatrix, cameraInfo, pImage))
            CIRCLE("module:CLIPLineFinder:CenterCirclePoints:Lower", pImage.x(), pImage.y(), 4, 1, Drawings::solidPen, ColorRGBA::orange, Drawings::solidBrush, ColorRGBA::orange);
        }
      }
    }
  }


  // then create field lines and penalty cross from remaining line segments
  segNo = 0;
  bool foundConnection = false;
  while (segNo < (int)lineSegments.size())
  {
    if (lineSegments[segNo].pointNo <= 3 && createPenaltyCross(lineSegments[segNo], upper))
    {
      Vector2f pImage((linePoints[lineSegments[segNo].endPoint].point->inImage + linePoints[lineSegments[segNo].endPoint].point->inImage) * 0.5f);
      Vector2f pField;
      if (Transformation::imageToRobot(pImage, cameraMatrix, cameraInfo, pField)
          && (!localPenaltyCrossPercept.penaltyCrossWasSeen || pField.norm() < localPenaltyCrossPercept.pointOnField.cast<float>().norm()))
      {
        // TODO: check if image point has to be scaled with /RES_DIV_FACTOR
        localPenaltyCrossPercept.pointInImage = pImage.cast<int>();
        localPenaltyCrossPercept.pointOnField = pField.cast<int>();
        localPenaltyCrossPercept.penaltyCrossWasSeen = true;
        localPenaltyCrossPercept.fromUpper = upper;
      }
    }
    if (lineSegments[segNo].onCircle || lineSegments[segNo].pointNo == 1 || std::abs(lineSegments[segNo].avgError) > (maxAvgLineErrorImage * parameterScale)
        || std::abs(lineSegments[segNo].maxError) > (maxTotalLineErrorImage * parameterScale))
    {
      lineSegments.erase(lineSegments.begin() + segNo);
      continue;
    }
    foundConnection = false;
    std::vector<CLIPFieldLinesPercept::FieldLine>& lineVector = upper ? (foundLinesUpper) : (foundLines);

    for (int lineNo = 0; lineNo < (int)lineVector.size(); lineNo++)
    {
      if (connectSegmentToFieldLine(linePoints[lineSegments[segNo].startPoint].point->inImage, linePoints[lineSegments[segNo].endPoint].point->inImage, lineVector[lineNo], upper))
      {
        lineSegments.erase(lineSegments.begin() + segNo);
        foundConnection = true;
        break;
      }
    }
    if (!foundConnection)
    {
      if (lineSegments[segNo].pointNo < minPointsForLine)
      {
        segNoOther = segNo + 1;
        while (segNoOther < (int)lineSegments.size())
        {
          if (lineSegments[segNoOther].pointNo > 1 && connect2Segments(lineSegments[segNo], lineSegments[segNoOther], upper))
          {
            // we have a connection for these 2 segments -> update point connections and make sure no points have the same successor/predecessor
            lineSegments[segNo].pointNo += lineSegments[segNoOther].pointNo;
            if (linePoints[lineSegments[segNo].endPoint].successor != -1)
              linePoints[linePoints[lineSegments[segNo].endPoint].successor].predecessor = -1;
            linePoints[lineSegments[segNo].endPoint].successor = lineSegments[segNoOther].startPoint;
            if (linePoints[lineSegments[segNoOther].startPoint].predecessor != -1)
              linePoints[linePoints[lineSegments[segNoOther].startPoint].predecessor].successor = -1;
            linePoints[lineSegments[segNoOther].startPoint].predecessor = lineSegments[segNo].endPoint;
            lineSegments[segNo].endPoint = lineSegments[segNoOther].endPoint;
            lineSegments.erase(lineSegments.begin() + segNoOther);
            segNoOther--;
          }
          segNoOther++;
        }
      }
      if (lineSegments[segNo].pointNo > minPointsForLine / 2 && createLineFromSingleSegment(lineSegments[segNo], upper))
      {
        lineSegments.erase(lineSegments.begin() + segNo);
        segNo--;
      }
    }
    else
      segNo--;
    segNo++;
  }
}

bool CLIPLineFinder::finishLineSegment(LineSegment& seg, int endPoint, std::vector<Vector2f>& testPoints, const bool& upper)
{
  if (testPoints.size() > 2)
  {
    seg.endPoint = endPoint;
    testPoints.pop_back();
    seg.avgWidth /= seg.pointNo;
    Geometry::Line testLine = Geometry::calculateLineByLinearRegression(testPoints, seg.avgError, seg.maxError);
    if (!verifyLineSegment(seg))
      return false;
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Upper")
    {
      if (upper == drawUpper)
      {
        float y = (linePoints[seg.endPoint].point->inImage.y() > 30) ? linePoints[seg.endPoint].point->inImage.y() - 1 : linePoints[seg.endPoint].point->inImage.y() + 7;
        int nr = (int)lineSegments.size();
        DRAWTEXT("module:CLIPLineFinder:Connections:Upper", linePoints[seg.endPoint].point->inImage.x(), y, 10, ColorRGBA::red, nr);
      }
    }
    DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:Connections:Lower")
    {
      if (upper != drawUpper)
      {
        float y = (linePoints[seg.endPoint].point->inImage.y() > 30) ? linePoints[seg.endPoint].point->inImage.y() - 1 : linePoints[seg.endPoint].point->inImage.y() + 7;
        int nr = (int)lineSegments.size();
        DRAWTEXT("module:CLIPLineFinder:Connections:Lower", linePoints[seg.endPoint].point->inImage.x(), y, 10, ColorRGBA::red, nr);
      }
    }
    return true;
  }
  return false;
}

bool CLIPLineFinder::verifyLineSegment(LineSegment& seg)
{
  // Methods removes points from (non-circle) segments if they do not create a straight line
  // Right now only removes from one end -> TODO: remove from both ends
  float pixelErrorSum = 0.f;
  int originalEndPoint = seg.endPoint;
  //if (std::abs(seg.angleSum) < minAngleSumCircleImage)
  {
    bool errorSumTooHigh = true;
    while (errorSumTooHigh && seg.pointNo > 2)
    {
      Geometry::Line testLine = Geometry::Line(linePoints[seg.startPoint].point->inImage, linePoints[seg.endPoint].point->inImage - linePoints[seg.startPoint].point->inImage);
      int lastPoint = seg.startPoint;
      int endPointToTest = linePoints[seg.startPoint].successor;
      int pointNo = 1;
      float lengthInImage = 0.f;
      bool possibleCircleSegment = true;
      while (pointNo < seg.pointNo)
      {
        testLine.direction = linePoints[endPointToTest].point->inImage - linePoints[seg.startPoint].point->inImage;
        lengthInImage += (linePoints[endPointToTest].point->inImage - linePoints[lastPoint].point->inImage).norm();
        pointNo++;
        int pointToTest = seg.startPoint;
        while (pointToTest != endPointToTest)
        {
          pixelErrorSum += Geometry::getDistanceToLine(testLine, linePoints[pointToTest].point->inImage);
          pointToTest = linePoints[pointToTest].successor;
        }
        lastPoint = endPointToTest;
        endPointToTest = linePoints[endPointToTest].successor;
        if (possibleCircleSegment && lengthInImage > imageWidth / minStraightLineLengthInImageFactor)
        {
          if (std::abs(pixelErrorSum) > imageWidth / pixelErrorSumToImageWidthFactor) //possible circle segment!
          {
            seg.onCircle = true;
            return true;
          }
          else
            possibleCircleSegment = false;
        }
        errorSumTooHigh = std::abs(pixelErrorSum) > imageWidth / pixelErrorSumToImageWidthFactor;
        if (errorSumTooHigh)
        {
          seg.endPoint = linePoints[seg.endPoint].predecessor;
          linePoints[seg.endPoint].onLine = false;
          seg.pointNo--;
          break;
        }
        pixelErrorSum = 0.f;
      }
    }
  }
  if (seg.endPoint != originalEndPoint)
  {
    LINE("module:CLIPLineFinder:verifyLineSegment",
        linePoints[seg.startPoint].point->inImage.x(),
        linePoints[seg.startPoint].point->inImage.y(),
        linePoints[originalEndPoint].point->inImage.x(),
        linePoints[originalEndPoint].point->inImage.y(),
        5,
        Drawings::solidPen,
        ColorRGBA::orange);
    LINE("module:CLIPLineFinder:verifyLineSegment",
        linePoints[seg.startPoint].point->inImage.x(),
        linePoints[seg.startPoint].point->inImage.y(),
        linePoints[seg.endPoint].point->inImage.x(),
        linePoints[seg.endPoint].point->inImage.y(),
        5,
        Drawings::solidPen,
        ColorRGBA::green);
    int secondPoint = linePoints[seg.startPoint].successor;
    int firstPoint = seg.startPoint;
    float lastAngle = (linePoints[secondPoint].point->inImage - linePoints[firstPoint].point->inImage).angle();
    seg.pointNo = 1;
    seg.avgWidth = linePoints[seg.startPoint].point->lineSizeInImage;
    seg.maxError = 0.f;
    seg.angleSum = 0.f;
    Geometry::Line testLine = Geometry::Line(linePoints[seg.startPoint].point->inImage, linePoints[seg.endPoint].point->inImage - linePoints[seg.startPoint].point->inImage);
    while (firstPoint != seg.endPoint)
    {
      seg.pointNo++;
      seg.avgWidth += linePoints[secondPoint].point->lineSizeInImage;
      float newError = Geometry::getDistanceToLine(testLine, linePoints[secondPoint].point->inImage);
      seg.avgError += newError;
      seg.maxError = std::max(seg.maxError, newError);
      float newAngle = (linePoints[secondPoint].point->inImage - linePoints[firstPoint].point->inImage).angle();
      seg.angleSum += newAngle - lastAngle;
      lastAngle = newAngle;
      secondPoint = linePoints[secondPoint].successor;
      firstPoint = linePoints[firstPoint].successor;
    }
    seg.avgError /= seg.pointNo;
    seg.avgWidth /= seg.pointNo;
  }
  return seg.pointNo > 2;
}

void CLIPLineFinder::correctCenterCircle() {}

void CLIPLineFinder::enhanceFieldLines(const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  float lineWidth, lastLineWidth;
  float lineLength;
  bool lineStartWide = false;

  std::vector<Vector2f> checkPoints;
  checkPoints.clear();

  std::vector<CLIPFieldLinesPercept::FieldLine>& lineVector = upper ? (foundLinesUpper) : (foundLines);
  std::vector<CLIPFieldLinesPercept::FieldLine>::iterator line = lineVector.begin();

  while (line != lineVector.end())
  {
    if (upper == drawUpper)
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesRaw:Upper")
      {
        LINE("module:CLIPLineFinder:LinesRaw:Upper", line->startInImage.x(), line->startInImage.y(), line->endInImage.x(), line->endInImage.y(), 3, Drawings::solidPen, ColorRGBA(153, 0, 255));
      }
    }
    else
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesRaw:Lower")
      {
        LINE("module:CLIPLineFinder:LinesRaw:Lower", line->startInImage.x(), line->startInImage.y(), line->endInImage.x(), line->endInImage.y(), 3, Drawings::solidPen, ColorRGBA(153, 0, 255));
      }
    }
    Vector2f lineStart = line->startInImage.cast<float>();
    Vector2f lineEnd = line->endInImage.cast<float>();
    Vector2f lineDir = lineEnd - lineStart;
    Vector2f lineDirField = (line->endOnField - line->startOnField);
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
    lineWidth = line->lineWidthEnd;
    lineDir.normalize(std::max(std::min(lineLength / 5, (float)(imageHeight / 12)), 1.f));

    checkPoints.clear();
    Vector2f scanPoint(lineEnd.x() + lineDir.x(), lineEnd.y() + lineDir.y());
    Vector2f scanDir = lineDir;
    scanDir.rotateLeft();
    Vector2f scanDirNormal(scanDir);
    scanDirNormal.normalize();
    scanDir.normalize(std::max<float>(1.f, line->lineWidthEnd / 10.f));
    scanPoint -= scanDirNormal * (std::max<float>(line->lineWidthEnd, 2.5f));
    Vector2f lastCenter(scanPoint);
    lastLineWidth = lineWidth;

    // from end of line to infinity - aaaaand beyond!
    while (getLineCenterAndWidth(scanPoint, scanDir, lastLineWidth, lastCenter, upper))
    {
      // if lineWidth seems wrong, stop enhancing
      if (lastLineWidth > lineWidth + std::max<float>(1.f, lineWidth / 10) || lastLineWidth < lineWidth - std::max<float>(1.f, lineWidth / 10))
      {
        lastLineWidth = lineWidth;
        break;
      }
      else
      {
        if (lineStartWide)
          lineWidth = std::min<float>(lineWidth, lastLineWidth);
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
    if (distSum > std::max(3.f, (line->lineWidthStart + line->lineWidthEnd) / 5))
    {
      if (lineLength < 1500.f)
        line = lineVector.erase(line);
      else
        line++;
      continue;
    }
    else if (!checkPoints.empty() && Transformation::imageToRobot(checkPoints.back(), cameraMatrix, cameraInfo, line->endOnField))
    {
      line->endInImage = checkPoints.back().cast<int>();
      lineEnd = checkPoints.back();
      line->lineWidthEnd = lastLineWidth;
    }

    lineDir = lineStart - lineEnd;

    lineWidth = line->lineWidthStart;
    lineDir.normalize(std::max(std::min(lineLength / 5, (float)(imageHeight / 12)), 1.f));

    checkPoints.clear();
    scanPoint = lineStart + lineDir;
    scanDir = lineDir;
    scanDir.rotateLeft();
    scanDirNormal = scanDir;
    scanDirNormal.normalize();
    scanDir.normalize(std::max<float>(1.f, line->lineWidthStart / 10.f));
    scanPoint -= scanDirNormal * line->lineWidthStart;
    lastCenter = scanPoint;
    lastLineWidth = lineWidth;

    // from start of line to infinity - aaaaand beyond!
    while (getLineCenterAndWidth(scanPoint, scanDir, lastLineWidth, lastCenter, upper))
    {
      // if lineWidth seems wrong, stop enhancing
      if (lastLineWidth > lineWidth + std::max<float>(1.f, lineWidth / 10) || lastLineWidth < lineWidth - std::max<float>(1.f, lineWidth / 10))
      {
        lastLineWidth = lineWidth;
        break;
      }
      else
      {
        if (lineStartWide)
          lineWidth = std::min(lineWidth, lastLineWidth);
        else
          lineWidth = std::max(lineWidth, lastLineWidth);
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
      distSum += std::abs(Geometry::getDistanceToLine(l, checkPoints[i]));
    }
    if (distSum > std::max<float>(3.f, (line->lineWidthStart + line->lineWidthEnd) / 5))
    {
      if (lineLength < 1500.f)
        line = lineVector.erase(line);
      else
        line++;
    }
    else
    {
      if (!checkPoints.empty() && Transformation::imageToRobot(checkPoints.back(), cameraMatrix, cameraInfo, line->startOnField))
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

bool CLIPLineFinder::getLineCenterAndWidth(const Vector2f scanPoint, const Vector2f scanDir, float& width, Vector2f& center, const bool& upper)
{
  // TODO: use full image here
  const Image& image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const FieldColors& fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  Vector2f checkPoint(scanPoint.x(), scanPoint.y());
  if (image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3))
    return false;

  center = scanPoint;
  float localWidth = 0;
  float lineWidth = 0;
#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
  Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
  int y = p.y;
  int minY = y;
  int maxY = y;
  int startY = y;
  bool foundLineStart = false;
  float stepSize = scanDir.norm();
  float maxWidth = std::max<float>(width * 2, 7.f); //tk: was 6.f

  while (!image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3) && localWidth < maxWidth)
  {
#ifdef USE_FULL_RESOLUTION
    image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
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
    if (foundLineStart && ((y - std::max(fieldColor.fieldColorArray[0].fieldColorOptY, startY)) < fieldColor.fieldColorArray[0].lineToFieldColorYThreshold))
    {
      width = lineWidth;
      center = checkPoint - scanDir * (lineWidth / (2 * stepSize));
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LinesEnhanced:Image")
      {
        if (upper == drawUpper)
          LINE("module:CLIPLineFinder:LinesEnhanced:Image", scanPoint.x(), scanPoint.y(), checkPoint.x(), checkPoint.y(), 1, Drawings::solidPen, ColorRGBA(153, 0, 255));
      }
      return true;
    }
    localWidth += stepSize;
    checkPoint += scanDir;
  }
  return false;
}

bool CLIPLineFinder::addSegmentPointsToCircle(LineSegment& seg, CenterCircle& circle, const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  int nextPoint = seg.startPoint;
  bool onField = true;
  float angleSum = 0;
  float angle = 0, lastAngle = 0;
  float distSum = 0.f;
  int count = 0;
  int sizeOffset = (int)circle.pointsOnCircle.size();
  float angleOfCircleCoveredBySegment = 0.f;
  float minAngle = 2 * pi2;
  float maxAngle = -pi;

  while (nextPoint >= 0 && nextPoint != linePoints[seg.endPoint].successor)
  {
    if (upper == drawUpper)
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Upper")
      {
        DOT("module:CLIPLineFinder:LineSegments:Upper", linePoints[nextPoint].point->inImage.x(), linePoints[nextPoint].point->inImage.y(), ColorRGBA::green, ColorRGBA::green);
      }
    }
    else
    {
      DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Lower")
      {
        DOT("module:CLIPLineFinder:LineSegments:Lower", linePoints[nextPoint].point->inImage.x(), linePoints[nextPoint].point->inImage.y(), ColorRGBA::green, ColorRGBA::green);
      }
    }
    Vector2f pointOnField;
    if (Transformation::imageToRobot(linePoints[nextPoint].point->inImage, cameraMatrix, cameraInfo, pointOnField))
    {
      circle.pointsOnCircle.push_back(pointOnField);
      if (onField && count > 1)
      {
        angle = (circle.pointsOnCircle[sizeOffset + count] - circle.pointsOnCircle[sizeOffset + count - 1]).angle();
        angleSum += (angle - lastAngle);
      }
      else if (onField && count == 1)
      {
        lastAngle = (circle.pointsOnCircle[sizeOffset + 1] - circle.pointsOnCircle[sizeOffset]).angle();
        angle = lastAngle;
      }
      count++;
      if (sizeOffset > 0)
        distSum += getDistancePointToCircle(pointOnField, circle.circle);
    }
    else
    {
      circle.pointsOnCircle.push_back(pointOnField);
      onField = false;
    }
    nextPoint = linePoints[nextPoint].successor;
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

void CLIPLineFinder::removeSegmentPointsFromCircle(LineSegment& seg, CenterCircle& circle)
{
  int size = (int)circle.pointsOnCircle.size();
  ASSERT(size >= seg.pointNo);
  circle.pointsOnCircle.erase(circle.pointsOnCircle.begin() + size - seg.pointNo, circle.pointsOnCircle.end());
}

bool CLIPLineFinder::verifyCircle(const CenterCircle& circle, const bool checkSeenAngle)
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
    dist = getDistancePointToCircle(circle.pointsOnCircle[i], circle.circle);
    //float angle = (circle.pointsOnCircle[i]-circle.circle.center).angle();
    maxDist = std::max(maxDist, dist);
    //minAngle = std::min(minAngle, angle);
    //maxAngle = std::max(maxAngle, angle);
    distSum += dist;
  }
  return distSum / pointNo < maxAvgPointDistToCircleField && maxDist < maxDistPointsToCircleField && pointNo > minPointsForCenterCircle / 2; // && (!checkSeenAngle || (maxAngle-minAngle < 0.7));
}

bool CLIPLineFinder::verifyCenterCircle(const CenterCircle& circle, const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  const float pi_16 = pi_4 / 4;
  const Vector2f pFieldBase(circle.circle.center);

  Vector2f pImageLeft, pImageMiddle, pImageRight;

  Vector2f pFieldCenterToCircle(circle.circle.radius, 0);
  Vector2f pFieldRight = pFieldBase + pFieldCenterToCircle;
  pFieldCenterToCircle.rotate(pi_16);
  Vector2f pFieldMiddle = pFieldBase + pFieldCenterToCircle;
  pFieldCenterToCircle.rotate(pi_16);
  Vector2f pFieldLeft = pFieldBase + pFieldCenterToCircle;
  if (!Transformation::robotToImage(pFieldLeft, cameraMatrix, cameraInfo, pImageLeft))
    pImageLeft.x() = -1;
  if (!Transformation::robotToImage(pFieldMiddle, cameraMatrix, cameraInfo, pImageMiddle))
    pImageMiddle.x() = -1;
  if (!Transformation::robotToImage(pFieldRight, cameraMatrix, cameraInfo, pImageRight))
    pImageRight.x() = -1;

  unsigned int i = 0;
  unsigned int fails = 0;
  unsigned int oks = 0;
  int tries = 16;

  while ((image.isOutOfImage(pImageLeft.x(), pImageLeft.y(), 3) || image.isOutOfImage(pImageMiddle.x(), pImageMiddle.y(), 3) || image.isOutOfImage(pImageRight.x(), pImageRight.y(), 3)) && i < 15)
  {
    Vector2f pFieldRight = pFieldLeft;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldMiddle = pFieldBase + pFieldCenterToCircle;
    pFieldCenterToCircle.rotate(pi_16);
    Vector2f pFieldLeft = pFieldBase + pFieldCenterToCircle;
    if (!Transformation::robotToImage(pFieldLeft, cameraMatrix, cameraInfo, pImageLeft))
      pImageLeft.x() = -1;
    if (!Transformation::robotToImage(pFieldMiddle, cameraMatrix, cameraInfo, pImageMiddle))
      pImageMiddle.x() = -1;
    if (!Transformation::robotToImage(pFieldRight, cameraMatrix, cameraInfo, pImageRight))
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
    if (!Transformation::robotToImage(pFieldLeft, cameraMatrix, cameraInfo, pImageLeft) || !Transformation::robotToImage(pFieldMiddle, cameraMatrix, cameraInfo, pImageMiddle)
        || !Transformation::robotToImage(pFieldRight, cameraMatrix, cameraInfo, pImageRight) || image.isOutOfImage(pImageLeft.x(), pImageLeft.y(), 3)
        || image.isOutOfImage(pImageMiddle.x(), pImageMiddle.y(), 3) || image.isOutOfImage(pImageRight.x(), pImageRight.y(), 3))
      continue;
    else
    {
      // TODO: drawing!
      // TODO: sanity check
      // TODO: check condition - should be between -normal and + normal?
      float lineSize = Geometry::calculateLineSizePrecise(pImageMiddle.cast<int>(), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth);
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

bool CLIPLineFinder::connect2Segments(LineSegment& segA, LineSegment& segB, const bool& upper)
{
  // TODO: check for sensible line width diffs
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  if (linePoints[segA.startPoint].point->isVertical != linePoints[segB.startPoint].point->isVertical)
    return false;
  float segAAngle = (linePoints[segA.endPoint].point->inImage - linePoints[segA.startPoint].point->inImage).angle();
  float segBAngle = (linePoints[segB.endPoint].point->inImage - linePoints[segB.startPoint].point->inImage).angle();
  if (std::abs(Angle::normalize(segAAngle - segBAngle)) > maxSegmentAngleDiffImage)
    return false;

  bool segAFirst = (linePoints[segA.startPoint].point->inImage - linePoints[segB.endPoint].point->inImage).norm()
      > (linePoints[segA.startPoint].point->inImage - linePoints[segB.startPoint].point->inImage).norm();
  Vector2f segDir = segAFirst
      ? linePoints[segB.endPoint].point->inImage - linePoints[segA.startPoint].point->inImage
      : linePoints[segA.endPoint].point->inImage - linePoints[segB.startPoint].point->inImage;
  if (std::abs(Angle::normalize(segAAngle - segDir.angle())) > maxSegmentAngleDiffImage || std::abs(Angle::normalize(segBAngle - segDir.angle())) > maxSegmentAngleDiffImage)
    return false;
  std::vector<Vector2f> pointsOnLine;
  int point = segAFirst ? segA.startPoint : segB.startPoint;
  while (point != (segAFirst ? linePoints[segA.endPoint].successor : linePoints[segB.endPoint].successor))
  {
    pointsOnLine.push_back(linePoints[point].point->inImage);
    point = linePoints[point].successor;
  }
  point = segAFirst ? segB.startPoint : segA.startPoint;
  ;
  while (point != (segAFirst ? linePoints[segB.endPoint].successor : linePoints[segA.endPoint].successor))
  {
    pointsOnLine.push_back(linePoints[point].point->inImage);
    point = linePoints[point].successor;
  }

  // calculate line with linear regression
  float biggestError = 0.0, avgError = 0.0;
  Geometry::Line regLine = Geometry::calculateLineByLinearRegression(pointsOnLine, avgError, biggestError);
  if (avgError > (maxAvgLineErrorImage * parameterScale) || biggestError > (maxTotalLineErrorImage * parameterScale))
    return false;

  // get start/end point
  Vector2f startImage, endImage;
  if (!Geometry::getIntersectionOfLines(regLine, Geometry::Line(segAFirst ? linePoints[segA.startPoint].point->inImage : linePoints[segB.startPoint].point->inImage, Vector2f(0.f, 1.f)), startImage)
      || !Geometry::getIntersectionOfLines(regLine, Geometry::Line(segAFirst ? linePoints[segB.endPoint].point->inImage : linePoints[segA.endPoint].point->inImage, Vector2f(0.f, 1.f)), endImage))
    return false;

  // build Field Line
  CLIPFieldLinesPercept::FieldLine newFL;
  newFL.startInImage = startImage.cast<int>();
  newFL.endInImage = endImage.cast<int>();
  newFL.fromUpper = upper;
  newFL.validity = 0.f;
  newFL.isPlausible = false;

  // project lineSize (not perpendicular to line direction, rather either vertical or horizontal)
  newFL.lineWidthStart = segAFirst
      ? projectLineSize(regLine.direction, linePoints[segA.startPoint].point->lineSizeInImage, linePoints[segA.startPoint].point->isVertical)
      : projectLineSize(regLine.direction, linePoints[segB.startPoint].point->lineSizeInImage, linePoints[segB.startPoint].point->isVertical);
  newFL.lineWidthEnd = segAFirst
      ? projectLineSize(regLine.direction, linePoints[segB.endPoint].point->lineSizeInImage, linePoints[segB.endPoint].point->isVertical)
      : projectLineSize(regLine.direction, linePoints[segA.endPoint].point->lineSizeInImage, linePoints[segA.endPoint].point->isVertical);

  // check if line widths match
  if (segAFirst)
  {
    const float& sizeA = linePoints[segA.endPoint].point->lineSizeInImage;
    const float& sizeB = linePoints[segB.startPoint].point->lineSizeInImage;
    if (std::abs(sizeA - sizeB) > std::max(3.f, std::min(sizeA, sizeB) / 4))
      return false;
  }
  else
  {
    const float& sizeA = linePoints[segA.startPoint].point->lineSizeInImage;
    const float& sizeB = linePoints[segB.endPoint].point->lineSizeInImage;
    if (std::abs(sizeA - sizeB) > std::max(3.f, std::min(sizeA, sizeB) / 4))
      return false;
  }

  // check distance of segment start/end points to line (SHOULD BE REDUNDANT AFTER ANGLE CHECK AT OF METHOD)
  float distSum = getPoint2LineDistance(linePoints[segA.startPoint].point->inImage, linePoints[segA.endPoint].point->inImage, linePoints[segB.endPoint].point->inImage);
  distSum += getPoint2LineDistance(linePoints[segA.startPoint].point->inImage, linePoints[segA.endPoint].point->inImage, linePoints[segB.startPoint].point->inImage);
  distSum += getPoint2LineDistance(linePoints[segB.startPoint].point->inImage, linePoints[segB.endPoint].point->inImage, linePoints[segA.startPoint].point->inImage);
  distSum += getPoint2LineDistance(linePoints[segB.startPoint].point->inImage, linePoints[segB.endPoint].point->inImage, linePoints[segA.startPoint].point->inImage);
  if (distSum < (maxDistSumPointsToLineImage * parameterScale) && Transformation::imageToRobot(newFL.endInImage, cameraMatrix, cameraInfo, newFL.endOnField)
      && Transformation::imageToRobot(newFL.startInImage, cameraMatrix, cameraInfo, newFL.startOnField))
  {
    Vector2f onFieldEndFirst, onFieldStartSecond;
    if (segAFirst && !checkForGreenBetween(linePoints[segA.endPoint].point->inImage, linePoints[segB.startPoint].point->inImage, upper))
    {
      if (!Transformation::imageToRobot(linePoints[segA.endPoint].point->inImage, cameraMatrix, cameraInfo, onFieldEndFirst)
          || !Transformation::imageToRobot(linePoints[segB.startPoint].point->inImage, cameraMatrix, cameraInfo, onFieldStartSecond))
        OUTPUT_WARNING("CLIPLineFinder : imageToRobot failed in connect2Segments");
    }
    else if (!segAFirst && !checkForGreenBetween(linePoints[segB.endPoint].point->inImage, linePoints[segA.startPoint].point->inImage, upper))
    {
      if (!Transformation::imageToRobot(linePoints[segB.endPoint].point->inImage, cameraMatrix, cameraInfo, onFieldEndFirst)
          || !Transformation::imageToRobot(linePoints[segA.startPoint].point->inImage, cameraMatrix, cameraInfo, onFieldStartSecond))
        OUTPUT_WARNING("CLIPLineFinder : imageToRobot failed in connect2Segments");
    }
    else
      return false;

    // connect segments, if distance on field is close enough and build line if line points is enough
    if ((onFieldEndFirst - onFieldStartSecond).norm() < maxSegConnectDistField)
    {
      if (upper == drawUpper)
      {
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Upper")
        {
          LINE("module:CLIPLineFinder:LineSegments:Upper",
              linePoints[segA.endPoint].point->inImage.x(),
              linePoints[segA.endPoint].point->inImage.y(),
              linePoints[segB.startPoint].point->inImage.x(),
              linePoints[segB.startPoint].point->inImage.y(),
              1,
              Drawings::dottedPen,
              ColorRGBA::magenta);
        }
      }
      else
      {
        DEBUG_RESPONSE("debug drawing:module:CLIPLineFinder:LineSegments:Lower")
        {
          LINE("module:CLIPLineFinder:LineSegments:Lower",
              linePoints[segA.endPoint].point->inImage.x(),
              linePoints[segA.endPoint].point->inImage.y(),
              linePoints[segB.startPoint].point->inImage.x(),
              linePoints[segB.startPoint].point->inImage.y(),
              1,
              Drawings::dottedPen,
              ColorRGBA::magenta);
        }
      }
      if (segA.pointNo + segB.pointNo >= minPointsForLine)
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

bool CLIPLineFinder::createLineFromSingleSegment(const LineSegment& seg, const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  std::vector<Vector2f> pointsOnLine;

  int point = seg.startPoint;
  float angleSum = 0;
  float angle = 0, lastAngle = 0;
  int count = 0;
  Vector2f lastPField(0, 0);

  while (point != linePoints[seg.endPoint].successor)
  {
    Vector2f pField(0, 0);
    if (seg.pointNo < 2 * minPointsForLine)
    {
      if (!Transformation::imageToRobot(linePoints[point].point->inImage, cameraMatrix, cameraInfo, pField))
        return false;
      if (count > 1)
      {
        angle = (pField - lastPField).angle();
        angleSum += (lastAngle - angle);
      }
      else if (count == 1)
      {
        lastAngle = angle = (pField - lastPField).angle();
      }
    }
    pointsOnLine.push_back(linePoints[point].point->inImage);
    ASSERT(linePoints[point].successor == linePoints[seg.endPoint].successor || linePoints[point].successor != -1);
    point = linePoints[point].successor;
    count++;
    lastAngle = angle;
    lastPField = pField;
  }
  if (std::abs(angleSum) > maxAngleSumLineField)
    return false;
  Vector2f dir(linePoints[seg.endPoint].point->inImage - linePoints[seg.startPoint].point->inImage);
  return createFieldLine(pointsOnLine,
      projectLineSize(dir, linePoints[seg.startPoint].point->lineSizeInImage, linePoints[seg.startPoint].point->isVertical),
      projectLineSize(dir, linePoints[seg.endPoint].point->lineSizeInImage, linePoints[seg.endPoint].point->isVertical),
      upper);
}

bool CLIPLineFinder::createPenaltyCross(const LineSegment& seg, const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const FieldColors& fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  const Vector2f basePoint((linePoints[seg.endPoint].point->inImage.x() + linePoints[seg.startPoint].point->inImage.x()) / 2.f,
      (linePoints[seg.endPoint].point->inImage.y() + linePoints[seg.startPoint].point->inImage.y()) / 2.f);
  Vector2f checkPoint = basePoint;
  float lineSize = Geometry::calculateLineSizePrecise(Vector2i((int)checkPoint.x(), (int)checkPoint.y()), cameraMatrix, cameraInfo, theFieldDimensions.fieldLinesWidth);
  if (lineSize < imageHeight / 80)
    return false;
  Vector2f scanDir(1.f, 0.f);
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
    while (count < lineSize * 3)
    {
      count++;
      checkPoint += scanDir;
      if (image.isOutOfImage((checkPoint + scanDir).x(), (checkPoint + scanDir).y(), 3))
        return false;
#ifdef USE_FULL_RESOLUTION
      Image::YUVPixel p;
      image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
      Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
      whiteCount[i] += (p.y > fieldColor.fieldColorArray[0].fieldColorOptY + fieldColor.fieldColorArray[0].lineToFieldColorYThreshold && !fieldColor.isPixelFieldColor(p.y, p.cb, p.cr));
    }
    scanDir.rotate(pi_4);
    avgWhiteCount += whiteCount[i];
    minWC = std::min(minWC, whiteCount[i]);
    maxWC = std::max(maxWC, whiteCount[i]);
  }
  avgWhiteCount /= 4;
  bool result = avgWhiteCount > lineSize / 2.f && maxWC < lineSize * 1.5f;
  if (result)
  {
    // verify by checking for white around potential cross
    scanDir.x() = lineSize * 3;
    scanDir.y() = 0;
    for (int i = 0; i < 32; i++)
    {
      checkPoint = basePoint + scanDir;
      if (image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3))
      {
        result = false;
        break;
      }
#ifdef USE_FULL_RESOLUTION
      Image::YUVPixel p;
      image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
      Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
      if (p.y > fieldColor.fieldColorArray[0].fieldColorOptY + fieldColor.fieldColorArray[0].lineToFieldColorYThreshold && !fieldColor.isPixelFieldColor(p.y, p.cb, p.cr))
      {
        result = false;
        break;
      }
      scanDir.rotate(pi / 16);
    }
  }
  CROSS("module:CLIPLineFinder:penaltyCross", basePoint.x(), basePoint.y(), lineSize, 5, Drawings::solidPen, result ? ColorRGBA::blue : ColorRGBA::red);
  return result;
}

void CLIPLineFinder::update(CLIPCenterCirclePercept& theCenterCirclePercept)
{
  wasReset = false;
  execute(false);
  execute(true);
  theCenterCirclePercept = localCenterCirclePercept;
}

void CLIPLineFinder::update(CLIPFieldLinesPercept& theCLIPFieldLinesPercept)
{
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCircle:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCircle:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCirclePoints:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:CenterCirclePoints:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LineSegments:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LineSegments:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesRaw:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesRaw:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesEnhanced:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:LinesEnhanced:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:Connections:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:Connections:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:penaltyCross", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:checkForLineBetween", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:checkForWhiteBetween", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPLineFinder:verifyLineSegment", "drawingOnImage");
  wasReset = false;
  execute(false);
  execute(true);
  theCLIPFieldLinesPercept.lines.clear();
  theCLIPFieldLinesPercept.lines.insert(theCLIPFieldLinesPercept.lines.end(), foundLines.begin(), foundLines.end());
  theCLIPFieldLinesPercept.lines.insert(theCLIPFieldLinesPercept.lines.end(), foundLinesUpper.begin(), foundLinesUpper.end());

  std::sort(theCLIPFieldLinesPercept.lines.begin(), theCLIPFieldLinesPercept.lines.end(), validityHigher);

  checkForPlausability(theCLIPFieldLinesPercept.lines);
}

void CLIPLineFinder::update(PenaltyCrossPercept& thePenaltyCrossPercept)
{
  wasReset = false;
  execute(false);
  execute(true);
  thePenaltyCrossPercept = localPenaltyCrossPercept;
}

bool CLIPLineFinder::connectSegmentToFieldLine(const Vector2f& imgStart, const Vector2f& imgEnd, CLIPFieldLinesPercept::FieldLine& line, const bool& upper)
{
  // TODO: check for sensible line width diffs
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  Vector2f startPointField, endPointField;
  if (!Transformation::imageToRobot(imgStart, cameraMatrix, cameraInfo, startPointField) || !Transformation::imageToRobot(imgEnd, cameraMatrix, cameraInfo, endPointField))
    return false;

  bool segFirst = (startPointField - line.endOnField).norm() > (startPointField - line.startOnField).norm();

  // check for distance between segment and line, also check for distance of points to new line
  float distSum = 0.f;
  if ((segFirst && (line.startOnField - endPointField).norm() > maxSegConnectDistField) || ((line.endOnField - startPointField).norm() > maxSegConnectDistField))
    return false;

  distSum += getPoint2LineDistance(line.startInImage, line.endInImage, imgStart);
  distSum += getPoint2LineDistance(line.startInImage, line.endInImage, imgEnd);
  if (distSum > (maxDistSumPointsToLineImage * parameterScale))
    return false;

  float segAngle = (imgEnd - imgStart).angle();
  float lineAngle = (line.endInImage - line.startInImage).cast<float>().angle();
  if (std::abs(Angle::normalize(segAngle - lineAngle)) > maxSegmentAngleDiffImage)
    return false;

  Vector2f segDir =
      segFirst ? Vector2f(line.endInImage.x() - imgStart.x(), line.endInImage.y() - imgStart.y()) : Vector2f(imgEnd.x() - line.startInImage.x(), imgEnd.y() - line.startInImage.y());

  if (std::abs(Angle::normalize(segAngle - segDir.angle())) > maxSegmentAngleDiffImage || std::abs(Angle::normalize(lineAngle - segDir.angle())) > maxSegmentAngleDiffImage)
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

bool CLIPLineFinder::createFieldLine(const std::vector<Vector2f>& pointsOnLine, const float& lineWidthStart, const float& lineWidthEnd, const bool& upper)
{
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  float biggestError = 0.f, avgError = 0.f;
  Geometry::Line regLine = Geometry::calculateLineByLinearRegression(pointsOnLine, avgError, biggestError);
  if (avgError > (maxAvgLineErrorImage * parameterScale) || biggestError > (maxTotalLineErrorImage * parameterScale))
    return false;

  CLIPFieldLinesPercept::FieldLine newFL;

  Vector2f startImage, endImage;
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
  if (!Transformation::imageToRobot(startImage, cameraMatrix, cameraInfo, newFL.startOnField) || !Transformation::imageToRobot(endImage, cameraMatrix, cameraInfo, newFL.endOnField))
    return false;
  float distSum = 0.f;
  Vector2f newFLDir = (newFL.endOnField - newFL.startOnField);
  Vector2f lineDir;
  std::vector<CLIPFieldLinesPercept::FieldLine>& lineVector = upper ? (foundLinesUpper) : (foundLines);

  for (int i = 0; i < (int)lineVector.size(); i++)
  {
    distSum = getPoint2LineDistance(lineVector[i].startOnField, lineVector[i].endOnField, newFL.startOnField);
    distSum += getPoint2LineDistance(lineVector[i].startOnField, lineVector[i].endOnField, newFL.endOnField);
    lineDir = (lineVector[i].endOnField - lineVector[i].startOnField);
    if (distSum < maxDistSumMergeLinesField && std::abs(newFLDir.angle() - lineDir.angle()) < 0.8f)
    {
      // <<<<<<<<<<<<<<<<< MERGE LINES HERE ??? >>>>>>>>>>>>>>>>>>
      if (newFLDir.norm() > lineDir.norm())
      {
        lineVector.erase(lineVector.begin() + i);
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

bool CLIPLineFinder::checkForLineBetween(const Vector2f& imgStart, const Vector2f& imgEnd, const float& lineSizeInImage, const bool& upper)
{
  LINE("module:CLIPLineFinder:checkForLineBetween", imgStart.x(), imgStart.y(), imgEnd.x(), imgEnd.y(), 3, Drawings::solidPen, ColorRGBA(138, 43, 226));
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors& fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  // first check for 'white' on field line (2 (even 1?) check(s) should be enough)
  float safetyBuffer = (float)(imageHeight / 40);
  Vector2f lineDir = imgEnd - imgStart;
  Vector2f lineDirNormal = lineDir;
  lineDirNormal.rotateLeft();
  lineDirNormal.normalize(1);
  Vector2f checkPoint = imgStart + lineDir / 2 - lineDirNormal * (safetyBuffer / 2 + lineSizeInImage / 2);
  int lineStart = 0, pointNo = 0, lineEnd = 0;
  if (image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3))
    return false;
#ifdef USE_FULL_RESOLUTION
  Image::YUVPixel p;
  image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
  Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
  int lastY = p.y;
  int y = lastY;

  while (pointNo <= lineSizeInImage + safetyBuffer && !image.isOutOfImage(checkPoint.x(), checkPoint.y(), 3))
  {
#ifdef USE_FULL_RESOLUTION
    image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
    p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    y = p.y;
    if (lineEnd == 0 && y - lastY > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold / 2)
      lineStart = pointNo;
    if (lastY - y > fieldColor.fieldColorArray[0].lineToFieldColorYThreshold / 2)
      lineEnd = pointNo;

    checkPoint += lineDirNormal;
    lastY = y;
    pointNo++;
  }
  if (lineStart == 0 || lineEnd == 0 || lineStart >= lineEnd
      || std::abs(lineSizeInImage - (float)(lineEnd - lineStart)) >= std::max<float>(std::min<float>(4.5f, lineSizeInImage / 6), 1.5f) || (lineStart - safetyBuffer) >= safetyBuffer / 3)
    return false;
  return true;
}


bool CLIPLineFinder::verifyLine(const CLIPFieldLinesPercept::FieldLine& line, const bool& upper)
{
  return checkForLineBetween(Vector2f(line.startInImage.cast<float>()), Vector2f(line.endInImage.cast<float>()), 0.5f * (line.lineWidthStart + line.lineWidthEnd), upper);
}

bool CLIPLineFinder::checkForGreenBetween(const Vector2f& startInImage, const Vector2f& endInImage, const bool& upper)
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors& fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  int length = static_cast<int>((endInImage - startInImage).norm());
  int sampleDistance = std::max(length / 10, 3);
  int sampleCount = length / sampleDistance;
  int steps = 0;
  int greenCount = 0;
  if (sampleCount < 5)
    return true;
  Vector2f checkPoint(startInImage);
  Vector2f scanDir(endInImage - startInImage);
  scanDir.normalize((float)sampleDistance);
  while (steps < sampleCount && !image.isOutOfImage(checkPoint.x(), checkPoint.y(), 2))
  {
#ifdef USE_FULL_RESOLUTION
    Image::YUVPixel p;
    image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
    Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    greenCount += fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
    steps++;
    checkPoint += scanDir;
  }
  if (greenCount >= steps / 2 || steps < 5)
    return true;
  return false;
}

bool CLIPLineFinder::checkForWhiteBetween(const Vector2f& startInImage, const Vector2f& endInImage, const float& lineSize, const bool& upper)
{
  LINE("module:CLIPLineFinder:checkForWhiteBetween", startInImage.x(), startInImage.y(), endInImage.x(), endInImage.y(), 3, Drawings::solidPen, ColorRGBA(138, 43, 226));

  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors& fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  int length = (int)(endInImage - startInImage).norm();
  int sampleDistance = std::max(length / 20, 1);
  int sampleCount = length / sampleDistance;
  int steps = 0;
  int greenCount = 0;
  if (sampleCount < 5)
    return true;
  Vector2f checkPoint(startInImage);
  Vector2f scanDir(endInImage - startInImage);
  scanDir.normalize((float)sampleDistance);
  int greenWhiteY = fieldColor.fieldColorArray[0].fieldColorOptY;
  int whiteCount = 0;
  while (steps < sampleCount && !image.isOutOfImage(checkPoint.x(), checkPoint.y(), 2))
  {
#ifdef USE_FULL_RESOLUTION
    Image::YUVPixel p;
    image.getPixel((unsigned int)checkPoint.x, (unsigned int)checkPoint.y, &p);
#else
    Image::Pixel p = image[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()];
#endif
    if (fieldColor.isPixelFieldColor(p.y, p.cb, p.cr))
    {
      greenCount++;
      greenWhiteY += p.y;
    }
    else if (p.y > (greenWhiteY / (greenCount + 1)) + fieldColor.fieldColorArray[0].lineToFieldColorYThreshold)
      whiteCount++;
    steps++;
    checkPoint += scanDir;
  }
  if (greenCount >= steps / 4 && whiteCount >= std::max(lineSize * 0.75f, 1.f))
    return true;
  return false;
}

void CLIPLineFinder::removeCenterCircleTangents(const bool& upper)
{
  if (!localCenterCirclePercept.centerCircleWasSeen)
    return;

  std::vector<CLIPFieldLinesPercept::FieldLine>& lineVector = upper ? (foundLinesUpper) : (foundLines);
  std::vector<CLIPFieldLinesPercept::FieldLine>::iterator i = lineVector.begin();
  float distSum = 0.f;

  while (i != lineVector.end())
  {
    distSum = 0.f;
    Vector2f endToCircle(localCenterCirclePercept.centerCircle.locationOnField.x() - i->endOnField.x(), localCenterCirclePercept.centerCircle.locationOnField.y() - i->endOnField.y());
    distSum += std::abs((endToCircle.norm() - theFieldDimensions.centerCircleRadius));
    Vector2f startToCircle(
        localCenterCirclePercept.centerCircle.locationOnField.x() - i->startOnField.x(), localCenterCirclePercept.centerCircle.locationOnField.y() - i->startOnField.y());
    distSum += std::abs(startToCircle.norm() - theFieldDimensions.centerCircleRadius);
    Vector2f middleToCircle(localCenterCirclePercept.centerCircle.locationOnField.x() - (i->endOnField.x() + i->startOnField.x()) / 2,
        localCenterCirclePercept.centerCircle.locationOnField.y() - (i->endOnField.y() + i->startOnField.y()) / 2);
    distSum += std::abs(middleToCircle.norm() - theFieldDimensions.centerCircleRadius);
    if (distSum < maxDistSegToCenterCircleField)
      i = lineVector.erase(i);
    else
      i++;
  }
}

MAKE_MODULE(CLIPLineFinder, perception)
