/**
* @file CLIPLineFinder.h
*
* Definition of class CLIPLineFinder
* Scans for penalty cross and field lines (including center circle), given the line points from preprocessing.
* @author <a href="mailto:ingmar.schwarz@uni-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CLIPPointsPercept.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/FieldColor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/DebugImages.h"
#include <algorithm>


MODULE(CLIPLineFinder,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CLIPPointsPercept),
  REQUIRES(FrameInfo),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  PROVIDES(CLIPFieldLinesPercept),
  PROVIDES(CLIPCenterCirclePercept),
  PROVIDES(PenaltyCrossPercept),
  LOADS_PARAMETERS(
  {,
    (float) maxDistPointsImage, /**< max distance for connection of two points */
    (short) maxNoDistLinesImage, /**< max # of scan line number distance for connection of points (contain line number) */
    (float) maxDistSumPointsToLineImage, /**< max summed distance of points to line segment at creation */
    (float) maxAngleSumLineImage, /**< max summed angle between point connections to be considered a line in image (for connections) */
    (float) maxAngleSumLineField, /**< max summed angle between point connections to be considered a line in field (for cc creation) */
    (float) minAngleSumCircleImage, /**< min summed angle in image on line segment to be considered part of circle */
    (float) maxDistSumFactorImage, /**< max divergence in summed distance to last sum at line segment creation */
    (float) maxDistSumMergeLinesField, /**< max sum of start and end point distance from one line to another */
    (float) maxDistPointsToCircleField, /**< max distance of any circle point to their circle */
    (float) maxAvgPointDistToCircleField, /**< max avg dist of all circle points to their circle */
    (float) maxDistSegToCenterCircleField, /**< max distance of line or segment start/middle/end points to circle (line will be removed) */
    (float) maxDistSegToCenterCircleImage, /**< .. same in image coords */
    (float) maxCenterCircleRadiusDiffField, /**< max difference in center circle radius to radius defined by field dimensions */
    (float) minFieldLineLength, /**< minimum length for a field line (in field coords) */
    (float) minDefiniteFieldLineLength, /**< if a field line is THAT LONG (in field coords), it must be a real one.. */
    (int) minPointsForLine, /**< minimum points for a segment to form a line */
    (int) minPointsForCenterCircle, /**< minimum number of points to form a center circle */
    (float) maxSegConnectDistField, /**< maximum distance of segments/lines for a possible connection */
    (float) maxSegConnectDistImage, /**< .. same in image coords */
    (float) maxAvgLineErrorImage, /**< max avg error of linear regression through points to form a line */
    (float) maxTotalLineErrorImage, /**< max biggest error of linear regression through points to form a line */
    (float) maxAvgCircleErrorImage, /**< max avg error of linear regression through points to be part of a circle */
    (float) maxTotalCircleErrorImage, /**< max biggest error of linear regression through points to be part of a circle */
    (float) maxSegmentAngleImage, /**< max angle between point connections to be on the same segment (radian)*/
    (float) maxSegmentAngleDiffImage, /** max angle difference between two segments to connect (radian)*/
    (float) maxValidityDenominator, /**< denominator of image diagonal length for full validity */
  }),
});

class CLIPLineFinder : public CLIPLineFinderBase
{
public:
  CLIPLineFinder();

  struct LinePoint
  {
    const CLIPPointsPercept::Point *point;
    LinePoint *predecessor;
    LinePoint *successor;
    bool onLine;
  };

  // representing center circle (in field coordinates!)
  struct CenterCircle
  {
    Geometry::Circle circle;
    std::vector< Vector2f > pointsOnCircle; // points are in field coordinates
    bool upper;
  };

  // representing possible line/center circle segment (in image coordinates!)
  struct LineSegment
  {
  public:
    LinePoint *startPoint;
    LinePoint *endPoint;
    float avgError;
    float maxError;
    int pointNo;
    float avgWidth;
    float angleSum;
    bool onCircle;
  };

  void reset();

  void execute(const bool &upper);

  void update(CLIPFieldLinesPercept &theCLIPFieldLinesPercept);
  void update(CLIPCenterCirclePercept &theCenterCirclePercept);
  void update(PenaltyCrossPercept &thePenaltyCrossPercept);

  std::vector<CLIPFieldLinesPercept::FieldLine> foundLines;
  std::vector<CLIPFieldLinesPercept::FieldLine> foundLinesUpper;
  CLIPCenterCirclePercept localCenterCirclePercept;
  PenaltyCrossPercept localPenaltyCrossPercept;
  std::vector<LineSegment> lineSegments;
  std::vector<LinePoint> linePoints;
  std::vector<CenterCircle> centerCircles;

private:
  unsigned lastExecutionTimeStamp;
  unsigned lastExecutionTimeStampUpper;

  float parameterScale; // some parameters scale with resolution

  int imageHeight, imageWidth;

  bool wasReset;

  // for debugging
  bool drawUpper;

  // connect points that are close enough
  void connectPoints();
  // split connected points to fitting line segments
  void createSegments(const bool &upper);
  // connect small segments and/or create center circle/field lines
  void connectSegments(const bool &upper);
  // remove lines that are on or near and tangent to center circle
  void removeCenterCircleTangents(const bool &upper);
  // try to maximize lines
  void enhanceFieldLines(const bool &upper);
  // try to correct center circle percept with the middle line
  void correctCenterCircle();

  bool addSegmentPointsToCircle(LineSegment &seg, CenterCircle &circle, const bool &upper);

  void removeSegmentPointsFromCircle(LineSegment &seg, CenterCircle &circle);

  /**
  * Verifies circle by checking sum of distances to center
  **/
  bool verifyCircle(const CenterCircle &circle, const bool checkSeenAngle);

  /**
  * Verifies final center circle with checks for field green
  * TODO: Currently unused
  **/
  bool verifyCenterCircle(const CenterCircle &circle, const bool &upper);

  /**
  * TODO : Merge similar lines!
  * expects first point of vector to be start point, last to be end point
  * return True if points build a straight line, line is not on circle and is not matching another existing line
  */
  bool createFieldLine(
    const std::vector< Vector2f > &pointsOnLine,
    const float &lineWidthStart,
    const float &lineWidthEnd,
    const bool &upper);

  /**
  * Verifies field line with check for white between start and end.
  * maybe check for gradient at start/end ? or at least only for white,
  * since this removes some good lines... (especially with obstacles)
  * @param line The line in question.
  * return True if white was found.
  */
  bool verifyLine(const CLIPFieldLinesPercept::FieldLine &line, const bool &upper);

  bool checkForLineBetween(
    const Vector2f &imgStart,
    const Vector2f &imgEnd,
    const float &lineSizeInImage,
    const bool &upper);

  bool getLineCenterAndWidth(
    const Vector2f scanPoint,
    const Vector2f scanDir,
    float &width, Vector2f &center,
    const bool &upper);

  /**
  * connect two line segments, if they are close enough and the angle is similar
  * @param segA The first segment.
  * @param segB The second segment.
  * return True if connection was successful
  */
  bool connect2Segments(LineSegment &segA, LineSegment &segB, const bool &upper);
  bool createLineFromSingleSegment(const LineSegment &seg, const bool &upper);
  bool createPenaltyCross(const LineSegment &seg, const bool &upper);
  bool connectSegmentToFieldLine(
    const Vector2f &imgStart,
    const Vector2f &imgEnd,
    CLIPFieldLinesPercept::FieldLine &line,
    const bool &upper);

  bool checkForGreenBetween(const Vector2f &startInImage, const Vector2f &endInImage, const bool &upper);
  bool checkForWhiteBetween(const Vector2f &startInImage, const Vector2f &endInImage, const float &lineSize, const bool &upper);


  // helper functions

  /** Sets plausability for each line.
  * Assumes that line with highest validity comes first.
  */
  static void checkForPlausability(std::vector<CLIPFieldLinesPercept::FieldLine>& lines)
  {
    const float maxAwayLength = 9000;

    if (lines.empty())
      return;

    std::vector<CLIPFieldLinesPercept::FieldLine>::iterator line = lines.begin();
    std::vector<CLIPFieldLinesPercept::FieldLine>::const_iterator end = lines.end();

    float lineAngle = pi_2 - (line->endOnField - line->startOnField).angle();
    while (std::abs(lineAngle) >= pi_4)
    {
      lineAngle += pi_2;
      lineAngle = Angle::normalize(lineAngle);
    }
    Vector2f lineCenter = (line->endOnField + line->startOnField) / 2.;
    line->isPlausible = true; // Line with highest validity is reference line and therefore plausible
    line++;
    for (; line < end; line++)
    {
      float currAngle = pi_2 - (line->endOnField - line->startOnField).angle();
      while (std::abs(currAngle) >= pi_4)
      {
        currAngle += pi_2;
        currAngle = Angle::normalize(currAngle);
      }
      Vector2f actCenter = (line->endOnField + line->startOnField) / 2.;
      float diff = Angle::normalize(lineAngle - currAngle);
      float deg = toDegrees(diff);
      float lengthMulti = 1 - (lineCenter - actCenter).norm() / maxAwayLength;

      line->isPlausible = ((std::abs(deg) * lengthMulti) <= 10 || ((90 - std::abs(deg)) * lengthMulti) <= 10);
    }
  }

  inline float getPoint2LineDistance(const Vector2i &lineStart, const Vector2i &lineEnd, const Vector2f &point)
  {
    Vector2f normal((lineEnd-lineStart).cast<float>());
    normal.rotateRight();
    normal.normalize();
    float distance = (point - Vector2f(lineStart.cast<float>())).dot(normal);
    return std::abs(distance);
  }

  inline float getPoint2LineDistance(const Vector2f &lineStart, const Vector2f &lineEnd, const Vector2f &point)
  {
    Vector2f normal(lineEnd.x() - lineStart.x(), lineEnd.y() - lineStart.y());
    normal.rotateRight();
    normal.normalize();
    float distance = (point - lineStart).dot(normal);
    return std::abs(distance);
  }

  float projectLineSize(const Vector2f &direction, const float &lineSize, const bool &vertical)
  {
    Vector2f dirNormal = direction;
    dirNormal.rotateLeft();
    float alpha = vertical ? dirNormal.angle() : direction.angle();
    return std::abs(lineSize*std::sin(alpha));
  }

  inline int clipToImageHeight(const int &toClip)
  {
    return std::min(imageHeight - 1, std::max(toClip, 0));
  }

  inline int clipToImageWidth(const int &toClip)
  {
    return std::min(imageWidth - 1, std::max(toClip, 0));
  }

  inline float getDistancePointToCircle(const Vector2f &point, const Geometry::Circle &circle)
  {
    return std::abs(circle.radius - (circle.center - point).norm());
  }

};