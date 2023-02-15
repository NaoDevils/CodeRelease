/**
* @file CLIPPreprocessor.h
* Declaration of class CLIPPreprocessor.
* Scans upper and lower images along fixed scan lines and seperates them into segments used in later modules.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CLIPPointsPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/RingBufferWithSum.h"
#include <algorithm>

MODULE(CLIPPreprocessor,
  REQUIRES(FallDownState),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  USES(RobotPose),
  PROVIDES(CLIPPointsPercept),
  PROVIDES(BallSpots),
  PROVIDES(ObstacleBasePoints),
  LOADS_PARAMETERS(,
    // TODO: initialized has to be changed when changing parameters!!
    (int) hScanLineDistanceLower, // distance of horizontal scan lines in lower image (320 image, will be scaled)
    (int) vScanLineDistanceLower, // distance of vertical scan lines in lower image (320 image, will be scaled)
    (int) hScanLineDistanceUpper, // distance of horizontal scan lines in upper image (320 image, will be scaled)
    (int) vScanLineDistanceUpper, // distance of vertical scan lines in upper image (320 image, will be scaled)
    (int) yellowGoalColorMinDiff, // min cr and cb channel diff for goal->nonGoal transition or v.v.
    (int) fieldBorderMaxDistance, // max distance in pixels of field end point to field end line to be inlier (RANSAC used)
    (int) fieldBorderMinPoints, // min number of inliers for a field end line
    (float)(0.3f) minFieldColorForFieldSegment, // min ratio of field color for segment to be a field segment
    (int) obstacleMaxPointsLow, // max count of scanlines to check
    (unsigned) obstacleMinPointsLow, // min count of scanlines for counting as obstacle segment
    (unsigned) obstacleMinPointsSide, // min number of points for an obstacle line -> base of obstacle percept
    (int) minColorDiff, // min sum of color diffs distance in (crDiff+cbDiff)
    (int) ballBaseCrValue, // min value of cr channel to be expecting ball (to do, only used in old approach (processScanLine(..))
    (bool) useAreaBasedFieldColor, // use are based field color?
    (bool) useObstacleBasePoints,
    (bool) sortBallSpots
  )
);

class CLIPPreprocessor : public CLIPPreprocessorBase
{
public:
  /**
  * Default constructor.
  */
  CLIPPreprocessor();

  DECLARE_DEBUG_IMAGE(SIPField);
  DECLARE_DEBUG_IMAGE(SIPFieldUpper);

  /** Destructor */
  ~CLIPPreprocessor();

  struct FieldEndPoint
  {
    Vector2f imageCoordinates;
    bool inlier;
  };

  enum ScanLineSegmentType
  {
    fieldSegment,
    lineSegment,
    ballSegment,
    obstacleSegment, // meaning dynamic obstacles on field, not covering goalposts
    unknownSegment,
    numOfScanLineSegmentType
  };

  class ScanLineSegment
  {
  public:
    ScanLineSegment(const float& x, const float& y, const int& fieldCount, bool hasColorChangeAtStart, ScanLineSegmentType type)
    {
      startPointInImage.x() = x;
      startPointInImage.y() = y;
      segmentType = type;
      endPointInImage = Vector2f(0, 0);
      gradientColorStart = hasColorChangeAtStart;
      gradientColorEnd = false;
      fieldColorCount = fieldCount;
      avgCr = 0;
      avgCb = 0;
      avgY = 0;
    }

    ScanLineSegment(const float& x, const float& y, const float& maxLength, ScanLineSegmentType type)
    {
      startPointInImage.x() = x;
      startPointInImage.y() = y;
      segmentType = type;
      endPointInImage = Vector2f(0, 0);
      gradientColorStart = false;
      gradientColorEnd = false;
      fieldColorCount = 0;
      avgCr = 0;
      avgCb = 0;
      avgY = 0;
    }

    ScanLineSegmentType segmentType;
    Vector2f startPointInImage; // starting point
    Vector2f endPointInImage; // ending point
    bool gradientColorStart,
        gradientColorEnd; // did segment start/end with possible color change?
    int fieldColorCount;
    int avgCr;
    int avgCb;
    int avgY;
  };

  class ScanLine
  {
  public:
    ScanLine()
    {
      from = Vector2i(0, 0);
      to = Vector2i(0, 0);
      stepSize = 1;
      fullScanLine = false;
      scanLineSegments.reserve(100);
    }
    ScanLine(const Vector2i& _from, const Vector2i& _to, int _stepSize, bool _fullScanLine)
    {
      from = _from;
      to = _to;
      stepSize = _stepSize;
      fullScanLine = _fullScanLine;
      scanLineSegments.reserve(50);
    }
    ~ScanLine() { clear(); }
    std::vector<ScanLineSegment> scanLineSegments;
    Vector2i from;
    Vector2i to;
    int stepSize;
    bool fullScanLine;

    void clear() { scanLineSegments.clear(); };
  };

private:
  void update(CLIPPointsPercept& theCLIPPointsPercept);
  void update(BallSpots& ballSpots);
  void update(ObstacleBasePoints& obstacleBasePoints);

  /*
  * Reset all local percepts, only to be called once a frame!
  */
  void reset();
  void execute(const bool& upper); // if new image is available, executes scanning process (once per frame and image)
  void createScanLines();
  void scanField(const bool& upper); // create scan lines
  void createObstacleBasePoints(const bool& upper); // add possible obstacles from obstacle points
  void postProcessScanLine(const ScanLine& scanLine, const bool& upper); // get additional info from finished ScanLine
  void classifyScanLineSegments(ScanLine& scanLine, const bool& upper); // run after processScanLine - classifies segments

  /*
                                                                        * Run one scanLine over field, generates unclassified scan line segments, no percepts generated here yet.
                                                                        * @param scanLine The Scan Line in question.
                                                                        * @param upper True if upper image will be scanned.
                                                                        */
  void processScanLine(ScanLine& scanLine, const bool& upper);

  /*
  * Adds a ball spot.
  * @param point The center point of the ball segment.
  * @param y The average y-value on the ball segment.
  * @param cb The average cb-value on the ball segment.
  * @param cr The average cr-value on the ball segment.
  * @param upper True if from upper image.
  */
  void addBallSpot(const Vector2f& point, const int& y, const int& cb, const int& cr, const bool& upper);

  /*
  * Adds a ball spot.
  * @param linePoint The center point of the line segment.
  * @param lineSize The length of the line segment.
  * @param isVertical True if line segment was on vertical scan line.
  * @param upper True if from upper image.
  */
  void addLinePoint(const Vector2f& linePoint, const float& lineSize, bool isVertical, const bool& upper);

  /*
  * Finds field border(s).
  */
  void findFieldBorders();

  /*
  * The same function as provided in FieldColor.h,
  * but faster since some values are precomputed,
  * TODO : work around this
  */
  inline bool isPixelBallColor(const int& y, const int& cb, const int& cr) { return cr > minBallCr && cb > minBallCb && cb < maxBallCb; }

  /** return sum of cb and cr channel differences, used to detect changes in color on scan lines */
  inline int lastColorDiff()
  {
    return std::abs(scanLinePixelBuffer[pixelCount - 1].cb - scanLinePixelBuffer[pixelCount].cb + scanLinePixelBuffer[pixelCount - 1].cr - scanLinePixelBuffer[pixelCount].cr);
  }

  /** return sum of cb and cr channel differences, used to detect changes in color on scan lines */
  inline int lastColorDiff2()
  {
    return std::abs(scanLinePixelBuffer[pixelCount - 2].cb - scanLinePixelBuffer[pixelCount].cb + scanLinePixelBuffer[pixelCount - 2].cr - scanLinePixelBuffer[pixelCount].cr);
  }

  unsigned timeStamp, timeStampUpper; // used to make sure that images are only processed once
  bool wasReset;
  bool initialized;

  int imageWidth, imageHeight;

  Geometry::Line horizon;

  std::vector<ScanLine> scanLinesVerticalLower; // vertical scan lines - to find field lines, field end, obstacles and ball
  std::vector<ScanLine> scanLinesVerticalUpper; // vertical scan lines - to find field lines, field end, obstacles and ball
  std::vector<ScanLine> scanLinesHorizontalLower; // horizontal scan lines - to find goal, field lines, obstacles and ball
  std::vector<ScanLine> scanLinesHorizontalUpper; // horizontal scan lines - to find goal, field lines, obstacles and ball
  int scanLineVNo, scanLineHNo; // remember number of current scan lines (needed for line spots)
  std::vector<Image::Pixel> scanLinePixelBuffer;
  RingBufferWithSum<int, 8> fieldColorBuffer;
  std::vector<float> lineSizes;

  // for field end detection
  std::vector<FieldEndPoint> fieldEndPoints;
  std::vector<Vector2f> fieldHull;
  Geometry::Line fieldBorderFront, fieldBorderLeft, fieldBorderRight;

  // scan line stuff
  // all unused
  //  int noVScanLinesVerticalLower;
  //  int noVScanLinesVerticalUpper;
  //  int noHScanLinesVerticalLower;
  //  int noHScanLinesVerticalUpper;
  int scanLineNoYStart;
  unsigned pixelCount;

  // field color related vars (constant for one image, used to improve speed of ball color check)
  int minBallCb;
  int maxBallCb;
  int minBallCr;

  // for obstacle detection
  std::vector<Vector2i> obstaclePointsLow;
  std::vector<Vector2i> obstaclePointsHigh;
  std::vector<Vector2i> obstaclePointsLeft;
  std::vector<Vector2i> obstaclePointsRight;

  // local percepts
  CLIPPointsPercept localCLIPPointsPercept;
  BallSpots localBallSpots;
  ObstacleBasePoints localObstacleBasePoints;

  // debugging stuff

  void drawFieldLower();
  void drawFieldUpper();
  void drawFieldHull(const bool& upper);
  void drawScanLineSegments(const bool& upper);
  void drawSegment(std::vector<ScanLineSegment>::const_iterator seg, const bool& upper);
};