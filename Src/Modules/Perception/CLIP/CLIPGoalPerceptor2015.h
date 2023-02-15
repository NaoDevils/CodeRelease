/**
* @file CLIPGoalPerceptor2015.h
* Declaration of class CLIPGoalPerceptor2015.
* Scans for possible goal posts, given (vertical) goal post segments including their average YUV-values.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Tools/Math/Eigen.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

MODULE(CLIPGoalPerceptor2015,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(JointSensorData), // for validity checks
  USES(RobotPose), // for validity checks
  PROVIDES(CLIPGoalPercept),
  LOADS_PARAMETERS(,
    (bool) verifyWidth,/**< If true, after finding edges and connecting, additional width scans are performed. */
    (bool) useWorldModel, /**< If true, the RobotPose is used to verify the goal posts */
    (bool) useGoalColor, /**< If true, expected y,cb,cr values have to match avg color from scans. */
    (bool) acceptWithoutBase, /**< If true, goal post may be accepted even if no base was found. */
    (int) expectedGoalCb, /**< see useGoalColor */
    (int) expectedGoalCr, /**< see useGoalColor */
    (int) expectedGoalY, /**< see useGoalColor */
    (int) maxCbDiffOnGoal, /**< Max Cb-channel difference allowed while on goal post. */
    (int) maxCrDiffOnGoal, /**< Max rb-channel difference allowed while on goal post. */
    (int) gradientMinDiff, /**< Y-channel difference for scan line segmentation. */
    (float) minValidity, /**< Min validity of goal post to be accepted. */
    (int) numberOfScanLines /**< Number of ScanLines used for goal segment scan. */
  )
);

class CLIPGoalPerceptor2015 : public CLIPGoalPerceptor2015Base
{
public:
  /**
  * Default constructor.
  */
  CLIPGoalPerceptor2015()
  {
    goalPosts.reserve(15);
    goalSideSpots.reserve(100);
    colorSegments.reserve(20);
  }

  struct GoalPost
  {
    GoalPost()
        : bottomFound(false), foundLineAtBottom(false), avgY(0), avgCb(0), avgCr(0), topWidth(-1.f), bottomWidth(-1.f), validity(0.f), startInImage(Vector2f::Zero()),
          endInImage(Vector2f::Zero()), direction(Vector2f::Zero()), locationOnField(Vector2f::Zero())
    {
    }
    bool bottomFound;
    bool foundTop;
    bool foundLineAtBottom;
    int avgY;
    int avgCb;
    int avgCr;
    float topWidth;
    float bottomWidth;
    float validity; // using width, location on field, color(+similarity), field lines, (world model?), TODO..
    Vector2f startInImage;
    Vector2f endInImage;
    Vector2f direction;
    Vector2f locationOnField; // based on endInImage+0.5*bottomWidth
  };

  struct GoalSideSpot
  {
    bool used;
    int y, cb, cr;
    int xPos, yPos;
    float angle; // using sobel (getAngle())
    GoalSideSpot* nextSpot;
  };

  // helper struct to identify different colors on a possible goal
  struct ColorSegment
  {
    ColorSegment() : avgCb(0), avgCr(0), avgY(0), length(0) {}
    int avgCb;
    int avgCr;
    int avgY;
    int length;
  };

  unsigned lastImageTimeStamp;

  std::vector<GoalPost> goalPosts;
  std::vector<GoalSideSpot> goalSideSpots;
  std::vector<Geometry::Line> goalPostLinesFinal;
  std::vector<ColorSegment> colorSegments;

  int imageWidth, imageHeight;
  RingBufferWithSum<int, 5> yBuffer;
  RingBufferWithSum<int, 5> cbBuffer;
  RingBufferWithSum<int, 5> crBuffer;

private:
  void update(CLIPGoalPercept& theCLIPGoalPercept);

  // called once, processes both images
  void execute(const bool& upper, CLIPGoalPercept& theCLIPGoalPercept);

  // run some scan lines around horizon to find y-jumps
  void scanForGoalSegments(const bool& upper);

  void runSegmentScanLine(const int& yPos, const bool& upper);
  void runSegmentScanLineGauss(const int& yPos, const bool& upper);

  void connectGoalSpots(const bool& upper);

  void createLinesFromSpots(const bool& upper);

  void createGoalPostsFromLines(const bool& upper);

  /*
  * Verify or falsify goal post candidates.
  */
  void verifyGoalPosts(CLIPGoalPercept& theCLIPGoalPercept, const bool& upper);

  /*
  * Use all info from scanned posts from upper and lower image to merge info.
  * TODO: Also use lines percept for more precise goal posts.
  */
  void mergeGoalPostInfo(CLIPGoalPercept& theCLIPGoalPercept);


  /*** functions to verify possible goal posts ***/

  /*
  * Scan for the bottom (on the field) of the possible goal post
  * Also look for field lines verifying gp
  * and verify color of gp while scanning
  * @return True, if the field was found.
  */
  bool scanForGoalPostBottom(GoalPost& gp, const bool& upper);

  // same with top
  bool scanForGoalPostTop(GoalPost& gp, const bool& upper);

  // what it says, uses checkForGoalPostColor(..)
  CLIPGoalPercept::GoalPost::GoalPostSide scanForGoalPostSide(const GoalPost& gp, const bool& upper);

  /*
  * Scan on different heights to check precise width (pixels) of goal post.
  * @return True, if enough of those scans return a fitting width.
  */
  bool scanForGoalPostWidth(GoalPost& gp, const int yStep, const bool& upper);

  // pixel precise width scan from around startpoint until expected width is reached
  // color values of gp are collected
  float scanForGoalPostWidthAt(const Vector2i& startPoint, const float& expectedWidth, int& realStartX, int& avgCb, int& avgCr, int& avgY, const bool& upper);

  /*
  * Check if distance by size of possible goal post is sane.
  * @param useWorldModel True, if goal post should fit into the world model
  * @return True, if the field was found.
  */
  bool verifyBySize(GoalPost& gp, const bool useWorldModel, const bool& upper);

  /*
  * Scan until color differs too much from avgCb/avgCr
  * @return length of segment in same color
  */
  int scanSameColor(const Vector2f& startPoint, const Vector2f& direction, const int& stepSize, int& avgCb, int& avgCr, int& avgY, const bool& upper);

  // checks below basePoint for field color
  bool scanForGreen(const Vector2f& basePoint, const bool& upper);

  // counts pixels with similar values to optCb/optCr, used for side determination
  int checkForGoalPostColor(const Vector2i& startPoint, const Vector2i& direction, const int& width, const int& optCb, const int& optCr, const int& optY, const bool& upper);

  // sobel
  inline float getAngle(const int xPos, const int yPos, const int length, const Image& image)
  {
    const int length2 = length / 2;
    const int x1 = xPos - length;
    const int x2 = xPos - length2;
    const int y1 = yPos - length2;
    const int y3 = yPos + length2;
    if (image.isOutOfImage(xPos, yPos, length))
      return 0.f;
    int yA1 = image[y1][x1].y;
    int yA2 = image[y1][x2].y;
    int yA3 = image[y1][xPos].y;
    int yB1 = image[yPos][x1].y;
    //int yB2 = image.image[yPos][x2].y;
    int yB3 = yBuffer[0];
    int yC1 = image[y3][x1].y;
    int yC2 = image[y3][x2].y;
    int yC3 = image[y3][xPos].y;
    int sumX = yA1;
    int sumY = yA1;
    sumY += 2 * yA2;
    sumY += yA3;
    sumX -= yA3;
    sumX += 2 * yB1;
    sumX -= 2 * yB3;
    sumX += yC1;
    sumY -= yC1;
    sumY -= 2 * yC2;
    sumX -= yC3;
    sumY -= yC3;
    Vector2f direction((float)sumY, (float)-sumX);
    ARROW("module:CLIPGoalPerceptor:goalSpots", xPos - length2, yPos, xPos - length2 + direction.normalize(15).x(), yPos + direction.normalize(15).y(), 1, Drawings::solidPen, ColorRGBA::red);
    return direction.angle();
  }
  // debugging stuff

  void drawGoalPostCandidates();
  inline int getDiffSum(const RingBufferWithSum<int, 5>& buf, int sum)
  {
    for (int i = 1; i < 5; i++)
    {
      int diff = buf[i] - buf[i - 1];
      if (sgn(sum) == sgn(diff) && diff > 10) // TODO: param
        sum += diff;
    }
    return sum;
  }

  inline int getGauss(const RingBufferWithSum<int, 5>& buf) { return -yBuffer[0] - 2 * yBuffer[1] + 2 * yBuffer[3] + yBuffer[4]; }
};
