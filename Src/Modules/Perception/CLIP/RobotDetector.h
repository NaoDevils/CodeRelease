/** 
* @file RobotDetector.h
* uses obstacle percepts from image processing and the obstacle model filled with sonar data
* and generates robot percepts.
* Currently only vision based, since sonar is not working yet.
* @author <a href="mailto:ingmar.schwarz@uni-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include <algorithm>

MODULE(RobotDetector,
{ ,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CLIPGoalPercept),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FrameInfo),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  USES(RobotPose),
  REQUIRES(ObstacleBasePoints),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TeammateData),
  PROVIDES(RobotsPercept),
  LOADS_PARAMETERS(
  {,
    (float) maxFootWidthDiff,
    (float) robotHeadWidth,
    (float) robotFootWidth, //average value of width of robot at feet
    (float) robotHeight,
    (float) waistBandHeight,
    (float) minDistToGoalPostField,
    (float) maxObstacleDistField,
    (bool) useSonar,
    (bool) useSonarWithoutVision,
    (bool) useBumpers,
    (bool) useTeamMatePoses,
    (bool) useGreenCheck,
    (bool) useWhiteCheck,
    (int) maxCrDiff, // max sum of diffs to fieldCr mainly to rule out goals as obstacles
    (int) shirtMinColorDiff,
    (float) featureFactor,
    (Vector2i) ownColor,
    (Vector2i) oppColor,
  }),
});

class RobotDetector : public RobotDetectorBase
{
public:
  RobotDetector();
private:

  struct ShirtSpot
  {
    int xStart,xEnd,y;
    int avgCb,avgCr;
    bool blue;
  };

  RobotsPercept localRobotsPercept;
  RingBufferWithSum<int,8> fieldColorBuffer;
  int imageWidth, imageHeight;
  int avgY, avgCb, avgCr;
  int pixelCount;

  bool scanForRobotWidth(Vector2f &basePoint, float &robotWidth, const bool &upper, const bool &imageEndOkLeft, const bool &imageEndOkRight);
  int scanForFeatures(const Vector2i &basePoint, const Vector2i &scanDir, const Vector2i &stopAt, const bool &upper);
  bool findEndOfObstacle(const Vector2f &basePoint, const Vector2f &scanDir, float &width, const bool &upper, const bool &imageEndOk);
  bool lookForOtherLeg(Vector2f &basePoint, float &robotWidth, const float &expectedWidth, const bool &upper);
  bool verifyObstacle(Vector2f &footBase, float &robotWidth, RobotEstimate &robot, const bool &upper);
  
  bool checkForGreen(const float &x, const float &y, Vector2f scanDir, int maxSteps, int minGreen, const bool &upper);

  /* Check leg form of possible percept.
  * @param footBase The lower left point of the obstacle.
  * @param robotWidth The scanned width of the legs.
  * @param upper True, if found in upper image.
  * @return True, if leg outlines are found.
  */
  bool checkForm(const Vector2f &footBase, const float &robotWidth, const bool &upper);
  bool checkYDiffs(const Vector2f &footBase, const float &robotWidth, const bool &upper);
  int checkForYGradient(const Vector2i &from, const Vector2i &dir, const bool &upper);

  RobotEstimate::RobotType scanForRobotColor(
    const Vector2f &from,
    const Vector2f &scanDir,
    const int &maxSteps,
    Vector2f wristPos,
    const bool &upper);

  void update(RobotsPercept &theRobotsPercept);
  
  void execute();
  void checkLowBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle);
  void checkHighBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle);
  void checkLeftBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle);
  void checkRightBasePoint(const ObstacleBasePoints::ObstacleBasePoint &obstacle);
  void addObstaclesFromGoodSegments();
  void addObstaclesFromTeamMateData();
  void addObstaclesFromColorScan();
  void calcImageCoords(RobotEstimate &robot, bool upper);

  // for detecting obstacles with foot bumpers
  //float minBufferAvgForObstacle;
  RingBufferWithSum<float, 10> pressBuffer;
  RingBufferWithSum<float, 600> buttonCheckFilter[4];
  void addObstacleFromBumpers();
};
