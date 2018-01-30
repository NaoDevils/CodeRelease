/**
* @file CLIPBallPerceptor.h
* Declaration of class CLIPBallPerceptor.
* Scans for ball given ball segments from imageprocessing.
* If ball is not found, additional scans are made (using the ball model here).
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/IntegralImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"                                                     
#include <algorithm>
#include "stdint.h"

MODULE(CLIPBallPerceptor,
{,
  REQUIRES(BodyContour),
  REQUIRES(BodyContourUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(IntegralImage),
  REQUIRES(BallSpots),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(RobotsPercept),
  USES(RobotPose),
  USES(MotionInfo),
  USES(BallModel),
  PROVIDES(BallHypotheses),
  PROVIDES(BallPercept),
  PROVIDES(MultipleBallPercept),
  LOADS_PARAMETERS(
  {,
    (bool) addExtraScan, // add additional scan lines if ball not found
    (int) numberOfScanLines, // Number of scan lines within possible ball.
    (bool) useCNN, // if true, CNN is used on every 'testCircle' (after ball center is found)
    (bool) useCNNOnly, // if true, all other checks after CNN are omitted (except basic size sanity checks)
    (bool) logTestCircles, // log hypotheses for CNN, does nothing if useCNN == false
    (int) minFittingPoints, // Minimum fitting points on calculated circle on possible ball to be accepted as ball percept.
    (int) minFittingPointsForSafeBall, // If this number is fitting, rest does not matter
    (int) maxFarPointsOnHull, // How many of the scan lines within ball are allowed to end outside of calculated circle?
    (int) minDistFromImageBorder, // take a guess
    (int) maxColorDiff, // Max color diff to avg ball color (coming from ball spot).
    (int) maxColorJumpDiff, // Max color diff while scanning on ball.
    (int) minNumberOfYJumps, // Min number of jumps on y channel (for black/white ball).
    (float) validityFactor, // Higher factor = less verification of ball size through cameramatrix.
    (float) minValidity, // Minimum validity for a secure ball spot (further ball spots are neglected!).
    (float) lowestValidity, // Minimum validity for a ball spot to become a ball percept.
    (float) lowerImageUpperBorderDistanceFactor, // Factor*radius min distance from upper image border in lower image (goal post along with wrong cameramatrix..)
    (float) minRadiusInImage, // minimal radius in image to accept the ball
    (float) minRadiusInImageForFeatures, // for features on ball, below this only accept ballpercepts lying free on the field
    (float) radiusForNoFeatureCheck, // below this radius, balls can be accepted without features on ball if hull check ok
    (bool) useFeaturesForGoodLargeBalls, // if active, feature check is needed for larger balls even if hullState is good
    (float) expectedBallFeatureSize, // compared to radius, i.e. 0.5 means diameter is half as big as radius
    (float) maxBallFeatureSizeDeviation, // max difference to expected size
    (float) expectedBallFeatureDistance, // compared to radius, i.e. 0.5 means distance is half as big as radius
    (float) maxBallFeatureDistanceDeviation, // max difference to expected distance
    (int) maxWrongColorDiv, // for feature scan -> if wrong color count on possible is greater than ball pixel count divided by this, reject
    (bool) useRobotPose, // If true, ball percepts that are seen outside of the field area are neglected.
    (bool) allowBallObstacleOverlap, // If true, ball percept overlapping with obstacles (goal,robots) will be accepted. Dangerous!
    (int) maxBallFeatureScore, // Max percent of black pixels on ball
    (int) minBallFeatureScore, // Min percent of black pixels on ball
    (int) maxBallWhiteScore, // Max percent of pixels counted as white on ball without black in between
    (int) minCornerResponses, // Minimum number of positive responses to 'darker corner' check on II
    (int) minWBResponses, // Minimum number of gradient responses on II
    (float) minScore, // Minimum number of gradient responses on II
    (bool) logPositives,
    (bool) useBallValidity, // If false, validity of ball percept is always 1
  }),
});

class CLIPBallPerceptor: public CLIPBallPerceptorBase
{
public:
  /**
  * Default constructor.
  */
  CLIPBallPerceptor();

  DECLARE_DEBUG_IMAGE(BallHypothesesLower);
  DECLARE_DEBUG_IMAGE(BallHypothesesUpper);

  struct BallFeature
  {
    int yAvg;
    int minX, maxX, minY, maxY;
    float scannedSizeX;
    float scannedSizeY;
    Vector2f center;
  };

  struct OverlapArea
  {
    int startID;
    int endID;
  };

  ENUM(HullCheckState,
  { ,
    none,
    edgy,
    normal,
    good,
  });

  struct BallPerceptState
  {
    void reset()
    {
      ballFeatures.clear();
      overlapAreas.clear();
      featureCheckNeeded = false;
      detailedCheckNeeded = false;
      circleOK = false;
      hullState = HullCheckState::none;
      ballObstacleOverlap = false;
      ballOnFieldLine = false;
      ballScannedOnce = false;
      integralImageResponse = -1;
      cnnCheck = false;
      ballOnField = Vector2f::Zero();
      validity = 0;
    }
    std::vector<BallFeature> ballFeatures;
    std::vector<OverlapArea> overlapAreas;
    bool featureCheckNeeded;
    bool detailedCheckNeeded;
    bool circleOK;
    HullCheckState hullState;
    bool ballObstacleOverlap;
    // for possible ball on field line - do not use scan lines ending on field line
    bool ballOnFieldLine;
    bool ballScannedOnce;
    int integralImageResponse;
    int yHistogram[32];
    bool cnnCheck;
    Vector2f ballOnField;
    float validity;
  };

  struct BallHullPoint
  {
    BallHullPoint operator=(const BallHullPoint &other)
    {
      pointInImage = other.pointInImage;
      directionID = other.directionID;
      return *this;
    }
    Vector2f pointInImage;
    int directionID;
  };

  Geometry::Line lineUpperBorder; //upper left
  Geometry::Line lineLowerBorder; //lower right

  float minDistOfCenterFromImageBorder;

  int imageWidth;
  int imageHeight;

  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;

  //for cnn
  std::vector<unsigned char> ballHypothesis;
  std::vector<unsigned char> ballHypothesisLog;

  std::vector< BallHullPoint > ballHullPoints; /**< all ball edge points, if scanline is not too long */
  std::vector< BallHullPoint > goodBallHullPoints; /**< ball edge points to field */
  std::vector< Vector2f > ballPoints;
  std::vector< Vector2f > goodBallPoints;
  Vector2f scannedCenter; /**< For comparing the model matching ball center with the ball center found via image scan. */
  float scannedRadius;
  BallPerceptState ballPerceptState;

  BallPercept localBallPercept;
  MultipleBallPercept localMultipleBallPercept;
  BallSpots localBallSpots;
  std::vector<BallSpot> noBallSpots;

  std::array < int, 100 > featureHistogram;
  std::array < int, 100 > distanceHistogram;

private:
  void update(BallPercept &theBallPercept);
  void update(MultipleBallPercept &theMultipleBallPercept);
  void update(BallHypotheses &theBallHypotheses);

  /*
  * Iterates through ball spots and tries to create ball percepts.
  * Validity of BallPercept is defined by comparing theoretical size in
  * image (calculated by distance on field) to scanned size.
  */
  void execute(const bool &upper, bool multi = false);

  // creates ball hull points from ball spot if constraints are fulfilled
  // call three times 
  // 1st only 4 directions to find center (centerFound, found = false)
  // 2nd again only 4 directions to verify center and first check for ball/obstacle overlap (found = false)
  // 3rd detailed scan for ball hull points, verify hull and ball/obstacle overlap
  bool calcBallSpot2016(BallSpot &spot, const bool &upper);

  // called in 3rd call of calcBallSpot2016
  bool verifyBallHull(BallSpot &spot, const bool &upper);

  // verify ball spot and fill ball percept
  bool verifyBallPercept(BallSpot &spot, const bool &upper);

  bool verifyBallSizeAndPosition(const Vector2f &posInImage, const float &radius, const bool &upper);

  // counts y jumps on ball (for white black ball)
  // TODO: make more reliable (using distance, distribution?)
  bool checkYJumps(BallSpot &spot, const bool &upper);

  // check features 
  bool checkFeatures(const BallSpot &spot, const bool &upper);

  // feature check using integral image, replaces 'checkFeatures' method
  bool checkFeaturesWithIntegralImage(const BallSpot &spot, const bool &upper);

  // check feature distribution on ball
  bool checkFeatureDistribution(const BallSpot &spot, const bool &upper);

  bool scanForOverlap(const BallSpot &spot, const bool &upper);

  bool checkOverlap(const BallSpot &spot, const bool &upper);

  bool scanFeature(BallFeature &feature, const Vector2f &center, const float &expectedSize, const bool &upper);

  /*
  * Calculates number of fitting points on a circle.
  *
  * return Maximum number of connected fitting points
  */
  int getFittingPoints(const Geometry::Circle &baseCircle,
    float &distSum, float &maxDist, const bool &upper);

  // used to compute ballpercept only once from multiple ball spots on one ball
  bool isOnBallSpot(const Vector2i &pos, const bool &upper);

  // extra scan lines if ball was not found
  void additionalBallSpotScan();
  void runBallSpotScanLine(int fromX, int toX, int stepX, int stepY, int steps, const bool &upper);

  // there should be no green on ball spot
  int countGreenOnBallSpot(BallSpot &spot, const bool &upper);

  inline bool isPixelBallColor(unsigned char y, unsigned char cb, unsigned char cr, const FieldColors &fieldColor)
  {
    return (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::orange && cr > 150 && cb < 150) ||
      (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && (cr > 95 && cb > 95 && cr < 160 && cb < 160)) ||
      (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any);
  }

  float getBallDiameterAt(const float &x, const float &y, const Image &image, const CameraMatrix &cameraMatrix, const CameraInfo &cameraInfo)
  {
    // theoretical diameter if cameramatrix is correct
    Vector2f posOnField;
    if (!Transformation::imageToRobotHorizontalPlane(
      Vector2f(x, y),
      theFieldDimensions.ballRadius,
      cameraMatrix,
      cameraInfo,
      posOnField))
      return 100;
    Geometry::Circle expectedCircle;
    if (!Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle))
      return 100;
    return expectedCircle.radius * 2;

  }

};

