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
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/TfliteInterpreter.h"
#include <algorithm>
#include "stdint.h"

MODULE(CLIPBallPerceptor,
  REQUIRES(BodyContour),
  REQUIRES(BodyContourUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraIntrinsics),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(BallSpots),
  REQUIRES(BallHypothesesYolo),
  REQUIRES(CLIPFieldLinesPercept),
  USES(RobotPose),
  REQUIRES(MotionInfo),
  REQUIRES(BallPerceptTfliteInterpreter),
  USES(BallModel),
  USES(BallPercept),
  PROVIDES(BallPercept),
  REQUIRES(MultipleBallPercept),
  PROVIDES(MultipleBallPercept),
  PROVIDES(ProcessedBallPatches),
  LOADS_PARAMETERS(,
    (bool) addExtraScan, // add additional scan lines if ball not found
    (int) numberOfScanLines, // Number of scan lines within possible ball.
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
    (bool) useRobotPose, // If true, ball percepts that are seen outside of the field area are neglected.
    (bool) allowBallObstacleOverlap, // If true, ball percept overlapping with obstacles (goal,robots) will be accepted. Dangerous!
    (float) minConfidence, // min confidence of cnn on lower image
    (float) minConfidenceUpper, // min confidence of cnn on upper image
    (float) minConfidenceReduced, // min confidence of cnn on lower image
    (float) minConfidenceUpperReduced, // min confidence of cnn on upper image
    (bool) useBallConfidence, // If false, validity of ball percept is always 1
    (int) cnnIndex, // Use the CNN with this index
    (bool) logPositives,
    (float)(0.50f) ballCNNWithPositionThreshold,
    (float)(2.f) ballCNNWithPositionZoomOutFactor
  )
);

class CLIPBallPerceptor : public CLIPBallPerceptorBase
{
public:
  /**
  * Default constructor.
  */
  CLIPBallPerceptor();

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
    none,
    edgy,
    normal,
    good
  );

  struct BallPerceptState
  {
    void reset()
    {
      overlapAreas.clear();
      featureCheckNeeded = false;
      detailedCheckNeeded = false;
      circleOK = false;
      hullState = HullCheckState::none;
      ballObstacleOverlap = false;
      ballOnFieldLine = false;
      ballScannedOnce = false;
      ballOnField = Vector2f::Zero();
      validity = 0;
    }
    std::vector<OverlapArea> overlapAreas;
    bool featureCheckNeeded;
    bool detailedCheckNeeded;
    bool circleOK;
    HullCheckState hullState;
    bool ballObstacleOverlap;
    // for possible ball on field line - do not use scan lines ending on field line
    bool ballOnFieldLine;
    bool ballScannedOnce;
    int yHistogram[32];
    Vector2f ballOnField;
    float validity;
  };

  struct BallHullPoint
  {
    Vector2f pointInImage;
    int directionID;
  };

  Geometry::Line lineUpperBorder; //upper left
  Geometry::Line lineLowerBorder; //lower right

  float minDistOfCenterFromImageBorder;

  int imageWidth;
  int imageHeight;

  std::string logDir;

  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;

  std::vector<BallHullPoint> ballHullPoints; /**< all ball edge points, if scanline is not too long */
  std::vector<BallHullPoint> goodBallHullPoints; /**< ball edge points to field */
  std::vector<Vector2f> ballPoints;
  std::vector<Vector2f> goodBallPoints;
  Vector2f scannedCenter; /**< For comparing the model matching ball center with the ball center found via image scan. */
  float scannedRadius;
  BallPerceptState ballPerceptState;

  BallPercept localBallPercept;
  MultipleBallPercept localMultipleBallPercept;
  std::vector<BallPatch> processedBallPatches;
  BallSpots localBallSpots;
  std::vector<BallSpot> checkedBallSpots;

  std::array<int, 100> featureHistogram;
  std::array<int, 100> distanceHistogram;

  size_t noOfTestCircles;
  size_t noOfTestCirclesUpper;
  size_t noOfProcessedSpots = 0;
  size_t sumOfBallSpots = 0;

private:
  void update(BallPercept& theBallPercept);
  void update(MultipleBallPercept& theMultipleBallPercept);
  void update(ProcessedBallPatches& theProcessedBallPatches);

  void reset();
  void executeDeclares();

  /*
  * Iterates through ball spots and tries to create ball percepts.
  * Validity of BallPercept is defined by comparing theoretical size in
  * image (calculated by distance on field) to scanned size.
  */
  void execute(const bool& upper, bool multi = false);

  // creates ball hull points from ball spot if constraints are fulfilled
  // call three times
  // 1st only 4 directions to find center (centerFound, found = false)
  // 2nd again only 4 directions to verify center and first check for ball/obstacle overlap (found = false)
  // 3rd detailed scan for ball hull points, verify hull and ball/obstacle overlap
  bool calcBallSpot2016(BallSpot& spot, const bool upper, const bool multi, const BallPatch::DetectionSource source);

  // called in 3rd call of calcBallSpot2016
  bool verifyBallHull(BallSpot& spot, const bool upper, const bool multi, const BallPatch::DetectionSource source);

  bool verifyBallSizeAndPosition(const Vector2f& posInImage, const float radius, const bool upper, const bool multi);

  /*
  * Calculates number of fitting points on a circle.
  *
  * return Maximum number of connected fitting points
  */
  int getFittingPoints(const Geometry::Circle& baseCircle, float& distSum, float& maxDist, const bool& upper);

  // used to compute ballpercept only once from multiple ball spots on one ball
  bool isOnCheckedBallSpot(const BallSpot& spot, const bool upper);
  bool isOnMultipleBallPercept(const BallSpot& spot, const bool upper);

  // extra scan lines if ball was not found
  void additionalBallSpotScan();
  void runBallSpotScanLine(int fromX, int toX, int stepX, int stepY, int steps, const bool& upper);

  inline bool isPixelBallColor(unsigned char y, unsigned char cb, unsigned char cr, const FieldColors& fieldColor)
  {
    return (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::orange && cr > 150 && cb < 150)
        || (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && (cr > 95 && cb > 95 && cr < 160 && cb < 160))
        || (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any);
  }

  bool applyBallRadiusFromCameraMatrix(BallSpot& ballSpot, const bool upper);
  bool verifyAndGetBallPositionOnField(const BallSpot& ballSpot, const bool upper, Vector2f& posOnField);
  void fillBallPercept(const BallSpot& spot, const bool upper, const Vector2f posOnField, const BallPatch::DetectionSource source, const BallPatch::DetectionVerifier verifier);
  bool checkBallSpot(BallSpot& spot, const bool upper, const bool multi, const BallPatch::DetectionSource source, const BallPatch::DetectionVerifier verifier);
  bool checkWithVerifier(BallSpot& spot, Vector2f& posOnField, const bool upper, const bool multi, const BallPatch::DetectionSource source, const BallPatch::DetectionVerifier verifier);

  std::vector<float> ballCNNPositionInputVector;
  std::vector<float> scanlinesCNNInputVector;
  bool checkBallCNNWithPosition(BallSpot& spot, const bool upper, const BallPatch::DetectionSource source);
  bool checkScanlinesAndCNN(BallSpot& spot, const bool upper, const BallPatch::DetectionSource source, const float minConfidenceForSpot);
};
