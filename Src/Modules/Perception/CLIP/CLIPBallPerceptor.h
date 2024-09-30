/**
* @file CLIPBallPerceptor.h
* Declaration of class CLIPBallPerceptor.
* Scans for ball given ball segments from imageprocessing.
* If ball is not found, additional scans are made (using the ball model here).
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/TfliteInterpreter.h"
#include <optional>
#include "stdint.h"

STREAMABLE(MatlabCNN,,
    (float)(0.81f) minConfidence, // min confidence of cnn on lower image
    (float)(0.75f) minConfidenceUpper, // min confidence of cnn on upper image
    (float)(0.73f) minConfidenceReduced, // min confidence of cnn on lower image
    (float)(0.71) minConfidenceUpperReduced, // min confidence of cnn on upper image
    (bool)(true) useBallConfidence, // If false, validity of ball percept is always 1
    (int)(0) cnnIndex // Use the CNN with this index
);

STREAMABLE(TFLiteCNN,,
    (float)(0.25f) ballCNNWithPositionThreshold, // min confidence of cnn
    (float)(0.5f) ballCNNWithPositionThresholdEarlyExit, // min confidence of early exit
    (float)(0.01f) minConfidenceForSecondCheck,
    (float)(1.5f) ballCNNWithPositionZoomOutFactor
);

MODULE(CLIPBallPerceptor,
  REQUIRES(BodyContour),
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
  REQUIRES(ScanlinesBallSpots),
  REQUIRES(BallHypothesesYolo),
  REQUIRES(BallHypothesesSegmentor),
  REQUIRES(CLIPFieldLinesPercept),
  USES(RobotPose),
  REQUIRES(BallPerceptTfliteInterpreter),
  REQUIRES(SplittedTfliteInterpreter),
  USES(BallModel),
  USES(BallPercept),
  PROVIDES(BallPercept),
  REQUIRES(MultipleBallPercept),
  PROVIDES(MultipleBallPercept),
  PROVIDES(ProcessedBallPatches),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    (bool) addExtraScan, // add additional scan lines if ball not found
    (int) numberOfScanLines, // Number of scan lines within possible ball.
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
    (int)(50) maxNumberOfHypotheses,
    (bool)(false) useEarlyExit,
    (MatlabCNN) matlabCNN,
    (TFLiteCNN) tfliteCNN
  )
);

class CLIPBallPerceptor : public CLIPBallPerceptorBase
{
  void execute(tf::Subflow& subflow);
  void update(BallPercept& theBallPercept);
  void update(MultipleBallPercept& theMultipleBallPercept);
  void update(ProcessedBallPatches& theProcessedBallPatches);

  void reset();
  void executeDeclares() const;


  bool addBallSpot(std::vector<CheckedBallSpot>& spots, const BallSpot& spot, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const;
  bool addBallSpots(std::vector<CheckedBallSpot>& spots, const std::vector<BallSpot>& newSpots, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const;
  bool addBallSpots(std::vector<CheckedBallSpot>& spots, const std::vector<ScanlinesBallSpot>& newSpots, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const;
  bool duplicateBallSpots(std::vector<CheckedBallSpot>& spots, size_t begin, size_t end, CheckedBallSpot::DetectionVerifier verifier) const;

  inline bool isPixelBallColor(unsigned char y, unsigned char cb, unsigned char cr, const FieldColors& fieldColor) const
  {
    return (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::orange && cr > 150 && cb < 150)
        || (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack && (cr > 95 && cb > 95 && cr < 160 && cb < 160))
        || (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any);
  }

  bool applyBallRadiusFromCameraMatrix(BallSpot& ballSpot) const;
  std::optional<Vector2f> verifyAndGetBallPositionOnField(const BallSpot& ballSpot) const;
  [[nodiscard]] std::tuple<bool, std::optional<BallPatch>> checkBallSpot(CheckedBallSpot& spot) const;
  [[nodiscard]] std::tuple<bool, std::optional<BallPatch>> checkWithVerifier(CheckedBallSpot& spot) const;

  [[nodiscard]] std::tuple<bool, BallPatch> checkBallCNNWithPosition(CheckedBallSpot& spot) const;
  [[nodiscard]] std::tuple<bool, std::optional<BallPatch>> checkScanlinesAndCNN(CheckedBallSpot& spot, const float minConfidenceForSpot) const;

  BallPercept localBallPercept;
  MultipleBallPercept localMultipleBallPercept;
  std::vector<std::vector<BallPatch>> localProcessedBallPatches;

  bool enableMultipleBallPercept = false;
  bool enableProcessedBallPatches = false;

  // LEGACY STUFF BELOW

  struct BallHullPoint
  {
    Vector2f pointInImage = Vector2f::Zero();
    int directionID = 0;
  };

  // extra scan lines if ball was not found
  [[nodiscard]] std::vector<ScanlinesBallSpot> additionalBallSpotScan() const;
  [[nodiscard]] std::vector<ScanlinesBallSpot> runBallSpotScanLine(int fromX, int toX, int stepX, int stepY, int steps, const bool upper) const;

  [[nodiscard]] std::tuple<bool, float> verifyBallSizeAndPosition(const Vector2f& posInImage, const float radius, const bool upper) const;

  // called in 3rd call of calcBallSpot2016
  [[nodiscard]] std::tuple<bool, std::optional<BallPatch>> verifyBallHull(
      CheckedBallSpot& spot, const std::vector<BallHullPoint>& ballHullPoints, const std::vector<BallHullPoint>& goodBallHullPoints, const Vector2f& scannedCenter) const;

  // creates ball hull points from ball spot if constraints are fulfilled
  [[nodiscard]] std::tuple<bool, std::optional<BallPatch>> calcBallSpot2016(CheckedBallSpot& spot) const;
};
