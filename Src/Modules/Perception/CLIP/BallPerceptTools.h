/**
* @file BallPerceptTools.h
* Declaration of class BallPerceptTools.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

#pragma once

#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/TfliteInterpreter.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <cstdio>
#include <optional>
#include <tuple>
#include "Modules/Perception/TFlite.h"

typedef void (*t_cnn_fp)(float* in_0, float* out_0);

class BallPerceptTools
{
public:
  /**
  * Default constructor.
  */
  BallPerceptTools(){};

  static std::vector<t_cnn_fp> cnns;

  static bool applyBallRadiusFromCameraMatrix(BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const float& ballRadius);
  static std::optional<Vector2f> verifyAndGetBallPositionOnField(
      const BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions, bool useRobotPose, const Pose2f& robotPose);
  static std::tuple<bool, BallPatch> checkBallCNNWithPositionTflite(
      tflite::Interpreter& interpreter, CheckedBallSpot& spot, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThreshold);
  static std::tuple<bool, BallPatch> checkBallCNNWithPositionEarlyExitTflite(
      const std::vector<TfliteInterpreter>& layers, CheckedBallSpot& spot, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThresholdEarlyExit, float ballCNNWithPositionThreshold);
  static std::tuple<bool, BallPatch> checkScanlinesAndCNN(CheckedBallSpot& spot, const Image& image, const float minConfidenceForSpot, const int variant, const float ballCNNWithPositionZoomOutFactor);
};
