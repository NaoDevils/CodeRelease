/**
* @file BallPerceptTools.h
* Declaration of class BallPerceptTools.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

#pragma once

#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <cstdio>
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
  static std::vector<std::string> ballCNNsTFlite;

  static bool applyBallRadiusFromCameraMatrix(BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const float& ballRadius);
  static bool verifyAndGetBallPositionOnField(
      const BallSpot& ballSpot, const bool upper, Vector2f& posOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions, bool useRobotPose, const Pose2f& robotPose);
  static void fillBallPercept(
      const BallSpot& spot, const bool upper, unsigned timestamp, const Vector2f posOnField, const BallPatch::DetectionSource source, const BallPatch::DetectionVerifier verifier, BallPercept& localBallPercept);
  static bool checkBallCNNWithPositionTflite(
      tflite::Interpreter& interpreter, BallSpot& spot, const bool upper, const BallPatch::DetectionSource source, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThreshold, BallPatch& ballPatch);
  static bool checkScanlinesAndCNN(
      BallSpot& spot, const bool upper, const BallPatch::DetectionSource source, const Image& image, std::vector<float>& ballCNNPositionInputVector, const float minConfidenceForSpot, BallPatch& ballPatch, int variante = 0);
};
