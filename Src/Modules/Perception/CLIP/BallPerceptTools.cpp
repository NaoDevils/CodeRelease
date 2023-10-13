#include "BallPerceptTools.h"

#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Modules/Perception/CNNs/CLIPBallPerceptorCNNs.h"

std::vector<t_cnn_fp> BallPerceptTools::cnns = {cnn_qball::cnn};

bool BallPerceptTools::applyBallRadiusFromCameraMatrix(BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const float& ballRadius)
{
  Vector2f posOnField;
  Geometry::Circle expectedCircle;
  if (Transformation::imageToRobotHorizontalPlane(ballSpot.position.cast<float>(), ballRadius, cameraMatrix, cameraInfo, posOnField)
      && Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, ballRadius, expectedCircle) && expectedCircle.radius > 1.f)
  {
    ballSpot.radiusInImage = expectedCircle.radius;
    return true;
  }
  else
  {
    return false;
  }
}

std::optional<Vector2f> BallPerceptTools::verifyAndGetBallPositionOnField(
    const BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions, bool useRobotPose, const Pose2f& robotPose)
{
  Vector2f posOnField;
  if (!Transformation::imageToRobotHorizontalPlane(ballSpot.position.cast<float>(), fieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField))
    return {};

  Vector2f ballFieldCoords = Transformation::robotToField(robotPose, posOnField);

  if (useRobotPose)
  {
    if (!fieldDimensions.isInsideCarpet(ballFieldCoords))
      return {};

    // A&A TODO: Does it make sense to filter here?
    if (posOnField.norm() < 1000 && (ballFieldCoords.x() < fieldDimensions.xPosOwnGroundline - 200 || ballFieldCoords.x() > fieldDimensions.xPosOpponentGroundline + 200))
      return {};
  }

  return posOnField;
}

std::tuple<bool, BallPatch> BallPerceptTools::checkBallCNNWithPositionTflite(
    tflite::Interpreter& interpreter, CheckedBallSpot& spot, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThreshold)
{
  const auto [min, max, size] = image.projectIntoImage(spot.position, Vector2i::Constant(static_cast<int>(spot.radiusInImage * 2.f * ballCNNWithPositionZoomOutFactor + 0.5f)));

  unsigned char* input = interpreter.typed_input_tensor<unsigned char>(0);
  image.copyAndResizeArea<true, false>(min, size, {CNN_POSITION_SIZE, CNN_POSITION_SIZE}, input);

  bool returnValue = false;
  BallPatch ballPatch;
  STOPWATCH_WITH_PLOT("BallCNNPositionTFlite:Invoke")
  {
    if (interpreter.Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
    }

    float* result = interpreter.typed_output_tensor<float>(0);
    spot.validity = 1 / (1 + std::exp(-result[0]));

    Vector2f ballCenter;
    ballCenter.x() = 1 / (1 + std::exp(-result[1]));
    ballCenter.y() = 1 / (1 + std::exp(-result[2]));

    ballCenter *= static_cast<float>(size.maxCoeff()); // TODO check if neccessary

    spot.position = (min.cast<float>() + Vector2f::Constant(0.5f) + ballCenter).cast<int>();
    spot.verifier = CheckedBallSpot::DetectionVerifier::ballPositionCNN;

    ballPatch.setPatch(image, min, size, {CNN_POSITION_SIZE, CNN_POSITION_SIZE});
    ballPatch.fromBallSpot(spot);

    if (spot.validity >= ballCNNWithPositionThreshold)
    {
      if (spot.upper)
      {
        RECTANGLE("module:CLIPBallPerceptor:ballPosition:Upper", min.x(), min.y(), max.x(), max.y(), 3, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:CLIPBallPerceptor:ballPosition:Upper", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
        DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Upper", min.x() + 3, max.y(), 15, ColorRGBA::black, spot.validity * 100.f);
      }
      else
      {
        RECTANGLE("module:CLIPBallPerceptor:ballPosition:Lower", min.x(), min.y(), max.x(), max.y(), 3, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:CLIPBallPerceptor:ballPosition:Lower", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
        DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Lower", min.x() + 3, max.y(), 15, ColorRGBA::black, spot.validity * 100.f);
      }
      returnValue = true;
    }
    else
    {
      returnValue = false;
    }
  }
  return {returnValue, ballPatch};
};

std::tuple<bool, BallPatch> BallPerceptTools::checkBallCNNWithPositionEarlyExitTflite(
    const std::vector<TfliteInterpreter>& layers, CheckedBallSpot& spot, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThresholdEarlyExit, float ballCNNWithPositionThreshold)
{
  const auto [min, max, size] = image.projectIntoImage(spot.position, Vector2i::Constant(static_cast<int>(spot.radiusInImage * 2.f * ballCNNWithPositionZoomOutFactor + 0.5f)));
  const Vector2i oldCenter = spot.position;

  auto& firstInterpreter = layers.at(0).getInterpreter();
  unsigned char* input = firstInterpreter.typed_input_tensor<unsigned char>(0);
  image.copyAndResizeArea<true, false>(min, size, {CNN_POSITION_SIZE, CNN_POSITION_SIZE}, input);

  bool returnValue = false;
  BallPatch ballPatch;
  STOPWATCH_WITH_PLOT("BallCNNPositionTFliteEarlyExit:Invoke")
  {
    if (firstInterpreter.Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
    }

    float* early_exit_result = firstInterpreter.typed_output_tensor<float>(1);
    float early_exit_validity = 1 / (1 + std::exp(-early_exit_result[0]));

    Vector2f ballCenter;
    if (early_exit_validity >= ballCNNWithPositionThresholdEarlyExit)
    {
      auto& secondInterpreter = layers.at(1).getInterpreter();
      if (secondInterpreter.Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
      }

      float* result = secondInterpreter.typed_output_tensor<float>(0);
      spot.validity = 1 / (1 + std::exp(-result[0]));
      ballCenter.x() = 1 / (1 + std::exp(-result[1]));
      ballCenter.y() = 1 / (1 + std::exp(-result[2]));
    }
    else
    {
      //spot.validity = early_exit_validity * (ballCNNWithPositionThreshold / ballCNNWithPositionThresholdEarlyExit);
      spot.validity = early_exit_validity / 100.f;
      ballCenter.x() = 0.5;
      ballCenter.y() = 0.5;
    }

    ballCenter *= static_cast<float>(size.maxCoeff());

    spot.position = (min.cast<float>() + Vector2f::Constant(0.5f) + ballCenter).cast<int>();
    spot.verifier = CheckedBallSpot::DetectionVerifier::ballPositionCNN;

    ballPatch.setPatch(image, min, size, {CNN_POSITION_SIZE, CNN_POSITION_SIZE});
    ballPatch.fromBallSpot(spot);

    if (early_exit_validity >= ballCNNWithPositionThresholdEarlyExit)
    {
      if (spot.upper)
      {
        ARROW("module:CLIPBallPerceptor:ballPosition:Upper", oldCenter.x(), oldCenter.y(), spot.position.x(), spot.position.y(), 1, Drawings::solidPen, ColorRGBA::green);
      }
      else
      {
        ARROW("module:CLIPBallPerceptor:ballPosition:Lower", oldCenter.x(), oldCenter.y(), spot.position.x(), spot.position.y(), 1, Drawings::solidPen, ColorRGBA::green);
      }
    }

    if (spot.validity >= ballCNNWithPositionThreshold)
    {
      if (spot.upper)
      {
        RECTANGLE("module:CLIPBallPerceptor:ballPosition:Upper", min.x(), min.y(), max.x(), max.y(), 3, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:CLIPBallPerceptor:ballPosition:Upper", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::black);
        DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Upper", min.x() + 3, max.y(), 15, ColorRGBA::black, spot.validity * 100.f);
      }
      else
      {
        RECTANGLE("module:CLIPBallPerceptor:ballPosition:Lower", min.x(), min.y(), max.x(), max.y(), 3, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:CLIPBallPerceptor:ballPosition:Lower", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::black);
        DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Lower", min.x() + 3, max.y(), 15, ColorRGBA::black, spot.validity * 100.f);
      }
      returnValue = true;
    }
    else
    {
      returnValue = false;
    }
  }
  return {returnValue, ballPatch};
};

std::tuple<bool, BallPatch> BallPerceptTools::checkScanlinesAndCNN(CheckedBallSpot& spot, const Image& image, const float minConfidenceForSpot, const int variant, const float ballCNNWithPositionZoomOutFactor)
{
  std::tuple<bool, BallPatch> ret(false, {});
  auto& [detected, ballPatch] = ret;

  const auto [min, max, size] = image.projectIntoImage(spot.position, Vector2i::Constant(static_cast<int>(spot.radiusInImage * 2.f + 0.5f)));

  std::array<float, CNN_SCANLINES_SIZE * CNN_SCANLINES_SIZE> scanlinesCNNInputVector{0.f};
  image.copyAndResizeArea<false, false>(min, size, {CNN_SCANLINES_SIZE, CNN_SCANLINES_SIZE}, scanlinesCNNInputVector.data());

  float scores[2];
  cnns.at(variant)(scanlinesCNNInputVector.data(), scores);

  detected = scores[1] > scores[0];
  if (detected)
    spot.validity = std::min<float>(1.f / (1.f + expf(-scores[1])), 1.f);
  else
    spot.validity = std::min<float>(1.f / (1.f + expf(-scores[0])), 1.f) / 100.f; // Scale to <1% for No Ball
  spot.verifier = CheckedBallSpot::DetectionVerifier::scanlinesAndCNN;

  // Always write yolo patches with CNN_POSITION_SIZE x CNN_POSITION_SIZE
  const auto [yoloMin, yoloMax, yoloSize] = image.projectIntoImage(spot.position, Vector2i::Constant(static_cast<int>(spot.radiusInImage * 2.f * ballCNNWithPositionZoomOutFactor + 0.5f)));
  ballPatch.setPatch(image, yoloMin, yoloSize, {CNN_POSITION_SIZE, CNN_POSITION_SIZE});
  ballPatch.fromBallSpot(spot);

  detected = detected && spot.validity >= minConfidenceForSpot;

  return ret;
};
