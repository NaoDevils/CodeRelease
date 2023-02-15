#include "BallPerceptTools.h"

#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Modules/Perception/CNNs/CLIPBallPerceptorCNNs.h"

std::vector<std::string> BallPerceptTools::ballCNNsTFlite = {"ball_position_v10_bs4096_19933.tflite"};

std::vector<t_cnn_fp> BallPerceptTools::cnns = {cnn_qball::cnn};

bool BallPerceptTools::applyBallRadiusFromCameraMatrix(BallSpot& ballSpot, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const float& ballRadius)
{
  Vector2f posOnField;
  Geometry::Circle expectedCircle;
  if (Transformation::imageToRobotHorizontalPlane(ballSpot.position.cast<float>(), ballRadius, cameraMatrix, cameraInfo, posOnField)
      && Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, ballRadius, expectedCircle))
  {
    ballSpot.radiusInImage = expectedCircle.radius;
    return true;
  }
  else
  {
    return false;
  }
}

bool BallPerceptTools::verifyAndGetBallPositionOnField(
    const BallSpot& ballSpot, const bool upper, Vector2f& posOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions, bool useRobotPose, const Pose2f& robotPose)
{
  if (!Transformation::imageToRobotHorizontalPlane(ballSpot.position.cast<float>(), fieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField))
    return false;

  Vector2f ballFieldCoords = Transformation::robotToField(robotPose, posOnField);

  if (useRobotPose)
  {
    if (!fieldDimensions.isInsideCarpet(ballFieldCoords))
      return false;
    if (posOnField.norm() < 1000 && (ballFieldCoords.x() < fieldDimensions.xPosOwnGroundline - 200 || ballFieldCoords.x() > fieldDimensions.xPosOpponentGroundline + 200))
      return false;
  }

  return true;
}

void BallPerceptTools::fillBallPercept(
    const BallSpot& spot, const bool upper, unsigned timestamp, const Vector2f posOnField, const BallPatch::DetectionSource source, const BallPatch::DetectionVerifier verifier, BallPercept& localBallPercept)
{
  localBallPercept.status = BallPercept::seen;
  localBallPercept.timestamp = timestamp;
  localBallPercept.positionInImage = spot.position.cast<float>();
  localBallPercept.radiusInImage = spot.radiusInImage;
  localBallPercept.relativePositionOnField = posOnField;
  localBallPercept.validity = spot.validity;
  localBallPercept.fromUpper = upper;
  localBallPercept.detectionSource = source;
  localBallPercept.detectionVerifier = verifier;
  localBallPercept.ballPatch.radiusInPatch = spot.radiusInImage / localBallPercept.ballPatch.resizeFactor;
  localBallPercept.ballPatch.validity = spot.validity;
  localBallPercept.ballPatch.fromUpper = upper;
}

bool BallPerceptTools::checkBallCNNWithPositionTflite(
    tflite::Interpreter& interpreter, BallSpot& spot, const bool upper, const BallPatch::DetectionSource source, const Image& image, float ballCNNWithPositionZoomOutFactor, float ballCNNWithPositionThreshold, BallPatch& ballPatch)
{
  float width = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), spot.radiusInImage * 2.f * ballCNNWithPositionZoomOutFactor));
  float height = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), spot.radiusInImage * 2.f * ballCNNWithPositionZoomOutFactor));

  float xmax = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), spot.position.x() + (width / 2.f)));
  float ymax = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), spot.position.y() + (height / 2.f)));

  float xmin = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), xmax - width));
  float ymin = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), ymax - height));

  float* input = interpreter.typed_input_tensor<float>(0);
  image.copyAndResizeAreaRGBFloat(
      static_cast<int>(xmin + 0.5f), static_cast<int>(ymin + 0.5f), static_cast<int>(width + 0.5f), static_cast<int>(height + 0.5f), CNN_POSITION_SIZE, CNN_POSITION_SIZE, input);

  if (interpreter.Invoke() != kTfLiteOk)
  {
    OUTPUT_ERROR("Failed to invoke tflite!");
  }

  float* result = interpreter.typed_output_tensor<float>(0);

  spot.validity = 1 / (1 + std::exp(-result[0]));

  Vector2f ballCenter;
  ballCenter.x() = 1 / (1 + std::exp(-result[1]));
  ballCenter.y() = 1 / (1 + std::exp(-result[2]));

  ballCenter.x() *= std::max<float>(width, height); // TODO check if neccessary
  ballCenter.y() *= std::max<float>(width, height);

  spot.position.x() = static_cast<int>(xmin + 0.5f + ballCenter.x());
  spot.position.y() = static_cast<int>(ymin + 0.5f + ballCenter.y());

  float resize_factor = std::max<float>(width, height) / CNN_POSITION_SIZE;
  std::vector<float> input_vector{input, input + CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3};
  ballPatch.patch = input_vector;
  ballPatch.centerInPatch.x() = ballCenter.x() / resize_factor;
  ballPatch.centerInPatch.y() = ballCenter.y() / resize_factor;
  ballPatch.radiusInPatch = spot.radiusInImage / resize_factor;
  ballPatch.resizeFactor = resize_factor;
  ballPatch.validity = spot.validity;
  ballPatch.fromUpper = upper;
  ballPatch.source = source;
  ballPatch.verifier = BallPatch::DetectionVerifier::ballPositionCNN;

  if (spot.validity >= ballCNNWithPositionThreshold)
  {
    if (upper)
    {
      RECTANGLE("module:CLIPBallPerceptor:ballPosition:Upper", xmin, ymin, xmax, ymax, 3, Drawings::solidPen, ColorRGBA::black);
      CROSS("module:CLIPBallPerceptor:ballPosition:Upper", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
      DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Upper", xmin + 3, ymax, 15, ColorRGBA::black, spot.validity * 100.f);
    }
    else
    {
      RECTANGLE("module:CLIPBallPerceptor:ballPosition:Lower", xmin, ymin, xmax, ymax, 3, Drawings::solidPen, ColorRGBA::black);
      CROSS("module:CLIPBallPerceptor:ballPosition:Lower", spot.position.x(), spot.position.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
      DRAWTEXT("module:CLIPBallPerceptor:ballPosition:Lower", xmin + 3, ymax, 15, ColorRGBA::black, spot.validity * 100.f);
    }
    return true;
  }
  else
  {
    return false;
  }
};

bool BallPerceptTools::checkScanlinesAndCNN(
    BallSpot& spot, const bool upper, const BallPatch::DetectionSource source, const Image& image, std::vector<float>& scanlinesCNNInputVector, const float minConfidenceForSpot, BallPatch& ballPatch, int variante)
{
  float width = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), spot.radiusInImage * 2.f));
  float height = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), spot.radiusInImage * 2.f));

  float xmax = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), spot.position.x() + (width / 2.f)));
  float ymax = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), spot.position.y() + (height / 2.f)));

  float xmin = std::max<float>(0.f, std::min<float>(static_cast<float>(image.width - 1), xmax - width));
  float ymin = std::max<float>(0.f, std::min<float>(static_cast<float>(image.height - 1), ymax - height));

  int cnnResult = -1;

  image.copyAndResizeAreaFloat(
      static_cast<int>(xmin + 0.5f), static_cast<int>(ymin + 0.5f), static_cast<int>(width + 0.5f), static_cast<int>(height + 0.5f), CNN_SCANLINES_SIZE, CNN_SCANLINES_SIZE, &scanlinesCNNInputVector[0]);

  float scores[2];
  cnns.at(variante)(scanlinesCNNInputVector.data(), scores);

  cnnResult = scores[1] > scores[0] ? 1 : 0;
  if (cnnResult == 1)
    spot.validity = std::min<float>(1.f / (1.f + expf(-scores[1])), 1.f);
  else
    spot.validity = std::min<float>(1.f / (1.f + expf(-scores[0])), 1.f) / 100.f; // Scale to <1% for No Ball

  float resize_factor = std::max<float>(width, height) / CNN_SCANLINES_SIZE;
  ballPatch.patch = scanlinesCNNInputVector;
  ballPatch.centerInPatch.x() = (spot.position.x() - xmin) / resize_factor;
  ballPatch.centerInPatch.y() = (spot.position.y() - ymin) / resize_factor;
  ballPatch.radiusInPatch = spot.radiusInImage / resize_factor;
  ballPatch.resizeFactor = resize_factor;
  ballPatch.validity = spot.validity;
  ballPatch.fromUpper = upper;
  ballPatch.source = source;
  ballPatch.verifier = BallPatch::DetectionVerifier::scanlinesAndCNN;

  if (cnnResult == 1 && spot.validity < minConfidenceForSpot)
    cnnResult = 0;

  if (cnnResult == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
};
