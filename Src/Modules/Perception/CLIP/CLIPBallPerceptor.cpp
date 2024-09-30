#include "CLIPBallPerceptor.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Platform/File.h"
#include "Modules/Perception/CNNs/CLIPBallPerceptorCNNs.h"
#include "Modules/Perception/CLIP/BallPerceptTools.h"
#include <filesystem>
#include <algorithm>
#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/find.hpp>
#include <taskflow/algorithm/transform.hpp>

void CLIPBallPerceptor::reset()
{
  localBallPercept.positionInImage = Vector2f::Zero();
  localBallPercept.relativePositionOnField = Vector2f::Zero();
  localBallPercept.status = BallPercept::notSeen;
  localBallPercept.validity = 0.f;
  localBallPercept.timestamp = 0;
  localBallPercept.ballPatch = BallPatch();
  localMultipleBallPercept.balls.clear();
  for (auto& processedBallPatchesPerThread : localProcessedBallPatches)
    processedBallPatchesPerThread.clear();
}

void CLIPBallPerceptor::executeDeclares() const
{
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:additionalScan", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballScannedCenter", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballValidity", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:yJumpScanLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:fittingPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:testCircles:upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballPosition:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballPosition:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballPatch", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballSpots:upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CLIPBallPerceptor:ballSpots:lower", "drawingOnImage");

  DECLARE_PLOT("module:CLIPBallPerceptor:noOfTestCirclesUpper");
  DECLARE_PLOT("module:CLIPBallPerceptor:noOfTestCircles");
  DECLARE_PLOT("module:CLIPBallPerceptor:ballSpots:upper:sum");
  DECLARE_PLOT("module:CLIPBallPerceptor:ballSpots:upper:processed");
  DECLARE_PLOT("module:CLIPBallPerceptor:ballSpots:lower:sum");
  DECLARE_PLOT("module:CLIPBallPerceptor:ballSpots:lower:processed");
  DECLARE_PLOT("module:CLIPBallPerceptor:sumOfBallSpots");
  DECLARE_PLOT("module:CLIPBallPerceptor:processedBallSpots");
}

void CLIPBallPerceptor::execute(tf::Subflow& subflow)
{
  executeDeclares();
  reset();

  std::vector<CheckedBallSpot> ballSpots;
  ballSpots.reserve(maxNumberOfHypotheses);

  if (enableProcessedBallPatches)
  {
    localProcessedBallPatches.resize(subflow.executor().num_workers());
    for (auto& patches : localProcessedBallPatches)
      patches.reserve(maxNumberOfHypotheses);
  }

  STOPWATCH("CLIPBallPerceptor:addBallSpots")
  {
    const auto addLastPercept = [&](const Image& image)
    {
      const bool upper = (&image == &theImageUpper);

      if (theBallPercept.fromUpper == upper)
      {
        const BallSpot spot(static_cast<int>(std::round(theBallPercept.positionInImage.x())), static_cast<int>(std::round(theBallPercept.positionInImage.y())), upper);
        return addBallSpot(ballSpots, spot, CheckedBallSpot::DetectionSource::ballModel, CheckedBallSpot::DetectionVerifier::ballPositionCNN);
      }
      return true;
    };

    const auto addBallModel = [&](const Image& image, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo)
    {
      const bool upper = (&image == &theImageUpper);

      const auto getBallSpot = [&](const Vector2f& ballRelative) -> std::optional<BallSpot>
      {
        Vector2f pImage = Vector2f::Zero();
        const Vector3f bP(ballRelative.x(), ballRelative.y(), theFieldDimensions.ballRadius);
        if (Transformation::robotToImage(bP, cameraMatrix, cameraInfo, pImage) && !image.isOutOfImage(pImage.x(), pImage.y(), 4))
          return BallSpot(static_cast<int>(std::round(pImage.x())), static_cast<int>(std::round(pImage.y())), upper);
        else
          return {};
      };

      if (const auto ballSpot = getBallSpot(BallPhysics::propagateBallPosition(theBallModel.estimate.position, theBallModel.estimate.velocity, 30.f / 1000.f, theBallModel.friction)))
        if (!addBallSpot(ballSpots, *ballSpot, CheckedBallSpot::DetectionSource::ballModel, CheckedBallSpot::DetectionVerifier::ballPositionCNN))
          return false;

      if (const auto ballSpot = getBallSpot(theBallModel.estimate.position))
        if (!addBallSpot(ballSpots, *ballSpot, CheckedBallSpot::DetectionSource::ballModel, CheckedBallSpot::DetectionVerifier::ballPositionCNN))
          return false;

      if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 750)
        if (const auto ballSpot = getBallSpot(theBallModel.lastPerception))
          return addBallSpot(ballSpots, *ballSpot, CheckedBallSpot::DetectionSource::ballModel, CheckedBallSpot::DetectionVerifier::ballPositionCNN);

      return true;
    };

    // clang-format off
    const bool cont = addLastPercept(theImage)
        && addLastPercept(theImageUpper)
        && addBallModel(theImage, theCameraMatrix, theCameraInfo)
        && addBallModel(theImageUpper, theCameraMatrixUpper, theCameraInfoUpper)
        && addBallSpots(ballSpots, theBallHypothesesYolo.ballSpots, CheckedBallSpot::DetectionSource::yoloHypothesis, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
        && addBallSpots(ballSpots, theBallHypothesesSegmentor.ballSpots, CheckedBallSpot::DetectionSource::segmentorHypothesis, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
        && addBallSpots(ballSpots, theScanlinesBallSpots.ballSpots, CheckedBallSpot::DetectionSource::scanlines, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
        && addBallSpots(ballSpots, theBallHypothesesYolo.ballSpotsUpper, CheckedBallSpot::DetectionSource::yoloHypothesis, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
        && addBallSpots(ballSpots, theScanlinesBallSpots.ballSpotsUpper, CheckedBallSpot::DetectionSource::scanlines, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
        && duplicateBallSpots(ballSpots, 0, ballSpots.size(), CheckedBallSpot::DetectionVerifier::scanlinesAndCNN);
    // clang-format on

    if (cont && addExtraScan)
    {
      const size_t begin = ballSpots.size();
      addBallSpots(ballSpots, additionalBallSpotScan(), CheckedBallSpot::DetectionSource::scanlines, CheckedBallSpot::DetectionVerifier::ballPositionCNN)
          && duplicateBallSpots(ballSpots, begin, ballSpots.size(), CheckedBallSpot::DetectionVerifier::scanlinesAndCNN);
    }
  }

  const auto isBallPercept = [&](const CheckedBallSpot& bs)
  {
    CheckedBallSpot spot(bs);

    Global::getTimingManager().startTiming("CLIPBallPerceptor:checkBallSpot");
    const auto [found, ballPatch] = checkBallSpot(spot);
    Global::getTimingManager().stopTiming("CLIPBallPerceptor:checkBallSpot");

    if (enableProcessedBallPatches && ballPatch)
      localProcessedBallPatches[subflow.executor().this_worker_id()].emplace_back(*ballPatch);

    return found;
  };

  const auto getBallPercept = [&](const CheckedBallSpot& bs)
  {
    CheckedBallSpot spot(bs);
    const auto [found, ballPatch] = checkBallSpot(spot);
    ASSERT(found);

    const std::optional<Vector2f> posOnField = verifyAndGetBallPositionOnField(spot);
    ASSERT(posOnField.has_value());

    ASSERT(ballPatch.has_value());
    BallPercept ballPercept(spot, theFrameInfo.time, *posOnField);
    ballPercept.ballPatch = *ballPatch;
    return ballPercept;
  };

  if (enableMultipleBallPercept)
  {
    std::vector<bool> results(ballSpots.size(), false);
    tf::Task transformTask = subflow.transform(ballSpots.begin(), ballSpots.end(), results.begin(), isBallPercept).name("TansformBallPercept [CLIPBallPerceptor]");

    const auto get = [&]()
    {
      for (auto it = results.begin(); it != results.end(); ++it)
      {
        if (*it)
        {
          const CheckedBallSpot& ballSpot = ballSpots[std::distance(results.begin(), it)];
          localMultipleBallPercept.balls.emplace_back() = getBallPercept(ballSpot);
        }
      }

      if (!localMultipleBallPercept.balls.empty())
        static_cast<BallPercept&>(localBallPercept) = localMultipleBallPercept.balls.front();
    };
    tf::Task getTask = subflow.emplace(get).name("GetBallPercept [CLIPBallPerceptor]");
    transformTask.precede(getTask);

    subflow.join();
  }
  else
  {
    std::vector<CheckedBallSpot>::iterator result;
    tf::Task findTask = subflow.find_if(ballSpots.begin(), ballSpots.end(), result, isBallPercept, tf::DynamicPartitioner()).name("FindBallPercept [CLIPBallPerceptor]");

    const auto get = [&]()
    {
      if (result != ballSpots.end())
        static_cast<BallPercept&>(localBallPercept) = getBallPercept(*result);
    };
    tf::Task getTask = subflow.emplace(get).name("GetBallPercept [CLIPBallPerceptor]");
    findTask.precede(getTask);

    subflow.join();
  }

  enableProcessedBallPatches = false;
  enableMultipleBallPercept = false;
}

void CLIPBallPerceptor::update(BallPercept& ballPercept)
{
  ballPercept = localBallPercept;
}

void CLIPBallPerceptor::update(MultipleBallPercept& multipleBallPercept)
{
  auto sorting_criteria = [](const auto& a, const auto& b)
  {
    if (a.validity == b.validity)
      return a.positionInImage.y() > b.positionInImage.y();
    return a.validity > b.validity;
  };
  std::sort(localMultipleBallPercept.balls.begin(), localMultipleBallPercept.balls.end(), sorting_criteria);

  multipleBallPercept = localMultipleBallPercept;
  enableMultipleBallPercept = true;
}

void CLIPBallPerceptor::update(ProcessedBallPatches& processedBallPatches)
{
  const auto addSize = [](size_t size, const auto& v)
  {
    return size + v.size();
  };
  const size_t count = std::accumulate(localProcessedBallPatches.begin(), localProcessedBallPatches.end(), size_t(0), addSize);

  processedBallPatches.patches.clear();
  processedBallPatches.patches.reserve(count);
  for (const auto& patches : localProcessedBallPatches)
    processedBallPatches.patches.insert(processedBallPatches.patches.end(), patches.begin(), patches.end());

  enableProcessedBallPatches = true;
}

bool CLIPBallPerceptor::applyBallRadiusFromCameraMatrix(BallSpot& ballSpot) const
{
  const CameraMatrix& cameraMatrix = ballSpot.upper ? static_cast<const CameraMatrix&>(theCameraMatrixUpper) : theCameraMatrix;
  const CameraInfo& cameraInfo = ballSpot.upper ? static_cast<const CameraInfo&>(theCameraInfoUpper) : theCameraInfo;

  return BallPerceptTools::applyBallRadiusFromCameraMatrix(ballSpot, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius);
}

std::optional<Vector2f> CLIPBallPerceptor::verifyAndGetBallPositionOnField(const BallSpot& ballSpot) const
{
  const CameraMatrix& cameraMatrix = ballSpot.upper ? static_cast<const CameraMatrix&>(theCameraMatrixUpper) : theCameraMatrix;
  const CameraInfo& cameraInfo = ballSpot.upper ? static_cast<const CameraInfo&>(theCameraInfoUpper) : theCameraInfo;

  return BallPerceptTools::verifyAndGetBallPositionOnField(ballSpot, cameraMatrix, cameraInfo, theFieldDimensions, useRobotPose, theRobotPose);
}

std::tuple<bool, std::optional<BallPatch>> CLIPBallPerceptor::checkBallSpot(CheckedBallSpot& spot) const
{
  ColorRGBA brushColor;
  switch (spot.source)
  {
  case CheckedBallSpot::DetectionSource::scanlines:
    brushColor = ColorRGBA::red;
    break;
  case CheckedBallSpot::DetectionSource::yoloHypothesis:
    brushColor = ColorRGBA::blue;
    break;
  case CheckedBallSpot::DetectionSource::ballModel:
    brushColor = ColorRGBA::violet;
    break;
  case CheckedBallSpot::DetectionSource::segmentorHypothesis:
    brushColor = ColorRGBA::yellow;
    break;
  }

  Vector2i originalSpotPosition(spot.position);

  std::tuple<bool, std::optional<BallPatch>> ret = checkWithVerifier(spot);
  auto& [fill, ballpatch] = ret;

  DEBUG_DRAWING("module:CLIPBallPerceptor:ballSpots:upper", "drawingOnImage")
  {
    int textsize = static_cast<int>(spot.radiusInImage / 2.f);
    if (spot.upper)
    {
      CIRCLE("module:CLIPBallPerceptor:ballSpots:upper", originalSpotPosition.x(), originalSpotPosition.y(), spot.radiusInImage, 2, Drawings::solidPen, brushColor, Drawings::noBrush, brushColor);
      if (spot.validity > 0.01f)
        DRAWTEXT("module:CLIPBallPerceptor:ballSpots:upper", originalSpotPosition.x() - textsize / 2, originalSpotPosition.y() + textsize / 2, textsize, brushColor, roundf(spot.validity * 100));
    }
  }
  DEBUG_DRAWING("module:CLIPBallPerceptor:ballSpots:lower", "drawingOnImage")
  {
    int textsize = static_cast<int>(spot.radiusInImage / 2.f);
    if (!spot.upper)
    {
      CIRCLE("module:CLIPBallPerceptor:ballSpots:lower", originalSpotPosition.x(), originalSpotPosition.y(), spot.radiusInImage, 2, Drawings::solidPen, brushColor, Drawings::noBrush, brushColor);
      if (spot.validity > 0.01f)
        DRAWTEXT("module:CLIPBallPerceptor:ballSpots:lower", originalSpotPosition.x() - textsize / 2, originalSpotPosition.y() + textsize / 2, textsize, brushColor, roundf(spot.validity * 100));
    }
  }

  if (!fill && spot.verifier == CheckedBallSpot::DetectionVerifier::ballPositionCNN && spot.validity > tfliteCNN.minConfidenceForSecondCheck) // TODO
  {
    spot.verifier = CheckedBallSpot::DetectionVerifier::scanlinesAndCNN;

    float minConfidenceForSpot = 0.f;
    if (spot.upper)
    {
      minConfidenceForSpot = matlabCNN.minConfidenceUpperReduced;
      //if (spot.radiusInImage < 15)
      //  minConfidenceForSpot -= ((15 - spot.radiusInImage) / 75.f);
      //minConfidenceForSpot = std::max(0.5f, minConfidenceForSpot);
    }
    else
    {
      minConfidenceForSpot = matlabCNN.minConfidenceReduced;
    }

    float savedValidity = std::min(0.1f, spot.validity);
    minConfidenceForSpot -= savedValidity;

    ret = checkScanlinesAndCNN(spot, minConfidenceForSpot);
    if (fill)
      spot.validity += savedValidity;
    else
      return ret;

    if (!applyBallRadiusFromCameraMatrix(spot))
    {
      fill = false;
      return ret;
    }
  }

  if (!verifyAndGetBallPositionOnField(spot))
    fill = false;

  return ret;
}

std::tuple<bool, std::optional<BallPatch>> CLIPBallPerceptor::checkWithVerifier(CheckedBallSpot& spot) const
{
  std::tuple<bool, std::optional<BallPatch>> ret = {false, {}};
  auto& [fill, ballPatch] = ret;

  switch (spot.verifier)
  {
  case CheckedBallSpot::DetectionVerifier::ballPositionCNN:
    ret = checkBallCNNWithPosition(spot);
    fill = fill && applyBallRadiusFromCameraMatrix(spot);
    break;
  case CheckedBallSpot::DetectionVerifier::scanlinesAndCNN:
    std::tie(fill, ballPatch) = calcBallSpot2016(spot);
    break;
  default:
    ASSERT(false);
  }

  return ret;
}

bool CLIPBallPerceptor::addBallSpot(std::vector<CheckedBallSpot>& spots, const BallSpot& spot, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const
{
  CheckedBallSpot& checkedSpot = spots.emplace_back(spot, source, verifier);

  if (!(checkedSpot.position.x() == 0 && checkedSpot.position.y() == 0) && (applyBallRadiusFromCameraMatrix(checkedSpot)))
  {
    const auto isContained = [&](const CheckedBallSpot& bs)
    {
      const float distance = (bs.position - checkedSpot.position).cast<float>().norm();
      return bs.upper == checkedSpot.upper && bs.verifier == checkedSpot.verifier
          && (distance < std::max<float>(bs.radiusInImage, checkedSpot.radiusInImage) * tfliteCNN.ballCNNWithPositionZoomOutFactor);
    };

    if (std::find_if(spots.begin(), spots.end() - 1, isContained) == spots.end() - 1)
      return spots.size() < spots.capacity();
  }
  spots.pop_back();

  return spots.size() < spots.capacity();
}

bool CLIPBallPerceptor::addBallSpots(std::vector<CheckedBallSpot>& spots, const std::vector<BallSpot>& newSpots, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const
{
  for (const auto& newSpot : newSpots)
    if (!addBallSpot(spots, newSpot, source, verifier))
      return false;
  return true;
}

bool CLIPBallPerceptor::addBallSpots(
    std::vector<CheckedBallSpot>& spots, const std::vector<ScanlinesBallSpot>& newSpots, CheckedBallSpot::DetectionSource source, CheckedBallSpot::DetectionVerifier verifier) const
{
  for (const auto& newSpot : newSpots)
    if (!addBallSpot(spots, newSpot, source, verifier))
      return false;
  return true;
}

bool CLIPBallPerceptor::duplicateBallSpots(std::vector<CheckedBallSpot>& spots, size_t begin, size_t end, CheckedBallSpot::DetectionVerifier verifier) const
{
  // Compare size to capacity => do not allow reallocations
  for (; begin < end; ++begin)
  {
    spots.emplace_back(spots[begin]).verifier = verifier;

    if (spots.size() >= spots.capacity())
      return false;
  }

  return true;
}


std::tuple<bool, BallPatch> CLIPBallPerceptor::checkBallCNNWithPosition(CheckedBallSpot& spot) const
{
  const Image& image = spot.upper ? (Image&)theImageUpper : theImage;

  std::tuple<bool, BallPatch> ret;

  if (useEarlyExit)
  {
    STOPWATCH_WITH_PLOT("CLIPBallPerceptor:TFliteEarlyExit")
    {
      ret = BallPerceptTools::checkBallCNNWithPositionEarlyExitTflite(
          theSplittedTfliteInterpreter.layers, spot, image, tfliteCNN.ballCNNWithPositionZoomOutFactor, tfliteCNN.ballCNNWithPositionThresholdEarlyExit, tfliteCNN.ballCNNWithPositionThreshold);
    }
  }
  else
  {
    STOPWATCH_WITH_PLOT("CLIPBallPerceptor:TFlite")
    {
      ret = BallPerceptTools::checkBallCNNWithPositionTflite(
          theBallPerceptTfliteInterpreter.getInterpreter(), spot, image, tfliteCNN.ballCNNWithPositionZoomOutFactor, tfliteCNN.ballCNNWithPositionThreshold);
    }
  }

  return ret;
};

std::tuple<bool, std::optional<BallPatch>> CLIPBallPerceptor::checkScanlinesAndCNN(CheckedBallSpot& spot, const float minConfidenceForSpot) const
{
  std::tuple<bool, std::optional<BallPatch>> ret = {false, {}};
  if (spot.radiusInImage > (spot.upper ? 30 : 15))
  {
    spot.validity = 0.f;
    return ret;
  }


  const Image& image = spot.upper ? (Image&)theImageUpper : theImage;
  auto& [fill, ballPatch] = ret;

  STOPWATCH("CLIPBallPerceptor:CNN")
  {
    std::tie(fill, ballPatch) = BallPerceptTools::checkScanlinesAndCNN(spot, image, minConfidenceForSpot, matlabCNN.cnnIndex, tfliteCNN.ballCNNWithPositionZoomOutFactor);
  }
  DRAWTEXT("module:CLIPBallPerceptor:testCircles:lower", spot.position.x(), spot.position.y(), 15, ColorRGBA::yellow, "Validity:" << spot.validity * 100.f);
  DRAWTEXT("module:CLIPBallPerceptor:testCircles:upper", spot.position.x(), spot.position.y(), 15, ColorRGBA::yellow, "Validity:" << spot.validity * 100.f);

  // if cnn response is positive, drawn in blue
  DEBUG_RESPONSE("debug drawing:module:CLIPBallPerceptor:testCircles:lower")
  {
    ColorRGBA testCircleColor = fill ? ColorRGBA(0, 0, 255) : ColorRGBA(0, 255, 0);
    if (!spot.upper)
      CIRCLE("module:CLIPBallPerceptor:testCircles:lower", spot.position.x(), spot.position.y(), spot.radiusInImage, 2, Drawings::solidPen, testCircleColor, Drawings::noBrush, testCircleColor);
  }
  DEBUG_RESPONSE("debug drawing:module:CLIPBallPerceptor:testCircles:upper")
  {
    ColorRGBA testCircleColor = fill ? ColorRGBA(0, 0, 255) : ColorRGBA(0, 255, 0);
    if (spot.upper)
      CIRCLE("module:CLIPBallPerceptor:testCircles:upper", spot.position.x(), spot.position.y(), spot.radiusInImage, 2, Drawings::solidPen, testCircleColor, Drawings::noBrush, testCircleColor);
  }

  if (fill && !matlabCNN.useBallConfidence)
    spot.validity = 1.f;

  return ret;
};

// LEGACY STUFF BELOW

std::vector<ScanlinesBallSpot> CLIPBallPerceptor::additionalBallSpotScan() const
{
  // only upper image for now
  bool upper = true;
  const Image& image = upper ? (Image&)theImageUpper : (Image&)theImage;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : (CameraMatrix&)theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  int maxStepSizeX = image.width / 8;
  int imgXStart = 4 + maxStepSizeX / 4 + random(maxStepSizeX / 2);
  int imgXEnd = std::min(imgXStart + maxStepSizeX * 2, image.width - 4);
  int minImgY = std::max<int>(std::min<int>((int)Geometry::calculateHorizon(cameraMatrix, cameraInfo).base.y(), image.height / 4), 4);
  int imgYStart = image.height - 4;
  int imgYEnd = minImgY;
  Vector2i scanPoint;

  int ballSize = 0;

  std::vector<ScanlinesBallSpot> ballSpots;

  //int timeSinceLastSeen = (int)theFrameInfo.getTimeSince(std::max(theBallModel.timeWhenLastSeen,theBallModel.timeWhenLastSeenByTeamMate));
  Vector2f pImage = Vector2f::Zero();
  if (Transformation::robotToImage(theBallModel.estimate.position, cameraMatrix, cameraInfo, pImage) && !image.isOutOfImage(pImage.x(), pImage.y(), 4))
  {
    ballSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius, theBallModel.estimate.position.norm());
    if (ballSize > 50 || ballSize < 3)
      return ballSpots;
    int scanRadius = std::max(ballSize * 5, 50);
    imgXStart = std::max(4, (int)pImage.x() - scanRadius);
    imgXEnd = std::min(imgXStart + scanRadius * 2, image.width - 4);
    imgYStart = std::min(image.height - 4, (int)pImage.y() + scanRadius);
    imgYEnd = std::max(imgYStart - scanRadius * 2, minImgY);
  }
  else
    return ballSpots;

  if (ballSize <= 0 || imgYStart < imgYEnd || imgXEnd < imgXStart)
    return ballSpots;
  scanPoint.x() = imgXStart;
  scanPoint.y() = imgYStart;
  int xStepSize = std::min(ballSize * 2, std::max((imgXEnd - imgXStart) / 10, 5));
  int yStepSize = std::min(ballSize * 2, std::max((imgYStart - imgYEnd) / 10, 5));
  int imgX = imgXStart;
  int imgY = imgYStart;
  int stepSize = image.height / 240;
  int stepsX = (imgXEnd - imgXStart) / stepSize;
  int stepsY = (imgYStart - imgYEnd) / stepSize;


  for (imgY = imgYStart; imgY > imgYEnd; imgY -= yStepSize)
  {
    LINE("module:CLIPBallPerceptor:additionalScan", imgXStart, imgY, imgXEnd, imgY, 2, Drawings::solidPen, ColorRGBA::orange);
    const std::vector<ScanlinesBallSpot> yBallSpots = runBallSpotScanLine(imgX, imgY, stepSize, 0, stepsX, upper);
    ballSpots.insert(ballSpots.end(), yBallSpots.begin(), yBallSpots.end());
  }

  for (imgX = imgXStart; imgX < imgXEnd; imgX += xStepSize)
  {
    LINE("module:CLIPBallPerceptor:additionalScan", imgX, imgYStart, imgX, imgYEnd, 2, Drawings::solidPen, ColorRGBA::orange);
    const std::vector<ScanlinesBallSpot> xBallSpots = runBallSpotScanLine(imgX, imgYEnd, 0, stepSize, stepsY, upper);
    ballSpots.insert(ballSpots.end(), xBallSpots.begin(), xBallSpots.end());
  }

  return ballSpots;
}

std::vector<ScanlinesBallSpot> CLIPBallPerceptor::runBallSpotScanLine(int x, int y, int stepX, int stepY, int steps, const bool upper) const
{
  std::vector<ScanlinesBallSpot> ballSpots;
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors& fieldColor = upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  //const std::vector<BallSpot> &ballSpots = (upper ? localBallSpots.ballSpotsUpper : localBallSpots.ballSpots);

  int stepNo = 0;
  const int step = std::max(stepX, stepY);
  int expectedBallSize = (int)Geometry::getSizeByDistance(cameraInfo, theFieldDimensions.ballRadius * 2, theBallModel.estimate.position.norm());
  ASSERT(step > 0);
  int lastY = image[y][x].y;

  bool onGreen = false;
  bool onBall = false;
  bool isEdge = false;
  int cb = 0;
  int cr = 0;
  int ballSegmentSize = 0;
  int lastBallSegmentSize = 0;
  while (stepNo < steps && !image.isOutOfImage(x, y, 10))
  {
    Image::Pixel p = image[y][x];
    isEdge = std::abs(p.y - lastY) > maxColorDiff;
    onGreen = fieldColor.isPixelFieldColor(p.y, p.cb, p.cr) || (onGreen && !isEdge);
    onBall = (!onGreen && !onBall && isEdge) || (onBall && !onGreen);
    cb += onBall * p.cb;
    cr += onBall * p.cr;

    lastBallSegmentSize = ballSegmentSize;
    ballSegmentSize = onBall * (ballSegmentSize + 1);
    if (ballSegmentSize < lastBallSegmentSize)
    {
      if (lastBallSegmentSize * step > expectedBallSize / 2 && lastBallSegmentSize * step < expectedBallSize * 2)
      {
        ScanlinesBallSpot& bs = ballSpots.emplace_back();
        bs.position.x() = x - (lastBallSegmentSize * stepX) / (2 * step);
        bs.position.y() = y - (lastBallSegmentSize * stepY) / (2 * step);
        bs.cb = cb / lastBallSegmentSize;
        bs.cr = cr / lastBallSegmentSize;
        bs.upper = upper;
        LINE("module:CLIPBallPerceptor:additionalScan", x, y, x - lastBallSegmentSize * stepX, y - lastBallSegmentSize * stepY, 2, Drawings::solidPen, ColorRGBA::red);
      }
      cb = 0;
      cr = 0;
    }
    x += stepX;
    y += stepY;
    lastY = p.y;
  }

  return ballSpots;
}

std::tuple<bool, float> CLIPBallPerceptor::verifyBallSizeAndPosition(const Vector2f& posInImage, const float radius, const bool upper) const
{
  std::tuple<bool, float> ret = {false, 0.f};
  auto& [fill, confidence] = ret;

  BallSpot spot(posInImage.cast<int>(), upper);
  spot.radiusInImage = radius;
  std::optional<Vector2f> ballOnField = verifyAndGetBallPositionOnField(spot);
  if (!ballOnField)
    return ret;

  const Image& image = upper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = upper ? static_cast<const CameraMatrix&>(theCameraMatrixUpper) : theCameraMatrix;
  const CameraInfo& cameraInfo = upper ? static_cast<const CameraInfo&>(theCameraInfoUpper) : theCameraInfo;
  confidence = 0.f;

  // TODO: check
  // project ball on field into image to verify image radius and get better validity
  Geometry::Circle ballByDistance;
  if (!Geometry::calculateBallInImage(*ballOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, ballByDistance))
    return ret;

  float validity = 1.f - std::abs(std::min<float>(1.f, 1.f - ballByDistance.radius / radius));
  DRAWTEXT("module:CLIPBallPerceptor:ballValidity",
      posInImage.x(),
      posInImage.y() + 1.5 * (posInImage.y() > image.height / 2 ? -radius : radius),
      15,
      ColorRGBA::yellow,
      "validity : " << validity << ";" << ballByDistance.radius << ";" << radius);
  // closer balls need better validity
  float validityFactor = ballByDistance.radius > radius ? 0.015f : 0.003f;
  if (std::abs(ballByDistance.radius - radius) < 2 || ((theFieldDimensions.ballType == SimpleFieldDimensions::BallType::any) && validity > 0.55f))
    validity = std::max<float>(validity, 0.81f);
  /*if ((ballType.ballColor == BallType::any) && image.isOutOfImage(posInImage.x, posInImage.y, radiusInImage * 1.5) &&
  !(!upper && posInImage.y + 3*radiusInImage/4 > image.resolutionHeight))
  return false;*/
  float minValidityForThisBall = std::min(lowestValidity + radius * validityFactor, minValidity);
  if (/*std::abs(posOnField.angle()) > fromDegrees(130) || */ localBallPercept.validity >= validity || minValidityForThisBall > validity)
  {
    CIRCLE("module:CLIPBallPerceptor:ballScanLines", posInImage.x(), posInImage.y(), radius, 2, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    return ret;
  }
  confidence = validity;
  fill = true;
  return ret;
}

std::tuple<bool, std::optional<BallPatch>> CLIPBallPerceptor::verifyBallHull(
    CheckedBallSpot& spot, const std::vector<BallHullPoint>& ballHullPoints, const std::vector<BallHullPoint>& goodBallHullPoints, const Vector2f& scannedCenter) const
{
  std::tuple<bool, std::optional<BallPatch>> ret = {false, {}};
  auto& [fill, ballPatch] = ret;

  // TODO: 'roundness' could/should be better detected when ball is big in image!
  // TODO: check distribution of ball hull points! they have to be equally distributed (maybe in getFittingPoints)!
  // TODO: check all conditions
  // TODO: improve ballObstacleOverlap stuff
  const Image& image = spot.upper ? (Image&)theImageUpper : theImage;
  //const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  //const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  // now find best fitting circle for all ballHullPoints
  //int fittingPoints = 0;
  //float maxDist = 0.f;
  //float distSum = 0;

  // In case of default the maximum radius of the ball is set to a quarter of the image height
  // In case of the goalie specialAction wideStance the maximum radius of the ball is set to half
  int maxRadius = image.height / 4;

  Geometry::Circle testCircle;
  std::vector<Vector2f> ballPoints;
  std::vector<Vector2f> goodBallPoints;
  for (unsigned i = 0; i < goodBallHullPoints.size(); i++)
    goodBallPoints.push_back(goodBallHullPoints[i].pointInImage);
  for (unsigned i = 0; i < ballHullPoints.size(); i++)
    ballPoints.push_back(ballHullPoints[i].pointInImage);
  if (!Geometry::computeCircleOnFieldLevenbergMarquardt(goodBallPoints, testCircle) || testCircle.radius >= maxRadius)
    return ret;

  //if (testCircle.radius < 20)
  //  testCircle.radius *= (1.f + (20-testCircle.radius)/20.f);
  if ((!image.isOutOfImage(testCircle.center.x(), testCircle.center.y(), (int)testCircle.radius))
      && (scannedCenter - testCircle.center).norm() > std::max<float>(testCircle.radius, (float)(image.height / 80)))
    return ret;
  // clip with body contour
  int yClipped = static_cast<int>(testCircle.center.y());
  theBodyContour.clipBottom(static_cast<int>(testCircle.center.x()), yClipped);
  if (yClipped - 2 > testCircle.center.y())
    return ret;

  // check if ball size and position on field makes sense (from old verifyBallPercept)
  float validity = 0.f;
  std::tie(fill, validity) = verifyBallSizeAndPosition(testCircle.center, testCircle.radius, spot.upper);
  if (!fill)
    return ret;
  spot.validity = validity;

  spot.radiusInImage = testCircle.radius;
  spot.position = testCircle.center.cast<int>();

  const bool yoloDetection = (spot.source == CheckedBallSpot::DetectionSource::yoloHypothesis);
  float minConfidenceForSpot = 0.f;
  if (spot.upper)
  {
    minConfidenceForSpot = yoloDetection ? matlabCNN.minConfidenceUpperReduced : matlabCNN.minConfidenceUpper;
    //if (spot.radiusInImage < 15)
    //  minConfidenceForSpot -= ((15 - spot.radiusInImage) / 75.f);
    //minConfidenceForSpot = std::max(0.5f, minConfidenceForSpot);
  }
  else
  {
    minConfidenceForSpot = yoloDetection ? matlabCNN.minConfidenceReduced : matlabCNN.minConfidence;
  }

  ret = checkScanlinesAndCNN(spot, minConfidenceForSpot);
  return ret;
}

std::tuple<bool, std::optional<BallPatch>> CLIPBallPerceptor::calcBallSpot2016(CheckedBallSpot& spot) const
{
  struct BallFeature
  {
    int yAvg = 0;
    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    float scannedSizeX = 0.f;
    float scannedSizeY = 0.f;
    Vector2f center = Vector2f::Zero();
  };

  struct OverlapArea
  {
    int startID = 0;
    int endID = 0;
  };

  enum HullCheckState
  {
    none,
    edgy,
    normal,
    good
  };

  struct BallPerceptState
  {
    std::vector<OverlapArea> overlapAreas;
    bool featureCheckNeeded = false;
    bool detailedCheckNeeded = false;
    bool circleOK = false;
    HullCheckState hullState = HullCheckState::none;
    bool ballObstacleOverlap = false;
    // for possible ball on field line - do not use scan lines ending on field line
    bool ballOnFieldLine = false;
    bool ballScannedOnce = false;
    int yHistogram[32] = {0};
    float validity = false;
  };

  std::vector<BallHullPoint> ballHullPoints; /**< all ball edge points, if scanline is not too long */
  std::vector<BallHullPoint> goodBallHullPoints; /**< ball edge points to field */

  Vector2f scannedCenter = Vector2f::Zero(); /**< For comparing the model matching ball center with the ball center found via image scan. */
  float scannedRadius = 0.f;
  BallPerceptState ballPerceptState;
  Geometry::Line lineUpperBorder; //upper left
  Geometry::Line lineLowerBorder; //lower right

  // Conversion from legacy version of calcBallSpots2016() for parallelization.
  // call three times
  // 1st only 4 directions to find center (centerFound, found = false)
  // 2nd again only 4 directions to verify center and first check for ball/obstacle overlap (found = false)
  // 3rd detailed scan for ball hull points, verify hull and ball/obstacle overlap
  const std::function<std::tuple<bool, std::optional<BallPatch>>(CheckedBallSpot&)> recursiveLambda = [&](CheckedBallSpot& spot)
  {
    std::tuple<bool, std::optional<BallPatch>> ret = {false, {}};
    auto& [ret_fill, ret_ballPatch] = ret;

    // TODO: shorten/split
    // TODO: far ball with obstacle overlap can be discarded? maybe only if tracking a good ball percept..!
    // TODO: too much ball spots are scanned.. check runtime! (improve speed?)
    // TODO: get ball on line, if possible not using field lines percept (center circle!)
    // TODO: remember onShadow state and correct ball position later/ remove those edge points? Problem is getting circle along shadow instead of ball.
    // TODO: how to find center if left/right overlap??
    const Image& image = spot.upper ? (Image&)theImageUpper : theImage;
    const FieldColors& fieldColor = spot.upper ? (FieldColors&)theFieldColorsUpper : theFieldColors;
    const CameraMatrix& cameraMatrix = spot.upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    const CameraInfo& cameraInfo = spot.upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

    if (image.isOutOfImage(spot.position.x(), spot.position.y(), minDistFromImageBorder))
      return ret;

    // theoretical diameter if cameramatrix is correct
    Vector2f posOnField(Vector2f::Zero());
    if (!Transformation::imageToRobotHorizontalPlane(Vector2f(spot.position.cast<float>()), theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField))
      return ret;
    Geometry::Circle expectedCircle;
    if (!Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle))
      return ret;
    float expectedSizeInImage = expectedCircle.radius * 2;
    // how far scan lines will run - can be shortened if center is (approx) right!
    int maxScanSize = spot.found ? static_cast<int>(expectedCircle.radius * 1.5f) : (spot.centerFound ? static_cast<int>(expectedSizeInImage) + 1 : static_cast<int>(expectedSizeInImage * 1.5f));
    if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack && spot.found && (posOnField.norm() < 1000 || posOnField.norm() > 2000))
      maxScanSize = static_cast<int>(expectedCircle.radius * 1.3f);
    maxScanSize = std::max(maxScanSize, static_cast<int>(expectedCircle.radius) + 5);
    // only check for yJumps ON ball (avoids adding useless jumps on edge of ball)
    float yJumpRadius = spot.found ? (expectedSizeInImage / 2 - std::max(2.f, expectedSizeInImage / 12)) : 0;
    int maxGreenCount = std::max(2, std::min(5, static_cast<int>(expectedCircle.radius / 6.f)));

    Vector2i scanPoint(spot.position); // the current point on the scan line
    // for yJump detection
    int yJumpSum = 0;
    int yJumpLine = -1;
    int lastY = -500;
    int newY = 0;
    bool goingUp = false;
    bool first = true;
    // for running scan line with only int operations
    const int increment = 3 * (!spot.found) + 1; // skip 3 scan lines if only scanning for center
    int xInc = increment;
    int yInc = -increment;
    int xDir = 0;
    int yDir = 4;
    float lengthPerStep = 1.f;
    float scanLength = 0.f;
    int maxI = spot.found ? 16 : 4; // TODO: make full scan dynamic according to parameter file
    // helper variables for quality check on scan line
    bool foundGreen = false;
    bool foundBallColor = false;
    int wrongColorCount = 0;
    int greenCount = 0;
    int ballColorCount = 0;
    int pointNo = 0;
    int pointNoSum = 0;
    // for detecting ball hull/center
    ballHullPoints.clear();
    goodBallHullPoints.clear();
    ballPerceptState.overlapAreas.clear();
    Vector2i lastBallColorPos(spot.position);
    scannedCenter = spot.position.cast<float>();
    scannedRadius = 0.f;
    int maxY = 0;
    if (spot.found) // when using all star like scanlines, collect y info for yAvg
    {
      for (int i = 0; i < 32; i++)
        ballPerceptState.yHistogram[i] = 0;
    }

    OverlapArea lastOVA;
    lastOVA.startID = -1;
    lastOVA.endID = -1;
    ballPerceptState.ballObstacleOverlap = false; // TODO: as of now, reset for every scan, check this
    bool overlap = false;
    bool wasOverlap = false;
    bool onImageEdge = false;
    bool onShadow = false;
    const int expectedShadowBrightness = std::min(128, fieldColor.fieldColorArray[0].fieldColorOptY);
    const Image::Pixel* lastP = nullptr;
    int i = 0;

    // run maxI scan lines
    for (i = 0; i < maxI; i++)
    {
      scanPoint = spot.position;
      yJumpLine = 0;
      first = true;
      int divisor = std::max(std::abs(xDir), std::abs(yDir));
      lengthPerStep = std::sqrt(sqr(static_cast<float>(xDir)) + sqr(static_cast<float>(yDir))) / divisor;
      wrongColorCount = 0;
      greenCount = 0;
      overlap = false;
      foundGreen = false;
      foundBallColor = false;
      pointNo = 0;
      lastP = &image[scanPoint.y()][scanPoint.x()];
      goingUp = lastP->y > 128;
      maxY = 0;
      scanLength = 0.f;
      onImageEdge = false;
      // run single scan line from spot.position in direction given via xDir/yDir
      for (int j = 0; scanLength < maxScanSize && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), 4); j++)
      {
        const Image::Pixel& p = image[scanPoint.y()][scanPoint.x()];
        newY = p.y;
        maxY = std::max(maxY, newY);
        pointNo++;
        bool isGreen = fieldColor.isPixelFieldColor(p.y, p.cb, p.cr);
        if (isGreen || std::abs(p.cb - lastP->cb) > maxColorJumpDiff || std::abs(p.cb - spot.cb) > maxColorDiff || std::abs(p.cr - spot.cr) > maxColorDiff || std::abs(p.cr - lastP->cr) > maxColorJumpDiff)
        {
          wrongColorCount++;
          greenCount += isGreen;

          if ((wrongColorCount > 5 && !(theFieldDimensions.ballType == FieldDimensions::BallType::any)) || greenCount > maxGreenCount)
          {
            break;
          }
          if ((theFieldDimensions.ballType == FieldDimensions::BallType::any) && greenCount == 0)
            lastBallColorPos = scanPoint;
        }
        else
        {
          ballColorCount += isPixelBallColor(p.y, p.cb, p.cr, fieldColor);
          onShadow = p.y < expectedShadowBrightness && yDir > -2; // no shadow on top of ball
          pointNoSum++;
          wrongColorCount = 0;
          lastBallColorPos = scanPoint;
          foundBallColor = true;

          // yJump stuff
          if (scanLength < yJumpRadius)
          {
            if (first)
            {
              first = false;
              lastY = newY;
            }
            int yDiff = std::abs(newY - lastY);
            if (!goingUp && ((newY - lastY) > 30 || newY - maxY < 50) && yDiff < 255)
            {
              yJumpLine++;
              goingUp = true;
            }
            if (goingUp && (newY - lastY) < -30 && yDiff < 255)
            {
              maxY = 0;
              yJumpLine++;
              goingUp = false;
            }
            lastY = newY;
            ballPerceptState.yHistogram[newY / 8]++;
          }
        }
        scanPoint.x() = spot.position.x() + (j * xDir) / divisor;
        scanPoint.y() = spot.position.y() + (j * yDir) / divisor;
        scanLength += lengthPerStep;
        lastP = &p;
      }
      if (image.isOutOfImage(scanPoint.x(), scanPoint.y(), 4))
        onImageEdge = true;
      //xInc = xInc*(2 * (!(xInc - 4)) - 1);
      // for detecting possible ball center, later compared to center of circle match on ball hull
      if (yDir == 4)
      {
        if (onImageEdge)
          scannedCenter.y() = spot.position.y() + expectedSizeInImage / 2;
        else
          scannedCenter.y() = static_cast<float>(lastBallColorPos.y());
      }
      if (xDir == 4)
      {
        scannedCenter.x() = static_cast<float>(lastBallColorPos.x());
        xInc = -increment;
      }
      if (xDir == -4)
      {
        scannedRadius = (scannedRadius + scannedCenter.x() - lastBallColorPos.x()) / 2;
        scannedCenter.x() = (scannedCenter.x() + lastBallColorPos.x()) / 2;
        xInc = increment;
      }
      if (yDir == -4)
      {
        if (pointNo >= maxScanSize || onImageEdge)
        {
          scannedRadius = expectedSizeInImage / 2;
          scannedCenter.y() = scannedCenter.y() - expectedSizeInImage / 2;
        }
        else
        {
          scannedRadius = scannedCenter.y() - lastBallColorPos.y();
          scannedCenter.y() = (scannedCenter.y() + lastBallColorPos.y()) / 2;
        }
        yInc = increment;
      }
      yJumpSum += yJumpLine;
      /*ColorRGBA lineColor = spot.found ? ColorRGBA(255, 0, 0) : (spot.centerFound ? ColorRGBA(200, 0, 0) : ColorRGBA(100, 0, 0));
    LINE("module:CLIPBallPerceptor:ballScanLines",
      spot.position.x, spot.position.y,
      scanPoint.x, scanPoint.y,
      std::max(1, static_cast<int>(spot.radiusInImage / 15)), Drawings::ps_solid, lineColor);*/
      foundGreen = (greenCount > 0);
      if (scanLength >= maxScanSize + wrongColorCount * lengthPerStep && greenCount < maxGreenCount + 1)
      {
        // usually this is a bad sign, but for white/black ball this could happen in front of robot/goal
        if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack)
        {
          /*if (yDir == 0 && xDir > 0)
          rightOverlap = true;
        if (yDir == 0 && xDir < 0)
          leftOverlap = true;*/
          ballPerceptState.ballObstacleOverlap = ballPerceptState.ballObstacleOverlap || !onShadow;
          if (!onShadow && !wasOverlap)
          {
            lastOVA.startID = i;
          }
          if (!onShadow)
            lastOVA.endID = i;
          if (yDir == 4)
            scannedCenter.y() = spot.position.y() + expectedSizeInImage / 2;

          wasOverlap = !onShadow;
          xDir += xInc;
          yDir += yInc;
          continue; // do not add to ball hull points here
        }
        else
          return ret;
      }
      if (!overlap && wasOverlap)
      {
        OverlapArea& ova = ballPerceptState.overlapAreas.emplace_back();
        ova.startID = lastOVA.startID;
        ova.endID = lastOVA.endID;
      }
      wasOverlap = overlap;
      if (!onImageEdge)
      {
        BallHullPoint bhp;
        bhp.pointInImage = lastBallColorPos.cast<float>();
        bhp.directionID = i;
        // good ball edge point, if green was found, ball color was found and scanline not too long
        if (foundBallColor && scanLength <= expectedSizeInImage && !image.isOutOfImage(scanPoint.x(), scanPoint.y(), minDistFromImageBorder) && (wrongColorCount > 0 && foundGreen))
        {
          goodBallHullPoints.push_back(bhp);
          yJumpSum += yJumpLine;
        }
        if (pointNo > wrongColorCount)
          ballHullPoints.push_back(bhp);
      }
      xDir += xInc;
      yDir += yInc;
    }

    if (wasOverlap)
    {
      if (lastOVA.startID == 0)
      {
        if (ballPerceptState.overlapAreas.empty())
          return ret;
        else
        {
          ballPerceptState.overlapAreas[0].startID = lastOVA.startID - 16;
        }
      }
      else
      {
        bool lastIsFirst = false;
        for (auto& ova : ballPerceptState.overlapAreas)
        {
          if (ova.startID == 0)
          {
            ova.startID = lastOVA.startID;
            lastIsFirst = true;
            break;
          }
        }
        if (!lastIsFirst)
        {
          OverlapArea& ova = ballPerceptState.overlapAreas.emplace_back();
          ova.startID = lastOVA.startID;
          ova.endID = lastOVA.endID;
        }
      }
    }
    // remove everything that has an overlap to the lower side
    for (auto& ova : ballPerceptState.overlapAreas)
    {
      if (ova.startID < 2 || ova.startID > 14 || ova.endID > 14)
      {
        if (ova.endID - ova.startID > (spot.upper ? 1 : 2))
          return ret;
        else
          ballPerceptState.detailedCheckNeeded = true;
      }
    }

    if (ballPerceptState.overlapAreas.size() == 2)
    {
      const OverlapArea& firstOVA = ballPerceptState.overlapAreas[0];
      const OverlapArea& secondOVA = ballPerceptState.overlapAreas[1];
      if ((firstOVA.endID - firstOVA.startID) < 2 && (secondOVA.endID - secondOVA.startID) < 2 && std::abs(Angle::normalize(firstOVA.startID * pi_8 - secondOVA.startID * pi_8)) > 75_deg)
        ballPerceptState.ballOnFieldLine = true;
    }

    DEBUG_DRAWING("module:CLIPBallPerceptor:overlapAreasRaw", "drawingOnImage")
    {
      if (spot.found)
      {
        float radius = expectedSizeInImage / 2;
        int numOfPoints = 3;
        Vector2f points[3];
        for (auto& ova : ballPerceptState.overlapAreas)
        {
          points[0] = spot.position.cast<float>();
          points[1] = Vector2f(0.f, 1.f);
          points[1].normalize(radius);
          points[1].rotate(ova.startID * pi_8);
          points[1] += points[0];
          points[1] = Vector2f(0.f, 1.f);
          points[1].normalize(radius);
          points[1].rotate(ova.endID * pi_8);
          points[2] += points[0];
          POLYGON("module:CLIPBallPerceptor:overlapAreasRaw", numOfPoints, points, 3, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
        }
      }
    }

    // scans done now check if spot does make sense so far

    if (theFieldDimensions.ballType == FieldDimensions::BallType::whiteBlack && !ballPerceptState.ballOnFieldLine)
    {
      for (auto& line : theCLIPFieldLinesPercept.lines)
      {
        if (line.fromUpper != spot.upper || line.lineWidthStart < 5)
          continue;
        Geometry::Line lineToCheck;
        lineToCheck.base = line.startInImage.cast<float>();
        lineToCheck.direction = (line.endInImage - line.startInImage).cast<float>();
        Vector2f firstInterSection, secondInterSection; // not used
        if (Geometry::getIntersectionOfLineAndCircle(lineToCheck, Geometry::Circle(scannedCenter, scannedRadius / 2), firstInterSection, secondInterSection) > 0)
        {
          ballPerceptState.ballOnFieldLine = true;
          Vector2f directionNormed = lineToCheck.direction; //TODO: check if direction is always left to right!
          directionNormed.normalize(scannedRadius);
          Vector2f toLineBorder = directionNormed;
          toLineBorder.rotateLeft();
          toLineBorder.normalize((line.lineWidthStart + line.lineWidthEnd) / 4); // TODO: wrong, just approx.. enough?
          lineUpperBorder.base = scannedCenter - directionNormed + toLineBorder;
          lineUpperBorder.direction.x() = (line.lineWidthStart + line.lineWidthEnd) / 2; // to remember width
          lineUpperBorder.direction.y() = lineUpperBorder.direction.x();
          lineLowerBorder.base = scannedCenter + directionNormed - toLineBorder; // TODO: see above
          lineLowerBorder.direction = lineUpperBorder.direction;
          break;
        }
      }
    }

    // overlap allowed?
    if (ballPerceptState.ballObstacleOverlap && !allowBallObstacleOverlap && !ballPerceptState.ballOnFieldLine)
    {
      return ret;
    }

    // specific colored balls have different requirements
    int minPercent = 50;
    if (theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack)
    {
      if (spot.found && yJumpSum < minNumberOfYJumps && scannedRadius / 2 > minRadiusInImage)
        return ret;
      minPercent = 40;
    }
    if (theFieldDimensions.ballType != SimpleFieldDimensions::BallType::any && pointNoSum > 0 && (100 * ballColorCount) / pointNoSum < minPercent)
      return ret;

    ColorRGBA centerColor = spot.found ? ColorRGBA(50, 50, 50) : (spot.centerFound ? ColorRGBA(25, 25, 25) : ColorRGBA(0, 0, 0));
    CROSS("module:CLIPBallPerceptor:ballScannedCenter", scannedCenter.x(), scannedCenter.y(), 5, 2, Drawings::solidPen, centerColor);

    if (!spot.centerFound) // 1st call, checks for scannedCenter scan
    {
      // only go on if at least 3 of 4 of the orthogonal lines are reasonable
      // exception: ball on possible field line (only allow horizontal lines and no other overlapping!)
      if (ballHullPoints.size() < 3 && !(ballPerceptState.ballOnFieldLine && ballHullPoints.size() >= 2))
        return ret;
      spot.centerFound = true;
      spot.position = scannedCenter.cast<int>();
      std::tie(ret_fill, ret_ballPatch) = recursiveLambda(spot);
      spot.found = ret_fill; //2nd scan
      if (!spot.found)
        return ret;
      spot.position = scannedCenter.cast<int>();
      return recursiveLambda(spot); //3rd scan
    }
    else if (!spot.found) // checks for 2nd call (center verification)
    {
      // only go on if at least 3 of 4 of the orthogonal lines are reasonable
      // exception: ball on possible field line (only allow horizontal lines and no other overlapping!)
      if (ballHullPoints.size() >= 3 || (ballHullPoints.size() >= 2 && ballPerceptState.ballOnFieldLine))
      {
        ret_fill = true;
        return ret;
      }
    }
    else // checks for 3rd call (ball hull verification)
    {
      // TODO: use yHistogram
      ballPerceptState.circleOK = false;
      std::tie(ret_fill, ret_ballPatch) = verifyBallHull(spot, ballHullPoints, goodBallHullPoints, scannedCenter);
      if (ret_fill)
        return ret;
      else if (ballPerceptState.circleOK && theFieldDimensions.ballType == SimpleFieldDimensions::BallType::whiteBlack)
      {
        if (!spot.upper && spot.radiusInImage > 25)
          ballPerceptState.detailedCheckNeeded = true;
        else if (ballPerceptState.ballObstacleOverlap && !ballPerceptState.ballOnFieldLine)
          return ret;
        //Vector2f posOnField;
        //if (!Transformation::imageToRobotHorizontalPlane(Vector2f(spot.position.cast<float>()), theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField))
        //  return ret;
        //if (true) //(posOnField - theBallModel.estimate.position).abs() < 500)
        //{
        //  BallSpot& newBallSpot = localBallSpots.ballSpots.emplace_back();
        //  newBallSpot.position = spot.position;
        //  newBallSpot.radiusInImage = spot.radiusInImage;
        //  newBallSpot.upper = upper;
        //  COMPLEX_DRAWING("module:CLIPBallPerceptor:ballScanLines")
        //  {
        //    std::vector<BallHullPoint>& points = ballPerceptState.ballObstacleOverlap ? goodBallHullPoints : ballHullPoints;
        //    for (int i = 0; i < (int)points.size(); i++)
        //      LINE("module:CLIPBallPerceptor:ballScanLines", newBallSpot.position.x(), newBallSpot.position.y(), points[i].pointInImage.x(), points[i].pointInImage.y(), 1, Drawings::solidPen, ColorRGBA::yellow);
        //  }
        //  return true;
        //}
      }
    }
    return ret; // default
  };

  return recursiveLambda(spot);
}


MAKE_MODULE(CLIPBallPerceptor, perception)
