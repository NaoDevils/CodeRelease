#include "RobotClassifier.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include <taskflow/taskflow.hpp>
#include "Tools/Build.h"
#include "Tools/ImageCoordinateTransformations.h"
#include "Tools/Math/Transformation.h"

static int roundToInt(float x)
{
  return static_cast<int>(round(x));
}

static Vector2i roundToInt(Vector2f x)
{
  return (x + Vector2f::Constant(0.5f)).cast<int>();
}

static float halfWidthFromHeight(float height)
{
  return height * 0.2f;
}

RobotClassifier::RobotClassifier()
{
  std::string basePath = std::string(File::getBHDir()) + "/Config/tflite/robot/";
  initModel(basePath + classifierName, classificationModelInterpreter, classificationModel);
  initModel(basePath + bboxCorrectifierName, bboxCorrectionModelInterpreter, bboxCorrectionModel);
}

void RobotClassifier::update(RobotsPerceptClassified& robotsPerceptClassified)
{
  std::swap(robotsPerceptClassified, localRobotsPerceptClassified);
}

void RobotClassifier::update(ProcessedRobotsHypotheses& processedRobotsHypotheses)
{
  std::swap(processedRobotsHypotheses, localRobotsHypotheses);
}

void RobotClassifier::execute(tf::Subflow& subflow)
{
  classifyHypotheses();
  postProcess();
}

void RobotClassifier::postProcess()
{
  if (localRobotsPerceptClassified.robots.size() == 0)
  {
    return;
  }
  sortLocalClassificationsByY();
  doRemainingClassifications();

  if (enableTracking)
  {
    interpolateTrackedBboxes();
  }
  removeInvalidatedRobots();

  lastValidEstimates.clear();
  copy(localRobotsPerceptClassified.robots.begin(), localRobotsPerceptClassified.robots.end(), back_inserter(lastValidEstimates));
}

void RobotClassifier::removeInvalidatedRobots()
{
  localRobotsPerceptClassified.robots.erase(
      std::remove_if(localRobotsPerceptClassified.robots.begin(),
          localRobotsPerceptClassified.robots.end(),
          [&](const RobotEstimate& re)
          {
            return re.validity == 0;
          }),
      localRobotsPerceptClassified.robots.end());
  //for (RobotEstimate& re : localRobotsPerceptClassified.robots)
  //{
  //  printIou(re);
  //}
}

void RobotClassifier::interpolateTrackedBboxes()
{
  for (RobotEstimate& re : localRobotsPerceptClassified.robots)
  {
    std::optional<RobotEstimate> reOldOpt = findClosestOldRobot(re);
    if (!reOldOpt.has_value())
      continue;
    RobotEstimate& reOld = reOldOpt.value();
    Vector2f resF;
    static_cast<void>(Transformation::robotToImage(reOld.locationOnField.translation, theCameraMatrixUpper, theCameraInfoUpper, resF));
    Vector2i reOldPos = resF.cast<int>();
    Vector2i reOldPosNow = Vector2i((reOld.imageLowerRight.x() + reOld.imageUpperLeft.x()) / 2, reOld.imageLowerRight.y());
    Vector2i moved = reOldPosNow - reOldPos;
    Vector2i movedOldUl = reOld.imageUpperLeft + moved, movedOldLr = reOld.imageLowerRight + moved;

    interpolateBbox(re, movedOldUl, movedOldLr, oldInterpolationFactor, false);
    re.trackingAge = reOld.trackingAge + 1;
    DRAWTEXT("module:RobotClassifier:tracking:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA::black, std::to_string(re.trackingAge));
    ARROW("module:RobotClassifier:tracking:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), reOld.imageLowerRight.x(), reOld.imageLowerRight.y(), 2, Drawings::solidPen, ColorRGBA::black);
  }
}

void RobotClassifier::sortLocalClassificationsByY()
{
  const auto sorting_criteria = [](const RobotEstimate& a, const RobotEstimate& b)
  {
    if (a.fromUpperImage && !b.fromUpperImage)
      return false;
    if (!a.fromUpperImage && b.fromUpperImage)
      return true;
    return a.imageLowerRight.y() > b.imageLowerRight.y();
  };
  std::sort(localRobotsPerceptClassified.robots.begin(), localRobotsPerceptClassified.robots.end(), sorting_criteria);
}

void RobotClassifier::doRemainingClassifications()
{
  // do the remaining bbox corrections but at most 2 per robot, thus total 3 per robot as one bbox correction was done earlier already
  int remainingBboxCorrections = std::min(maxBboxCorrections - doneBboxCorrections, static_cast<int>(localRobotsPerceptClassified.robots.size()) * 2);
  for (int i = 0; i < remainingBboxCorrections; i++)
  {
    if (maxBboxCorrections <= doneBboxCorrections)
      break;
    for (RobotEstimate& re : localRobotsPerceptClassified.robots)
    {
      if (maxBboxCorrections <= doneBboxCorrections)
        break;
      correctBbox(re);
      filterNms(re, true);
    }
  }
}

std::optional<RobotEstimate> RobotClassifier::findClosestOldRobot(const RobotEstimate& re)
{
  Vector2i rePos((re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2, re.imageLowerRight.y());
  if (!re.fromUpperImage)
    ImageCoordinateTransformations::toUpper(rePos, rePos);
  const int maxDist = roundToInt((re.imageLowerRight.x() - re.imageUpperLeft.x()) * maxTrackingDist);
  for (auto& reOld : lastValidEstimates)
  {
    Vector2f resF;
    static_cast<void>(Transformation::robotToImage(reOld.locationOnField.translation, theCameraMatrixUpper, theCameraInfoUpper, resF));
    Vector2i reOldPos = resF.cast<int>();
    if (!reOld.fromUpperImage)
      ImageCoordinateTransformations::toUpper(reOldPos, reOldPos);

    const int dist = (rePos - reOldPos).norm();

    if (dist < maxDist)
    {
      return std::optional<RobotEstimate>(reOld);
    }
  }
  return std::nullopt;
}

void RobotClassifier::acceptRecallRobots()
{
  for (auto& re : declinedButPossibleRobots)
  {
    if (filterNms(re, true))
    {
      printDeclinedRobot(re, std::to_string(re.validity));
      continue;
    }
    // get closest old robot
    std::optional<RobotEstimate> closestOldRe = findClosestOldRobot(re);
    if (!closestOldRe.has_value())
      continue;
    RobotEstimate reOld = closestOldRe.value();
    Vector2f resF;
    static_cast<void>(Transformation::robotToImage(reOld.locationOnField.translation, theCameraMatrixUpper, theCameraInfoUpper, resF));
    // correct bbox once, as it was ommitted before
    correctBbox(re);
    // filter again in case the bbox prediction changed enough, put into classified vector
    if (!filterNms(re, true))
    {
      re.source = RobotEstimate::recallAcceptance;
      localRobotsPerceptClassified.robots.push_back(re);
      Vector2i rePos = Vector2i((re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2, re.imageLowerRight.y());
    }
  }

  // remove newly filtered robot estimates
  removeInvalidatedRobots();
}

void RobotClassifier::classifyHypotheses()
{
  DECLARE_PLOT("module:RobotClassifier:sumOfRobotsHypotheses");
  DECLARE_PLOT("module:RobotClassifier:processedRobotsHypotheses");
  DECLARE_PLOT("module:RobotClassifier:validatedRobotPercepts");
  DECLARE_PLOT("module:RobotClassifier:declinedRobotPercepts");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:updateEstimate:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:updateEstimate:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:declinedRobotPercepts:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:declinedRobotPercepts:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:declinedRobotPercepts:Close", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:bboxCorrection:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:bboxCorrection:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:nms:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:recallRobots:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:tracking:Upper", "drawingOnImage");

  sumOfRobotsHypotheses = 0;
  processedRobotsHypotheses = 0;
  localRobotsPerceptClassified.robots.clear();
  localRobotsHypotheses.robots.clear();
  doneBboxCorrections = 0;

  // if the validity is similar, the nearest estimate is more important, else the more valid
  const auto sorting_criteria = [](const RobotEstimate& a, const RobotEstimate& b)
  {
    return a.validity > b.validity;
  };
  int classified = 0;
  declinedButPossibleRobots.clear();

  // Segmentor
  {
    Stopwatch s("RobotClassifier-update(Segmentor)");
    std::vector<RobotEstimate> sortedHyptotheses;
    copy(theRobotsHypothesesSegmentor.robots.begin(), theRobotsHypothesesSegmentor.robots.end(), back_inserter(sortedHyptotheses));
    std::sort(sortedHyptotheses.begin(), sortedHyptotheses.end(), sorting_criteria);

    sumOfRobotsHypotheses += sortedHyptotheses.size();
    for (auto& estimate : sortedHyptotheses)
    {
      Stopwatch s2("RobotClassifier-checkRobotEstimate");
      RobotEstimate& re = localRobotsHypotheses.robots.emplace_back(estimate);
      // check if the estimate can be filtered fast
      if (filterNms(re, false))
      {
        re.validity = 0;
        continue;
      }
      // classify
      if (re.validity >= 1.0)
      {
        correctBbox(re);
      }
      else
      {
        updateEstimate(re);
        classified++;
      }
      processedRobotsHypotheses++;
      if (re.validity >= classifierThresholdLower)
      {
        localRobotsPerceptClassified.robots.push_back(re);
      }
      else if (re.validity >= classifierThresholdRecall)
      {
        declinedButPossibleRobots.push_back(re);
      }
      else
      {
        printDeclinedRobot(re, std::to_string(re.validity));
      }
    }
  }

  // LOWER //
  {
    Stopwatch s("RobotClassifier-update(Lower)");
    std::vector<RobotEstimate> sortedHyptotheses;
    copy(theRobotsHypothesesYolo.robots.begin(), theRobotsHypothesesYolo.robots.end(), back_inserter(sortedHyptotheses));
    std::sort(sortedHyptotheses.begin(), sortedHyptotheses.end(), sorting_criteria);

    sumOfRobotsHypotheses += sortedHyptotheses.size();
    for (auto& estimate : sortedHyptotheses)
    {
      Stopwatch s1("RobotClassifier-checkRobotEstimate");
      RobotEstimate& re = localRobotsHypotheses.robots.emplace_back(estimate);
      // check if the estimate can be filtered fast
      if (filterNms(re, false))
      {
        re.validity = 0;
        continue;
      }
      // if budget is exceeded, print debug info and continue
      if (classified >= maxClassficationsLower)
      {
        if constexpr (!Build::targetSimulator())
          break;
        printDeclinedRobot(re, "");
        continue;
      }
      // else update with nets
      updateEstimate(re);
      classified++;
      processedRobotsHypotheses++;
      if (re.validity >= classifierThresholdLower)
      {
        localRobotsPerceptClassified.robots.push_back(re);
      }
      else if (re.validity >= classifierThresholdRecall)
      {
        declinedButPossibleRobots.push_back(re);
      }
      else
      {
        printDeclinedRobot(re, std::to_string(re.validity));
      }
    }
  }

  // UPPER //
  {
    Stopwatch s("RobotClassifier-update(Upper)");
    std::vector<RobotEstimate> sortedHyptotheses;
    copy(theRobotsHypothesesYoloUpper.robots.begin(), theRobotsHypothesesYoloUpper.robots.end(), back_inserter(sortedHyptotheses));
    std::sort(sortedHyptotheses.begin(), sortedHyptotheses.end(), sorting_criteria);

    sumOfRobotsHypotheses += sortedHyptotheses.size();
    for (auto& estimate : sortedHyptotheses)
    {
      Stopwatch s2("RobotClassifier-checkRobotEstimate");
      RobotEstimate& re = localRobotsHypotheses.robots.emplace_back(estimate);
      // check if the estimate can be filtered fast
      if (filterNms(re, false))
      {
        re.validity = 0;
        continue;
      }
      // if budget is exceeded, print debug info and continue
      if (classified >= maxClassficationsTotal)
      {
        if constexpr (!Build::targetSimulator())
          break;
        printDeclinedRobot(re, "");
        continue;
      }
      // else update the estimate
      updateEstimate(re);
      classified++;
      processedRobotsHypotheses++;
      if (re.validity >= classifierThreshold)
      {
        localRobotsPerceptClassified.robots.push_back(re);
      }
      else if (re.validity >= classifierThresholdRecall)
      {
        declinedButPossibleRobots.push_back(re);
      }
      else
      {
        printDeclinedRobot(re, std::to_string(re.validity));
      }
    }
  }

  int validatedRobotPercepts = static_cast<short>(localRobotsPerceptClassified.robots.size());
  PLOT("module:RobotClassifier:sumOfRobotsHypotheses", sumOfRobotsHypotheses);
  PLOT("module:RobotClassifier:processedRobotsHypotheses", processedRobotsHypotheses);
  PLOT("module:RobotClassifier:validatedRobotPercepts", validatedRobotPercepts);
  PLOT("module:RobotClassifier:declinedRobotPercepts", (processedRobotsHypotheses - validatedRobotPercepts));

  // RECALL
  acceptRecallRobots();
}

void RobotClassifier::initModel(std::string path, std::unique_ptr<tflite::Interpreter>& interpreter, std::unique_ptr<tflite::FlatBufferModel>& model)
{
  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(path.c_str());
  TFLITE_MINIMAL_CHECK(model != nullptr);

  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  TFLITE_MINIMAL_CHECK(interpreter != nullptr);
  interpreter->SetNumThreads(1);

  // Allocate memory for the tensors
  interpreter->AllocateTensors();

  // Check interpreter state
  // tflite::PrintInterpreterState(featureModelInterpreter.get());
}

void RobotClassifier::updateEstimate(RobotEstimate& re)
{
  Stopwatch s("RobotClassifier-updateEstimate");
  // skip if not valid to avoid unnecessary bbox correction
  if (classifyEstimate(re))
  {
    correctBbox(re);
    filterNms(re, true);
  }
}

bool RobotClassifier::classifyEstimate(RobotEstimate& re)
{
  Stopwatch s("RobotClassifier-classifyEstimate");
  Vector2i upperLeftArea, sizeArea;
  getArea(re, classifierMargin, upperLeftArea, sizeArea);

  const std::unique_ptr<tflite::Interpreter>& clfInterpreter = classificationModelInterpreter;
  unsigned char* input = clfInterpreter->typed_tensor<unsigned char>(clfInterpreter->inputs()[0]);
  Vector2i inputDims(clfInterpreter->input_tensor(0)->dims->data[2], clfInterpreter->input_tensor(0)->dims->data[1]);
  re.patch = std::vector<unsigned char>{input, input + inputDims.x() * inputDims.y() * 3};

  {
    Stopwatch s1("RobotClassifier-classifyEstimate-copyImageAndRunNet");
    {
      Stopwatch s2("RobotClassifier-classifyEstimate-copyAndResize");
      // dim shape is (batchSize, height, width, channels)
      (re.fromUpperImage ? theImageUpper : theImage).copyAndResizeArea(upperLeftArea, sizeArea, inputDims, input);
      if (!re.fromUpperImage && upperLeftArea.y() < 0)
      {
        Vector2i ulUpper = Vector2i();
        Vector2i lrUpper = Vector2i();
        ImageCoordinateTransformations::toUpper(upperLeftArea, ulUpper);
        ImageCoordinateTransformations::toUpper(upperLeftArea + sizeArea, lrUpper);
        theImageUpper.copyAndResizeArea<true, true, false>(ulUpper, lrUpper - ulUpper, inputDims, input);
      }
    }
    {
      Stopwatch s3("RobotClassifier-classifyEstimate-runNet");
      if (clfInterpreter->Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
        return true;
      }
    }
    re.validity = clfInterpreter->typed_output_tensor<float>(0)[0];
  }
  return re.validity >= (re.fromUpperImage ? classifierThreshold : classifierThreshold);
}

void RobotClassifier::correctBbox(RobotEstimate& re)
{
  // old bbox before corrections
  const Vector2i ulOld = re.imageUpperLeft;
  const Vector2i lrOld = re.imageLowerRight;
  if (useBboxPrediction)
  {
    Vector2i ulPred = Vector2i();
    Vector2i lrPred = Vector2i();
    predictBbox(re, ulPred, lrPred);
    if (interpolatePredictedBbox)
    {
      interpolateBbox(re, ulPred, lrPred, interpolatePredictedBboxFactor, false);
    }
    else
    {
      re.imageUpperLeft = ulPred, re.imageLowerRight = lrPred;
    }
  }
  if (useGeometricHeight)
  {
    Vector2i ulGeometric = Vector2i();
    Vector2i lrGeometric = Vector2i();
    getGeometricBbox(re, ulGeometric, lrGeometric);
    if (interpolateGeometricBbox)
    {
      interpolateBbox(re, ulGeometric, lrGeometric, interpolateGeometricBboxFactor, true);
    }
    else
    {
      re.imageUpperLeft = ulGeometric, re.imageLowerRight = lrGeometric;
    }
  }
  calculateDistance(re);

  // draws the area, the prediction and an arrow from the area to the prediction
  if (re.fromUpperImage)
  {
    RECTANGLE("module:RobotClassifier:bboxCorrection:Upper", ulOld.x(), ulOld.y(), lrOld.x(), lrOld.y(), 1, Drawings::dottedPen, ColorRGBA::black);
    ARROW("module:RobotClassifier:bboxCorrection:Upper", lrOld.x(), lrOld.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 1, Drawings::dottedPen, ColorRGBA::black);
  }
  else
  {
    RECTANGLE("module:RobotClassifier:bboxCorrection:Lower", ulOld.x(), ulOld.y(), lrOld.x(), lrOld.y(), 1, Drawings::dottedPen, ColorRGBA::black);
    ARROW("module:RobotClassifier:bboxCorrection:Lower", lrOld.x(), lrOld.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 1, Drawings::dottedPen, ColorRGBA::black);
  }
}

void RobotClassifier::calculateDistance(RobotEstimate& re)
{
  const int midX = (re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2;
  const CameraMatrix& cameraMatrix = re.fromUpperImage ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = re.fromUpperImage ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const bool outOfField = !Transformation::imageToRobot(Vector2f(midX, re.imageLowerRight.y()), cameraMatrix, cameraInfo, re.locationOnField.translation);
  if (outOfField)
  {
    re.validity = 0;
    printDeclinedRobot(re, "outOfField");
  }
  re.distance = re.locationOnField.translation.norm();
}

void RobotClassifier::getGeometricBbox(RobotEstimate& re, Vector2i& ul, Vector2i& lr)
{
  // update height from known robot dimensions if wanted
  const CameraMatrix& cameraMatrix = re.fromUpperImage ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = re.fromUpperImage ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const int midX = (re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2;
  const bool outOfField = !Transformation::imageToRobot(Vector2f(midX, re.imageLowerRight.y()), cameraMatrix, cameraInfo, re.locationOnField.translation);
  if (outOfField)
  {
    re.validity = 0;
    printDeclinedRobot(re, "outOfField");
  }
  re.distance = re.locationOnField.translation.norm();
  const float heightInImage = Geometry::getSizeByDistance(cameraInfo, ROBOT_HEIGHT, re.distance);

  ul.x() = roundToInt(midX - halfWidthFromHeight(heightInImage));
  ul.y() = roundToInt(re.imageLowerRight.y() - heightInImage);
  lr.x() = roundToInt(midX + halfWidthFromHeight(heightInImage));
  lr.y() = re.imageLowerRight.y();
}

void RobotClassifier::interpolateBbox(RobotEstimate& re, Vector2i& ul, Vector2i& lr, float factor, bool keepLower)
{
  const Vector2f mid = (ul + lr).cast<float>() / 2;
  const int height = lr.y() - ul.y();
  const Vector2f midOld = (re.imageUpperLeft + re.imageLowerRight).cast<float>() / 2;
  const int heightOld = re.imageLowerRight.y() - re.imageUpperLeft.y();

  const Vector2i midInterpolated = roundToInt(factor * mid + (1 - factor) * midOld);
  const float heightInterpolated = factor * height + (1 - factor) * heightOld;

  if (keepLower)
  {
    re.imageLowerRight.y() = re.imageLowerRight.y();
    re.imageUpperLeft.y() = roundToInt(re.imageLowerRight.y() - heightInterpolated);
    re.imageLowerRight.x() = roundToInt(midInterpolated.x() + halfWidthFromHeight(heightInterpolated));
    re.imageUpperLeft.x() = roundToInt(midInterpolated.x() - halfWidthFromHeight(heightInterpolated));
  }
  else
  {
    re.imageUpperLeft.x() = roundToInt(midInterpolated.x() - halfWidthFromHeight(heightInterpolated));
    re.imageUpperLeft.y() = roundToInt(midInterpolated.y() - heightInterpolated / 2.f);
    re.imageLowerRight.x() = roundToInt(midInterpolated.x() + halfWidthFromHeight(heightInterpolated));
    re.imageLowerRight.y() = roundToInt(midInterpolated.y() + heightInterpolated / 2.f);
  }
}

void RobotClassifier::getArea(RobotEstimate& re, const float margin, Vector2i& upperLeftArea, Vector2i& sizeArea)
{
  // the net is bad at predicting bboxes that are larger than the input, therefore we take the max of the geometric and original bbox plus some margin
  Vector2i ulGeometric = Vector2i();
  Vector2i lrGeometric = Vector2i();
  getGeometricBbox(re, ulGeometric, lrGeometric);

  const Vector2i sizeGeometric = lrGeometric - ulGeometric;
  const Vector2i sizeRe = re.imageLowerRight - re.imageUpperLeft;

  const Vector2i size = sizeGeometric.y() > sizeRe.y() ? sizeGeometric : sizeRe;
  const Vector2i ul = sizeGeometric.y() > sizeRe.y() ? ulGeometric : re.imageUpperLeft;
  const Vector2i lr = sizeGeometric.y() > sizeRe.y() ? lrGeometric : re.imageLowerRight;

  const Vector2i marginAbs = roundToInt(size.cast<float>() * margin);

  upperLeftArea = ul - marginAbs;
  sizeArea = size + 2 * marginAbs;
}

void RobotClassifier::predictBbox(RobotEstimate& re, Vector2i& ulPred, Vector2i& lrPred)
{
  Stopwatch s("RobotClassifier-correctBbox");
  Vector2i upperLeftArea, sizeArea;
  getArea(re, bboxMargin, upperLeftArea, sizeArea);

  // get bbox
  {
    Stopwatch s1("RobotClassifier-correctBbox-copyImageAndRunNet");
    const std::unique_ptr<tflite::Interpreter>& bboxInterpreter = bboxCorrectionModelInterpreter;
    unsigned char* input = bboxInterpreter->typed_tensor<unsigned char>(bboxInterpreter->inputs()[0]);
    Vector2i inputDims(bboxInterpreter->input_tensor(0)->dims->data[2], bboxInterpreter->input_tensor(0)->dims->data[1]);
    {
      Stopwatch s2("RobotClassifier-correctBbox-copyAndResize");
      // dim shape is (batchSize, height, width, channels)
      (re.fromUpperImage ? theImageUpper : theImage).copyAndResizeArea(upperLeftArea, sizeArea, inputDims, input);
      if (!re.fromUpperImage && upperLeftArea.y() < 0)
      {
        Vector2i ulUpper = Vector2i();
        Vector2i lrUpper = Vector2i();
        ImageCoordinateTransformations::toUpper(upperLeftArea, ulUpper);
        ImageCoordinateTransformations::toUpper(upperLeftArea + sizeArea, lrUpper);
        theImageUpper.copyAndResizeArea<true, true, false>(ulUpper, lrUpper - ulUpper, inputDims, input);
      }
    }
    {
      Stopwatch s3("RobotClassifier-correctBbox-runNet");
      if (bboxInterpreter->Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
        return;
      }
    }

    // outputs shape : (midX, midY, half Height)
    float* bboxOutput = bboxInterpreter->typed_output_tensor<float>(0);
    const Vector2f midRel(bboxOutput[0], bboxOutput[1]);
    const Vector2f halfSizeRel(bboxOutput[2], bboxOutput[2]);

    // relative anchor points
    const Vector2f ulRel = midRel - halfSizeRel;
    const Vector2f lrRel = midRel + halfSizeRel;

    // absolute anchor points
    const Vector2f ulOffset = ulRel.cwiseProduct(sizeArea.cast<float>());
    const Vector2f lrOffset = lrRel.cwiseProduct(sizeArea.cast<float>());

    // fill prediction
    ulPred = roundToInt(upperLeftArea.cast<float>() + ulOffset);
    lrPred = roundToInt(upperLeftArea.cast<float>() + lrOffset);
  }
}

bool RobotClassifier::filterNms(RobotEstimate& re, bool respectValidity)
{
  for (RobotEstimate& other : localRobotsPerceptClassified.robots)
  {
    if (&re == &other)
      continue;
    const float overlap = iou(re, other);
    if (overlap >= nonMaximumSuppressionThreshold)
    {
      RobotEstimate& guilty = other;

      if (respectValidity && re.validity > guilty.validity)
      {
        guilty.validity = 0.f;
        printDeclinedRobot(guilty, "nms");
      }
      else
      {
        re.validity = 0.f;
        printDeclinedRobot(re, "nms");
        return true;
      }
    }
  }
  return false;
}
bool RobotClassifier::printIou(RobotEstimate& re)
{
  for (size_t i = 0; i < localRobotsPerceptClassified.robots.size(); i++)
  {
    RobotEstimate& other = localRobotsPerceptClassified.robots[i];
    if (other.imageLowerRight == re.imageLowerRight)
      continue;
    const float overlap = iou(re, other);
    if (overlap == 0)
      continue;
    Vector2i pos = roundToInt(.3 * Vector2f(re.imageLowerRight.cast<float>()) + .7 * Vector2f(other.imageLowerRight.cast<float>()));
    DRAWTEXT("module:RobotClassifier:nms:Upper", pos.x(), pos.y(), 10, ColorRGBA(0, 0, 0, 200), std::to_string(overlap));
    ARROW("module:RobotClassifier:nms:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), other.imageLowerRight.x(), other.imageLowerRight.y(), 1, Drawings::solidPen, ColorRGBA::black);
  }
  return false;
}

float RobotClassifier::iou(const RobotEstimate& re1, const RobotEstimate& re2)
{
  Vector2i ul1(re1.imageUpperLeft.x(), re1.imageUpperLeft.y());
  Vector2i lr1(re1.imageLowerRight.x(), re1.imageLowerRight.y());
  Vector2i ul2(re2.imageUpperLeft.x(), re2.imageUpperLeft.y());
  Vector2i lr2(re2.imageLowerRight.x(), re2.imageLowerRight.y());

  if (re1.fromUpperImage && !re2.fromUpperImage)
  {
    re2.getUpperImageCoordinates(ul2, lr2);
  }
  else if (!re1.fromUpperImage && re2.fromUpperImage)
  {
    re1.getUpperImageCoordinates(ul1, lr1);
  }

  const int intersection = std::max(0, std::min(lr1.x(), lr2.x()) - std::max(ul1.x(), ul2.x()) + 1) * std::max(0, std::min(lr1.y(), lr2.y()) - std::max(ul1.y(), ul2.y()) + 1);

  const int volumei = (lr1.x() - ul1.x()) * (lr1.y() - ul1.y());
  const int volumej = (lr2.x() - ul2.x()) * (lr2.y() - ul2.y());

  const int unite = volumei + volumej - intersection;

  return intersection / (float)unite;
}

void RobotClassifier::printDeclinedRobot(const RobotEstimate& re, const std::string reason)
{
  if constexpr (!Build::targetSimulator())
    return;
  unsigned char alpha = reason != "" ? 200 : 100;
  if (re.source == RobotEstimate::segmentor)
  {
    RECTANGLE(
        "module:RobotClassifier:declinedRobotPercepts:Segmentor", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, alpha));
    DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Segmentor", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), reason);
  }
  else if (re.fromUpperImage)
  {
    RECTANGLE(
        "module:RobotClassifier:declinedRobotPercepts:Upper", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, alpha));
    DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), reason);
  }
  else if (!re.fromUpperImage)
  {
    RECTANGLE(
        "module:RobotClassifier:declinedRobotPercepts:Lower", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, alpha));
    DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Lower", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), reason);
  }
}

MAKE_MODULE(RobotClassifier, perception);
