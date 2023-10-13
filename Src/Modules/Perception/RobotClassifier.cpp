#include "RobotClassifier.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/CameraInfo.h"

RobotClassifier::RobotClassifier()
{
  std::string basePath = std::string(File::getBHDir()) + "/Config/tflite/robot/";
  initModel(basePath + "robot_classifier_rec_0.824_thr_0.828.tflite", classificationModelInterpreter, classificationModel);
  initModel(basePath + "bbox_correctifier_y_rel_0.057.tflite", bboxCorrectionModelInterpreter, bboxCorrectionModel);
}

void RobotClassifier::update(RobotsPerceptClassified& robotsPerceptClassified)
{
  std::swap(robotsPerceptClassified, localRobotsPerceptClassified);
}

void RobotClassifier::update(ProcessedRobotsHypotheses& processedRobotsHypotheses)
{
  std::swap(processedRobotsHypotheses, localRobotsHypotheses);
}

void RobotClassifier::execute(tf::Subflow&)
{
  DECLARE_PLOT("module:RobotClassifier:sumOfRobotsHypotheses");
  DECLARE_PLOT("module:RobotClassifier:processedRobotsHypotheses");
  DECLARE_PLOT("module:RobotClassifier:validatedRobotPercepts");
  DECLARE_PLOT("module:RobotClassifier:declinedRobotPercepts");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:updateEstimate:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:updateEstimate:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:declinedRobotPercepts:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:declinedRobotPercepts:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:bboxCorrection:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RobotClassifier:bboxCorrection:Lower", "drawingOnImage");

  sumOfRobotsHypotheses = 0;
  processedRobotsHypotheses = 0;
  localRobotsPerceptClassified.robots.clear();
  localRobotsHypotheses.robots.clear();

  // if the validity is similar, the nearest estimate is more important, else the more valid
  auto sorting_criteria = [](const RobotEstimate& a, const RobotEstimate& b)
  {
    return a.validity > b.validity;
  };
  int classified = 0;

  // LOWER //
  {
    Stopwatch s("RobotClassifier-update(Lower)");
    std::vector<RobotEstimate> sortedHyptotheses;
    copy(theRobotsHypothesesYolo.robots.begin(), theRobotsHypothesesYolo.robots.end(), back_inserter(sortedHyptotheses));
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
      if (classified >= maxClassficationsLower)
      {
        RECTANGLE(
            "module:RobotClassifier:declinedRobotPercepts:Lower", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, 200));
        std::string str = "lower clf budget exceeded ";
        str.append(std::to_string(classified));
        DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Lower", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), str);
        continue;
      }
      // else update with nets
      updateEstimate(theImage, re);
      classified++;
      processedRobotsHypotheses++;
      if (re.validity >= classifierThresholdLower)
      {
        localRobotsPerceptClassified.robots.push_back(re);
        continue;
      }
      else
      {
        // if not accepted, print debug info
        RECTANGLE(
            "module:RobotClassifier:declinedRobotPercepts:Lower", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(0, 0, 0, 200));
        DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Lower", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), re.validity);
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
        RECTANGLE(
            "module:RobotClassifier:declinedRobotPercepts:Upper", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, 200));
        std::string str = "clf budget exceeded ";
        str.append(std::to_string(classified));
        DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), str);
        continue;
      }
      // else update the estimate
      updateEstimate(theImageUpper, re);
      classified++;
      processedRobotsHypotheses++;
      if (re.validity >= classifierThreshold)
      {
        localRobotsPerceptClassified.robots.push_back(re);
        continue;
      }
      else
      {
        // if not accepted, print debug info
        RECTANGLE(
            "module:RobotClassifier:declinedRobotPercepts:Upper", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(0, 0, 0, 200));
        DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), re.validity);
      }
    }
  }
  int validatedRobotPercepts = static_cast<short>(localRobotsPerceptClassified.robots.size());
  PLOT("module:RobotClassifier:sumOfRobotsHypotheses", sumOfRobotsHypotheses);
  PLOT("module:RobotClassifier:processedRobotsHypotheses", processedRobotsHypotheses);
  PLOT("module:RobotClassifier:validatedRobotPercepts", validatedRobotPercepts);
  PLOT("module:RobotClassifier:declinedRobotPercepts", (processedRobotsHypotheses - validatedRobotPercepts));
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

void RobotClassifier::updateEstimate(const Image& image, RobotEstimate& re)
{
  Stopwatch s("RobotClassifier-updateEstimate");
  // skip if not valid to avoid unnecessary bbox correction
  if (classifyEstimate(image, re))
  {
    correctBbox(image, re);
    filterNms(re, true);
  }
}

bool RobotClassifier::classifyEstimate(const Image& image, RobotEstimate& re)
{
  Stopwatch s("RobotClassifier-classifyEstimate");
  const Vector2i reSize = re.imageLowerRight - re.imageUpperLeft;
  const Vector2i margin = (reSize.cast<float>() * classifierMargin + Vector2f::Constant(0.5f)).cast<int>();

  const Vector2i upperLeftArea = re.imageUpperLeft - margin;
  const Vector2i sizeArea = reSize + 2 * margin;

  // get features
  int* inputDims = classificationModelInterpreter->input_tensor(0)->dims->data;
  {
    Stopwatch s2("RobotClassifier-classifyEstimate-copyImageAndRunNet");
    {
      Stopwatch s3("RobotClassifier-classifyEstimate-copyAndResize");
      // dim shape is (batchSize, height, width, channels)
      image.copyAndResizeArea(upperLeftArea, sizeArea, {inputDims[2], inputDims[1]}, classificationModelInterpreter->typed_input_tensor<unsigned char>(0));
      if (!re.fromUpperImage && re.imageUpperLeft.y() < 0)
      {
        Vector2i ul = re.imageUpperLeft;
        Vector2i lr = re.imageLowerRight;
        getUpperImageCoordinates(re, ul.x(), ul.y(), lr.x(), lr.y());
        theImageUpper.copyAndResizeArea<true, true, false>(ul, lr - ul, {inputDims[2], inputDims[1]}, classificationModelInterpreter->typed_input_tensor<unsigned char>(0));
      }
    }
    {
      Stopwatch s3("RobotClassifier-classifyEstimate-runNet");
      if (classificationModelInterpreter->Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
        return true;
      }
    }
    float* classificationOutput = classificationModelInterpreter->typed_output_tensor<float>(0);

    re.validity = classificationOutput[0];
  }
  return re.fromUpperImage ? re.validity >= classifierThreshold : re.validity >= classifierThresholdLower;
}

void RobotClassifier::correctBbox(const Image& image, RobotEstimate& re)
{
  if (useBboxPrediction)
  {
    int xUlPred, yUlPred, xLrPred, yLrPred;
    predictBbox(image, re, xUlPred, yUlPred, xLrPred, yLrPred);
    if (interpolatePredictedBbox)
    {
      interpolateBbox(re, xUlPred, yUlPred, xLrPred, yLrPred, interpolatePredictedBboxFactor, false);
    }
    else
    {
      re.imageUpperLeft.x() = xUlPred, re.imageUpperLeft.y() = yUlPred, re.imageLowerRight.x() = xLrPred, re.imageLowerRight.y() = yLrPred;
    }
  }
  if (useGeometricHeight)
  {
    int xUlGeometric, yUlGeometric, xLrGeometric, yLrGeometric;
    getGeometricBbox(re, xUlGeometric, yUlGeometric, xLrGeometric, yLrGeometric);
    if (interpolateGeometricBbox)
    {
      interpolateBbox(re, xUlGeometric, yUlGeometric, xLrGeometric, yLrGeometric, interpolateGeometricBboxFactor, true);
    }
    else
    {
      re.imageUpperLeft.x() = xUlGeometric, re.imageUpperLeft.y() = yUlGeometric, re.imageLowerRight.x() = xLrGeometric, re.imageLowerRight.y() = yLrGeometric;
    }
  }
  int midX = (re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2;
  const CameraMatrix& cameraMatrix = re.fromUpperImage ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = re.fromUpperImage ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  bool outOfField = !Transformation::imageToRobot(Vector2f(midX, re.imageLowerRight.y()), cameraMatrix, cameraInfo, re.locationOnField.translation);
  if (outOfField)
  {
    RECTANGLE(
        "module:RobotClassifier:declinedRobotPercepts:Upper", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, 200));
    DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), "out of field");
    re.validity = 0;
  }
  re.distance = re.locationOnField.translation.norm();
}

void RobotClassifier::getGeometricBbox(RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr)
{
  // update height from known robot dimensions if wanted
  const CameraMatrix& cameraMatrix = re.fromUpperImage ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = re.fromUpperImage ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  int midX = (re.imageUpperLeft.x() + re.imageLowerRight.x()) / 2;
  bool outOfField = !Transformation::imageToRobot(Vector2f(midX, re.imageLowerRight.y()), cameraMatrix, cameraInfo, re.locationOnField.translation);
  if (outOfField)
  {
    RECTANGLE(
        "module:RobotClassifier:declinedRobotPercepts:Upper", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 2, Drawings::dottedPen, ColorRGBA(75, 75, 75, 200));
    DRAWTEXT("module:RobotClassifier:declinedRobotPercepts:Upper", re.imageLowerRight.x(), re.imageLowerRight.y(), 10, ColorRGBA(0, 0, 0, 200), "out of field");
    re.validity = 0;
  }
  re.distance = re.locationOnField.translation.norm();
  float heightInImage = Geometry::getSizeByDistance(cameraInfo, 580.f, re.distance);

  xUl = static_cast<int>(round(midX - heightInImage / 5));
  yUl = static_cast<int>(round(re.imageLowerRight.y() - heightInImage));
  xLr = static_cast<int>(round(midX + heightInImage / 5));
  yLr = static_cast<int>(round(re.imageLowerRight.y()));
}

void RobotClassifier::interpolateBbox(RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr, float factor, bool keepLower)
{
  int midX = (xUl + xLr) / 2;
  int midY = (yUl + yLr) / 2;
  int height = yLr - yUl;
  int oldXul = re.imageUpperLeft.x(), oldYul = re.imageUpperLeft.y(), oldXlr = re.imageLowerRight.x(), oldYlr = re.imageLowerRight.y();
  int midXOld = (oldXul + oldXlr) / 2, midYOld = (oldYul + oldYlr) / 2;
  int heightOld = yLr - yUl;

  int midXInterpolated = static_cast<int>(std::round(factor * midX + (1 - factor) * midXOld)), midYInterpolated = static_cast<int>(std::round(factor * midY + (1 - factor) * midYOld));
  int heightInterpolated = static_cast<int>(std::round(factor * height + (1 - factor) * heightOld));

  re.imageUpperLeft.x() = static_cast<int>(midXInterpolated - heightInterpolated / 5.f);
  re.imageUpperLeft.y() = static_cast<int>(midYInterpolated - heightInterpolated / 2);
  re.imageLowerRight.x() = static_cast<int>(midXInterpolated + heightInterpolated / 5.f);
  if (!keepLower)
    re.imageLowerRight.y() = static_cast<int>(midYInterpolated + heightInterpolated / 2);
}

void RobotClassifier::predictBbox(const Image& image, RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr)
{
  Stopwatch s("RobotClassifier-correctBbox");
  const Vector2i reSize = re.imageLowerRight - re.imageUpperLeft;
  const Vector2i margin = (reSize.cast<float>() * bboxMargin + Vector2f::Constant(0.5f)).cast<int>();

  const Vector2i upperLeftArea = re.imageUpperLeft - margin;
  const Vector2i sizeArea = reSize + 2 * margin;

  // get features
  int* inputDims = classificationModelInterpreter->input_tensor(0)->dims->data;

  // get bbox
  {
    Stopwatch s2("RobotClassifier-correctBbox-copyImageAndRunNet");
    inputDims = bboxCorrectionModelInterpreter->input_tensor(0)->dims->data;
    {
      Stopwatch s3("RobotClassifier-correctBbox-copyAndResize");
      // dim shape is (batchSize, height, width, channels)
      image.copyAndResizeArea(upperLeftArea, sizeArea, {inputDims[2], inputDims[1]}, bboxCorrectionModelInterpreter->typed_input_tensor<unsigned char>(0));
    }
    {
      Stopwatch s3("RobotClassifier-correctBbox-runNet");
      if (bboxCorrectionModelInterpreter->Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
        return;
      }
    }
    // old bbox before corrections
    const Vector2i oldUl = re.imageUpperLeft;
    const Vector2i oldLr = re.imageLowerRight;

    // outputs
    float* bboxOutput = bboxCorrectionModelInterpreter->typed_output_tensor<float>(0);
    const Vector2f midRel(bboxOutput[0], bboxOutput[1]);
    const Vector2f sizeRel(bboxOutput[2], bboxOutput[2]);

    // relative anchor points
    const Vector2f ulRel = midRel - sizeRel;
    const Vector2f lrRel = midRel + sizeRel;

    // absolute anchor points
    const Vector2f ulOffset = ulRel.cwiseProduct(sizeArea.cast<float>());
    const Vector2f lrOffset = lrRel.cwiseProduct(sizeArea.cast<float>());

    const Vector2f Ul = upperLeftArea.cast<float>() + ulOffset;
    const Vector2f Lr = upperLeftArea.cast<float>() + lrOffset;

    // TODO: use Vector2i as output parameters
    xUl = static_cast<int>(Ul.x());
    yUl = static_cast<int>(Ul.y());
    xLr = static_cast<int>(Lr.x());
    yLr = static_cast<int>(Lr.y());

    // draws the area, the prediction and an arrow from the area to the prediction
    if (re.fromUpperImage)
    {
      RECTANGLE("module:RobotClassifier:bboxCorrection:Upper", oldUl.x(), oldUl.y(), oldLr.x(), oldLr.y(), 1, Drawings::dottedPen, ColorRGBA::black);
      ARROW("module:RobotClassifier:bboxCorrection:Upper", oldUl.x(), oldUl.y(), Ul.x(), Ul.y(), 1, Drawings::dottedPen, ColorRGBA::black);
    }
    else
    {
      RECTANGLE("module:RobotClassifier:bboxCorrection:Lower", oldUl.x(), oldUl.y(), oldLr.x(), oldLr.y(), 1, Drawings::dottedPen, ColorRGBA::black);
      ARROW("module:RobotClassifier:bboxCorrection:Lower", oldUl.x(), oldUl.y(), Ul.x(), Ul.y(), 1, Drawings::dottedPen, ColorRGBA::black);
    }
  }
}

bool RobotClassifier::filterNms(RobotEstimate& re, bool respectValidity)
{
  if (re.fromUpperImage)
  {
    for (size_t i = 0; i < localRobotsPerceptClassified.robots.size(); i++)
    {
      float overlap = iou(re, localRobotsPerceptClassified.robots[i]);
      if (overlap >= nonMaximumSuppressionThreshold)
      {
        RobotEstimate& guilty = localRobotsPerceptClassified.robots[i];
        RECTANGLE("module:RobotClassifier:declinedRobotPercepts", re.imageUpperLeft.x(), re.imageUpperLeft.y(), re.imageLowerRight.x(), re.imageLowerRight.y(), 1, Drawings::dottedPen, ColorRGBA::magenta);
        RECTANGLE("module:RobotClassifier:declinedRobotPercepts", guilty.imageUpperLeft.x(), guilty.imageUpperLeft.y(), guilty.imageLowerRight.x(), guilty.imageLowerRight.y(), 1, Drawings::dottedPen, ColorRGBA::magenta);
        ARROW("module:RobotClassifier:declinedRobotPercepts", guilty.imageUpperLeft.x(), guilty.imageUpperLeft.y(), re.imageUpperLeft.x(), re.imageUpperLeft.y(), 1, Drawings::dottedPen, ColorRGBA::magenta);

        if (respectValidity && re.validity > guilty.validity)
        {
          guilty.validity = 0.f;
        }
        else
        {
          re.validity = 0.f;
          return true;
        }
      }
    }
  }

  for (size_t i = 0; i < localRobotsPerceptClassified.robots.size(); i++)
  {
    float overlap = iou(re, localRobotsPerceptClassified.robots[i]);
    if (overlap >= nonMaximumSuppressionThreshold)
    {
      RobotEstimate& guilty = localRobotsPerceptClassified.robots[i];
      if (respectValidity && re.validity > guilty.validity)
      {
        guilty.validity = 0.f;
      }
      else
      {
        re.validity = 0.f;
        return true;
      }
    }
  }
  return false;
}

float RobotClassifier::iou(const RobotEstimate& re1, const RobotEstimate& re2)
{
  int ulX1 = re1.imageUpperLeft.x(), ulY1 = re1.imageUpperLeft.y(), lrX1 = re1.imageLowerRight.x(), lrY1 = re1.imageLowerRight.y();
  int ulX2 = re2.imageUpperLeft.x(), ulY2 = re2.imageUpperLeft.y(), lrX2 = re2.imageLowerRight.x(), lrY2 = re2.imageLowerRight.y();

  if (re1.fromUpperImage && !re2.fromUpperImage)
  {
    getUpperImageCoordinates(re2, ulX2, ulY2, lrX2, lrY2);
  }
  else if (!re1.fromUpperImage && re2.fromUpperImage)
  {
    getUpperImageCoordinates(re1, ulX1, ulY1, lrX1, lrY1);
  }

  int intersection = std::max(0, std::min(lrX1, lrX2) - std::max(ulX1, ulX2)) * std::max(0, std::min(lrY1, lrY2) - std::max(ulY1, ulY2));

  int volumei = (lrX1 - ulX1) * (lrY1 - ulY1);
  int volumej = (lrX2 - ulX2) * (lrY2 - ulY2);

  int unite = volumei + volumej - intersection;

  return intersection / (float)unite;
}

void RobotClassifier::getUpperImageCoordinates(const RobotEstimate& re, int& upperLeftX, int& upperLeftY, int& lowerRightX, int& lowerRightY)
{
  if (re.fromUpperImage)
  {
    upperLeftX = re.imageUpperLeft.x(), upperLeftY = re.imageUpperLeft.y(), lowerRightX = re.imageLowerRight.x(), lowerRightY = re.imageLowerRight.y();
  }
  float xFraction = static_cast<float>(theImageUpper.width) / theImage.width;
  float yFraction = static_cast<float>(theImageUpper.height) / theImage.height;
  upperLeftX = static_cast<int>(round(re.imageUpperLeft.x() * xFraction));
  upperLeftY = std::max(0, static_cast<int>(round(theImageUpper.height + re.imageUpperLeft.y() * yFraction)));
  lowerRightX = static_cast<int>(round(re.imageLowerRight.x() * xFraction));
  lowerRightY = static_cast<int>(round(theImageUpper.height + re.imageLowerRight.y() * yFraction));
}

MAKE_MODULE(RobotClassifier, perception);
