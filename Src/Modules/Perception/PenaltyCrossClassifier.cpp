#include "PenaltyCrossClassifier.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/File.h"

PenaltyCrossClassifier::PenaltyCrossClassifier()
{
  initClassifier();
}

void PenaltyCrossClassifier::update(PenaltyCrossPercept& penaltyCrossPercept)
{
  DECLARE_DEBUG_DRAWING("module:PenaltyCrossClassifier:Image:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:PenaltyCrossClassifier:Image:Lower", "drawingOnImage");
  DECLARE_PLOT("module:PenaltyCrossClassifier:sumOfPenaltyCrossHypotheses");
  DECLARE_PLOT("module:PenaltyCrossClassifier:processedPenaltyCrossHypotheses");
  std::swap(penaltyCrossPercept, localPenaltyCrossPercept);
}

void PenaltyCrossClassifier::update(PenaltyCrossHypotheses& penaltyCrossHypotheses)
{
  std::swap(penaltyCrossHypotheses, localPenaltyCrossHypotheses);
}

void PenaltyCrossClassifier::execute(tf::Subflow&)
{
  DEBUG_RESPONSE_ONCE("module:PenaltyCrossClassifier:initClassifier") initClassifier();
  sumOfPenaltyCrossHypotheses = 0;
  processedPenaltyCrossHypotheses = 0;

  localPenaltyCrossPercept.reset();
  localPenaltyCrossHypotheses.penaltyCrosses.clear();
  localPenaltyCrossHypotheses.penaltyCrossesUpper.clear();

  bool shouldReturn = false;
  STOPWATCH("PenaltyCrossPercept-update(Lower)")
  {
    sumOfPenaltyCrossHypotheses += thePrePenaltyCrossHypothesesYolo.penaltyCrosses.size();
    if (!shouldReturn && checkPenaltyCrosses(localPenaltyCrossPercept, thePrePenaltyCrossHypothesesYolo.penaltyCrosses, PenaltyCrossPercept::yolo))
      if (stopOnFirstDetection)
        shouldReturn = true;
    sumOfPenaltyCrossHypotheses += thePrePenaltyCrossHypothesesScanlines.penaltyCrosses.size();
    if (!shouldReturn && checkPenaltyCrosses(localPenaltyCrossPercept, thePrePenaltyCrossHypothesesScanlines.penaltyCrosses, PenaltyCrossPercept::scanlines))
      if (stopOnFirstDetection)
        shouldReturn = true;
  }
  STOPWATCH("PenaltyCrossPercept-update(Upper)")
  {
    sumOfPenaltyCrossHypotheses += thePrePenaltyCrossHypothesesYolo.penaltyCrossesUpper.size();
    if (!shouldReturn && checkPenaltyCrosses(localPenaltyCrossPercept, thePrePenaltyCrossHypothesesYolo.penaltyCrossesUpper, PenaltyCrossPercept::yolo))
      if (stopOnFirstDetection)
        shouldReturn = true;
    sumOfPenaltyCrossHypotheses += thePrePenaltyCrossHypothesesScanlines.penaltyCrossesUpper.size();
    if (!shouldReturn && checkPenaltyCrosses(localPenaltyCrossPercept, thePrePenaltyCrossHypothesesScanlines.penaltyCrossesUpper, PenaltyCrossPercept::scanlines))
      if (stopOnFirstDetection)
        shouldReturn = true;
  }
  PLOT("module:PenaltyCrossClassifier:sumOfPenaltyCrossHypotheses", sumOfPenaltyCrossHypotheses);
  PLOT("module:PenaltyCrossClassifier:processedPenaltyCrossHypotheses", processedPenaltyCrossHypotheses);
}

void PenaltyCrossClassifier::initClassifier()
{
  std::string filename = std::string(File::getBHDir()) + "/Config/" + std::string(modelName);

  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());
  TFLITE_MINIMAL_CHECK(model != nullptr);

  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  TFLITE_MINIMAL_CHECK(interpreter != nullptr);
  interpreter->SetNumThreads(1);

  // Allocate memory for the tensors
  interpreter->AllocateTensors();

  // Check interpreter state
  //tflite::PrintInterpreterState(class_interpreter.get());
}

bool PenaltyCrossClassifier::checkPenaltyCrosses(PenaltyCrossPercept& thePenaltyCrossPercept, const std::vector<PenaltyCross> penaltyCrosses, PenaltyCrossPercept::DetectionType detectionType)
{
  for (const PenaltyCross& pc : penaltyCrosses)
  {
    bool pcFound = false;
    STOPWATCH("PenaltyCrossClassifier-checkPenaltyCross")
    {
      PenaltyCross pc_classified(pc);
      checkPenaltyCross(pc_classified);
      if (pc.fromUpper)
        localPenaltyCrossHypotheses.penaltyCrossesUpper.push_back(pc_classified);
      else
        localPenaltyCrossHypotheses.penaltyCrosses.push_back(pc_classified);

      if (pc_classified.validity > penaltyCrossThreshold)
      {
        const CameraMatrix& cameraMatrix = pc.fromUpper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
        const CameraInfo& cameraInfo = pc.fromUpper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
        Vector2f pImage(pc.positionInImage);
        Vector2f pField;
        if (Transformation::imageToRobot(pImage, cameraMatrix, cameraInfo, pField))
        {
          if (pauseLogOnDetection)
            OUTPUT(idConsole, text, "log pause");
          thePenaltyCrossPercept.penaltyCrossWasSeen = true;
          thePenaltyCrossPercept.detectionType = detectionType;
          thePenaltyCrossPercept.pointInImage = pImage.cast<int>();
          thePenaltyCrossPercept.pointOnField = pField.cast<int>();
          thePenaltyCrossPercept.fromUpper = pc.fromUpper;
          if (stopOnFirstDetection)
            pcFound = true;
        }
      }
    }
    if (pcFound)
      return true;
  }
  return false;
}

void PenaltyCrossClassifier::checkPenaltyCross(PenaltyCross& penaltyCross)
{
  if (processedPenaltyCrossHypotheses >= static_cast<size_t>(maxNumberOfHypotheses))
  {
    penaltyCross.validity = 0.f;
    return;
  }

  char buffer[10];
  const Image& image = penaltyCross.fromUpper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = penaltyCross.fromUpper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = penaltyCross.fromUpper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;

  Vector2i centerPosInImage = penaltyCross.positionInImage.cast<int>();
  float diameter = Geometry::calculateLineSizePrecise(centerPosInImage, cameraMatrix, cameraInfo, theFieldDimensions.penaltyMarkSize) * zoomOutFactor;
  bool projected = penaltyCross.fromUpper ? theImageUpper.projectIntoImage(centerPosInImage, diameter / 2.f) : theImage.projectIntoImage(centerPosInImage, diameter / 2.f);
  if (!projected)
  {
    penaltyCross.validity = 0.f;
    return;
  }

  const Vector2f patchSize(diameter, diameter);
  const Vector2i patchPos = (centerPosInImage.cast<float>() - (patchSize / 2.f) + Vector2f::Constant(0.5f)).cast<int>();

  float* input = interpreter->typed_tensor<float>(interpreter->inputs()[0]);
  STOPWATCH("PenaltyCrossClassifier-checkPenaltyCrosses-copyAndResize")
  {
    image.copyAndResizeArea(patchPos, patchSize.cast<int>(), {PENALTY_CROSS_SIZE, PENALTY_CROSS_SIZE}, input);
  }

  float* output = interpreter->typed_tensor<float>(interpreter->outputs()[0]);
  STOPWATCH("PenaltyCrossClassifier-checkPenaltyCrosses-runNet")
  {
    if (interpreter->Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
      penaltyCross.validity = 0.f;
      return;
    }
  }

  if (penaltyCross.fromUpper)
  {
    COMPLEX_DRAWING("module:PenaltyCrossClassifier:Image:Upper")
    {
      RECTANGLE("module:PenaltyCrossClassifier:Image:Upper", patchPos.x(), patchPos.y(), patchPos.x() + patchSize.x(), patchPos.y() + patchSize.y(), 3, Drawings::solidPen, ColorRGBA::black);
      sprintf(buffer, "%.1f", *output * 100.f);
      DRAWTEXT("module:PenaltyCrossClassifier:Image:Upper", patchPos.x(), centerPosInImage.y() - (patchSize.y() * 0.15f), static_cast<int>(patchSize.x() * 0.25f), ColorRGBA::black, buffer << "%");
    }
  }
  else
  {
    COMPLEX_DRAWING("module:PenaltyCrossClassifier:Image:Lower")
    {
      RECTANGLE("module:PenaltyCrossClassifier:Image:Lower", patchPos.x(), patchPos.y(), patchPos.x() + patchSize.x(), patchPos.y() + patchSize.y(), 3, Drawings::solidPen, ColorRGBA::black);
      sprintf(buffer, "%.1f", *output * 100.f);
      DRAWTEXT("module:PenaltyCrossClassifier:Image:Lower", patchPos.x(), centerPosInImage.y() - (patchSize.y() * 0.15f), static_cast<int>(patchSize.x() * 0.25f), ColorRGBA::black, buffer << "%");
    }
  }
  penaltyCross.validity = *output;
  penaltyCross.patch = std::vector<float>{input, input + PENALTY_CROSS_SIZE * PENALTY_CROSS_SIZE * 3};
  processedPenaltyCrossHypotheses++;
}

MAKE_MODULE(PenaltyCrossClassifier, perception);
