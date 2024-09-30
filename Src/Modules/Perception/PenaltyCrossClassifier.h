#pragma once

#include <optional>
#include <functional>

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/PenaltyCrossHypotheses.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

#include "Tools/Debugging/Debugging.h"

#include <cstdio>
#include "Modules/Perception/TFlite.h"

#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    OUTPUT_TEXT("Error at " << __FILE__ << ":" << __LINE__); \
  }

MODULE(PenaltyCrossClassifier,
  REQUIRES(ImageUpper),
  REQUIRES(Image),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(PrePenaltyCrossHypothesesYolo),
  REQUIRES(PrePenaltyCrossHypothesesScanlines),

  PROVIDES(PenaltyCrossHypotheses),
  PROVIDES(PenaltyCrossPercept),
  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
    (float)(0.5f) penaltyCrossThreshold,
    (float)(1.5f) zoomOutFactor,
    (bool)(true) stopOnFirstDetection,
    (bool)(true) pauseLogOnDetection,
    (std::string) modelName,
    (int)(20) maxNumberOfHypotheses
  )
);


class PenaltyCrossClassifier : public PenaltyCrossClassifierBase
{
public:
  PenaltyCrossClassifier();

  void update(PenaltyCrossPercept& thePenaltyCrossPercept);
  void update(PenaltyCrossHypotheses& thePenaltyCrossHypotheses);

  std::unique_ptr<tflite::Interpreter> interpreter;

private:
  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  PenaltyCrossHypotheses localPenaltyCrossHypotheses;
  PenaltyCrossPercept localPenaltyCrossPercept;

  size_t sumOfPenaltyCrossHypotheses;
  size_t processedPenaltyCrossHypotheses;

  void execute(tf::Subflow&);
  void initClassifier();
  bool checkPenaltyCrosses(PenaltyCrossPercept& thePenaltyCrossPercept, const std::vector<PenaltyCross> penaltyCrosses, PenaltyCrossPercept::DetectionType detectionType);
  void checkPenaltyCross(PenaltyCross& penaltyCross);
};
