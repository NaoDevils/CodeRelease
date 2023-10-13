#pragma once

#include <optional>
#include <functional>

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"

#include "Tools/Debugging/Debugging.h"

#include <cstdio>
#include "Modules/Perception/TFlite.h"
#include "Modules/Perception/YoloRobotDetector.h"

MODULE(RobotClassifier,
  REQUIRES(FrameInfo),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(RobotsHypothesesYolo),
  REQUIRES(RobotsHypothesesYoloUpper),

  PROVIDES(RobotsPerceptClassified),
  PROVIDES(ProcessedRobotsHypotheses),
  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
    (float)(0.3f) classifierThreshold,
    (float)(0.3f) classifierThresholdLower,
    (float)(1) classifierMargin,
    (int)(10) maxClassficationsLower,
    (int)(3) maxClassficationsTotal,
    (float)(0.1f) nonMaximumSuppressionThreshold,
    (bool)(true) useBboxPrediction,
    (float)(0.5) bboxMargin,
    (bool)(true) interpolatePredictedBbox,
    (float)(1.f) interpolatePredictedBboxFactor,
    (bool)(false) useGeometricHeight,
    (bool)(true) interpolateGeometricBbox,
    (float)(1.f) interpolateGeometricBboxFactor
  )
);


class RobotClassifier : public RobotClassifierBase
{
public:
  RobotClassifier();

  void update(RobotsPerceptClassified& theRobotsPerceptClassified);
  void update(ProcessedRobotsHypotheses& theProcessedRobotsHypotheses);

  std::unique_ptr<tflite::Interpreter> classificationModelInterpreter;
  std::unique_ptr<tflite::Interpreter> bboxCorrectionModelInterpreter;

private:
  std::unique_ptr<tflite::FlatBufferModel> classificationModel;
  std::unique_ptr<tflite::FlatBufferModel> bboxCorrectionModel;

  tflite::ops::builtin::BuiltinOpResolver resolver;

  void initModel(std::string path, std::unique_ptr<tflite::Interpreter>& interpreter, std::unique_ptr<tflite::FlatBufferModel>& model);

  RobotsPerceptClassified localRobotsPerceptClassified;
  ProcessedRobotsHypotheses localRobotsHypotheses;

  size_t sumOfRobotsHypotheses;
  size_t processedRobotsHypotheses;

  void execute(tf::Subflow&);

  /* Run the classifier net on an estimate and return true, if classified as correct detection.
   * Update the estimates confidence
   */
  void updateEstimate(const Image&, RobotEstimate&);
  bool classifyEstimate(const Image&, RobotEstimate&);
  void correctBbox(const Image&, RobotEstimate&);
  void predictBbox(const Image& image, RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr);
  void getGeometricBbox(RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr);
  void interpolateBbox(RobotEstimate& re, int& xUl, int& yUl, int& xLr, int& yLr, float factor, bool keepLower);
  bool filterNms(RobotEstimate&, bool respectValidity);
  float iou(const RobotEstimate& re1, const RobotEstimate& re2);
  void getUpperImageCoordinates(const RobotEstimate&, int&, int&, int&, int&);
};
