#pragma once

#include <optional>
#include <functional>

#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"

#include "Tools/Debugging/Debugging.h"

#include <cstdio>
#include "Modules/Perception/TFlite.h"
#include "Modules/Perception/YoloRobotDetector.h"

constexpr float ROBOT_HEIGHT = 580.f;

MODULE(RobotClassifier,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(RobotsHypothesesYolo),
  REQUIRES(RobotsHypothesesYoloUpper),
  REQUIRES(RobotsHypothesesSegmentor),

  PROVIDES(RobotsPerceptClassified),
  PROVIDES(ProcessedRobotsHypotheses),
  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
    (float)(0.3f) classifierThreshold,
    (float)(0.1f) classifierThresholdLower,
    (float)(0.3f) classifierThresholdRecall,
    (float)(1) classifierMargin,
    (int)(10) maxClassficationsLower,
    (int)(3) maxClassficationsTotal,
    (float)(0.1f) nonMaximumSuppressionThreshold,
    (float)(0.5f) maxTrackingDist,
    (bool)(true) useBboxPrediction,
    (float)(0.5) bboxMargin,
    (bool)(true) interpolatePredictedBbox,
    (float)(1.f) interpolatePredictedBboxFactor,
    (int)(20) maxBboxCorrections,
    (bool)(false) useGeometricHeight,
    (bool)(true) interpolateGeometricBbox,
    (float)(1.f) interpolateGeometricBboxFactor,
    (float)(1.f) oldInterpolationFactor,
    (bool)(true) enableTracking,
    (std::string)("robot_classifier_rec_0.88_thr_0.70541.tflite") classifierName,
    (std::string)("bbox_correctifier_y_rel_0.057.tflite") bboxCorrectifierName 
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
  std::vector<RobotEstimate> lastValidEstimates;
  std::vector<RobotEstimate> declinedButPossibleRobots;

  size_t sumOfRobotsHypotheses;
  size_t processedRobotsHypotheses;
  int doneBboxCorrections;

  void execute(tf::Subflow&);


  /* Run the classifier net on an estimate and return true, if classified as correct detection.
   * Update the estimates confidence
   */
  void classifyHypotheses();
  void updateEstimate(RobotEstimate&);
  bool classifyEstimate(RobotEstimate&);
  void correctBbox(RobotEstimate&);
  void calculateDistance(RobotEstimate& re);
  void predictBbox(RobotEstimate& re, Vector2i& ul, Vector2i& lr);
  void getGeometricBbox(RobotEstimate& re, Vector2i& ul, Vector2i& lr);
  void interpolateBbox(RobotEstimate& re, Vector2i& ul, Vector2i& lr, float factor, bool keepLower);
  void getArea(RobotEstimate& re, const float margin, Vector2i& upperLeftArea, Vector2i& sizeArea);
  bool filterNms(RobotEstimate&, bool respectValidity);
  bool printIou(RobotEstimate& re);
  void postProcess();
  void removeInvalidatedRobots();
  void interpolateTrackedBboxes();
  void sortLocalClassificationsByY();
  void doRemainingClassifications();
  std::optional<RobotEstimate> findClosestOldRobot(const RobotEstimate& re);
  void acceptRecallRobots();
  float iou(const RobotEstimate& re1, const RobotEstimate& re2);
  void printDeclinedRobot(const RobotEstimate& re, const std::string reason);
};
