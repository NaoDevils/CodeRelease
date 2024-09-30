/**
 * @file RefereeKeypointsDetector.h
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include <string>
#include "Platform/File.h"

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/RefereeKeypoints.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"

#include "Modules/Perception/TFlite.h"

MODULE(RefereeKeypointsDetector,
  REQUIRES(FrameInfo),
  REQUIRES(ImageUpper),
  REQUIRES(VisualRefereeBehaviorSymbols),
  PROVIDES(RefereeKeypoints),

  LOADS_PARAMETERS(,
    (std::string)("tflite/pose/pose.tflite") modelName,
    (float)(0.2f) minConfidence,
    (float)(0.9f) smoothKeypointsFactor,
    (int)(1) numThreads,
    (bool)(true) useCOCO
  )
);

class RefereeKeypointsDetector : public RefereeKeypointsDetectorBase
{
public:
  std::unique_ptr<tflite::Interpreter> interpreter;
  void update(RefereeKeypoints& refereeKeypoints);
  void reorder(std::array<Vector2f, 13>& v, std::array<size_t, 13> const& order);

private:
  bool initialized = false;
  TfLiteType input_type;
  int input_height;
  int input_width;
  int input_channels;

  Eigen::MatrixXi rangeWeightX;
  Eigen::MatrixXi rangeWeightY;

  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;

  std::vector<float> input_patch_float;
  std::vector<float> result_vector;

  void initClassifier();
  float getElement(std::vector<int>& index, const std::vector<float>& vec, const std::vector<int>& shape);

  template <typename T> T* applyPatchOnImage(int input_tensor, int xmin, int ymin, int width, int height);
};
