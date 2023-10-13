/**
  * YoloRobotDetector to detect robots in complete images
  * @created <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
  * @created <a href="mailto:maximilian.otten@tu-dortmund.de">Maximilian Otten</a>
 **/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/YoloInput.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/PenaltyCrossHypotheses.h"
#include "Representations/Configuration/FieldDimensions.h"

#include <cstdio>
#include "Modules/Perception/TFlite.h"

#define NO_HORIZON
#define BOTTOM_AS_CENTER

#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    OUTPUT_TEXT("Error at " << __FILE__ << ":" << __LINE__); \
  }

STREAMABLE(YUVColor,,
  (unsigned char) y, 
  (unsigned char) cb, 
  (unsigned char) cr
);

enum YoloClasses
{
  Robot,
  Ball,
  Penaltycross,
};

MODULE(YoloRobotDetector,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FallDownState),
  REQUIRES(KeySymbols),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  //USES(RobotPose),

  PROVIDES_CONCURRENT_WITHOUT_MODIFY(YoloInputUpper),
  PROVIDES_CONCURRENT_WITHOUT_MODIFY(YoloInput),

  PROVIDES_CONCURRENT(RobotsHypothesesYolo),
  PROVIDES_CONCURRENT(RobotsHypothesesYoloUpper),
  PROVIDES_CONCURRENT(BallHypothesesYolo),
  PROVIDES_CONCURRENT(PrePenaltyCrossHypothesesYolo),
  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
    (float)(0.9f) lowerRobotThreshold,
    (float)(0.01f) lowerBallThreshold,
    (float)(0.5f) lowerPenaltyCrossThreshold,
    (float)(0.3f) lowerNMSThreshold,
    (bool)(true) skipRobotNMS,
    (bool)(false) useLowerYoloHeight,
    (float)(20.f) xOffset,
    (bool)(true) useXOffset,

    (float)(0.3f) upperRobotThreshold,
    (float)(0.01f) upperBallThreshold,
    (float)(0.0f) upperPenaltyCrossThreshold,
    (float)(0.3f) upperNMSThreshold,
    (bool)(false) useUpperYoloHeight,

    (bool)(true) useTFlite
  )
);

struct YoloRegionBox
{
  float x, y, w, h, conf;
  int grid_x, grid_y;
};

struct YoloDetection
{
  YoloRegionBox bbox;
  int classes;
  float prob;
  int sortClass;
  bool operator<(const YoloDetection& yd) const { return (prob > yd.prob); }
};

struct YoloParameter
{
  unsigned int input_height;
  unsigned int input_width;
  unsigned int input_channel;
  unsigned int output_height;
  unsigned int output_width;
  unsigned int num_of_boxes;
  unsigned int num_of_classes;
  unsigned int num_of_coords;
  std::vector<float> anchors;
};

class YoloResult
{
public:
  unsigned output_height, output_width, num_of_boxes, num_of_coords, num_of_classes;
  std::vector<float> result;

  YoloResult(unsigned output_height, unsigned output_width, unsigned num_of_boxes, unsigned num_of_coords, unsigned num_of_classes)
      : output_height(output_height), output_width(output_width), num_of_boxes(num_of_boxes), num_of_coords(num_of_coords), num_of_classes(num_of_classes),
        result(output_height * output_width * (num_of_boxes * (num_of_coords + 1 + num_of_classes)))
  {
  }

  // 3D to 1D mapping accessor
  float& operator()(unsigned row, unsigned column, unsigned attr)
  {
    int index = row * output_width * (num_of_boxes * (num_of_coords + 1 + num_of_classes)) + column * (num_of_boxes * (num_of_coords + 1 + num_of_classes)) + attr;
    return result[index];
  }
};

class YoloRobotDetector : public YoloRobotDetectorBase
{
public:
  DECLARE_DEBUG_IMAGE(YoloDebugImage);
  DECLARE_DEBUG_IMAGE(YoloDebugImageUpper);
  YoloRobotDetector();

  void execute(tf::Subflow& subflow);
  void update(RobotsHypothesesYolo& theRobotsHypothesesYolo);
  void update(RobotsHypothesesYoloUpper& theRobotsHypothesesYoloUpper);
  void update(YoloInputUpper& theYoloInputUpper);
  void update(YoloInput& theYoloInput);
  void update(BallHypothesesYolo& theBallHypothesesYolo);
  void update(PrePenaltyCrossHypothesesYolo& thePrePenaltyCrossHypothesesYolo);
  void execute(const bool& upper);
  void reset(const bool& upper);

  std::unique_ptr<tflite::Interpreter> interpreter;

private:
  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  int input_tensor, output_tensor;

  unsigned timeStamp, timeStampUpper; // used to make sure that images are only processed once

  YoloParameter yoloParameter;
  YoloParameter yoloParameterUpper;

  RobotsHypothesesYolo localRobotsPerceptYolo;
  RobotsHypothesesYoloUpper localRobotsPerceptYoloUpper;

  BallHypothesesYolo localBallHypotheses;
  PrePenaltyCrossHypothesesYolo localPenaltyCrossHypotheses;

  std::vector<YoloDetection> detectionVectorUpper;
  std::vector<YoloDetection> detectionVector;

  std::vector<float> inputVectorUpper;
  std::vector<float> inputVector;
#ifdef NO_HORIZON
  std::vector<int> yIdxs;
#endif

  float iou(YoloRegionBox& box1, YoloRegionBox& box2, int heigth, int width);

  /* Fill a single box from network output */
  void getRegionBox(YoloRegionBox& b, YoloResult& yoloResult, int numOfBox, int col, int row, const YoloParameter& localParameter);

  /* If output w/h ration is not equal to image w/h ratio, correct this */
  void correctRegionBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, const bool& upper);

  /* Collect all boxes that have high enough confidence score */
  void generateNetworkBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, YoloResult& yoloResult, const bool& upper);

  void addObstacleFromBumpers();

  void calcImageCoords(RobotEstimate& robot, bool upper);
};
