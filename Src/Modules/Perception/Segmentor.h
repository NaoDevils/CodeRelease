#pragma once

#include <optional>
#include <functional>

#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Sensing/FallDownState.h"
#include <stack>

#include "Tools/Debugging/Debugging.h"

#include <cstdio>
#include "Modules/Perception/TFlite.h"

#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x))                                                  \
  {                                                          \
    OUTPUT_TEXT("Error at " << __FILE__ << ":" << __LINE__); \
  }

//STREAMABLE(MinPercentages,, // the three values are corresponding to h,s,l respectively
//  (std::vector<float, 4>) vals
//);

MODULE(Segmentor,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(BodyContour),
  REQUIRES(FallDownState),

  PROVIDES(BallHypothesesSegmentor),
  PROVIDES(RobotsHypothesesSegmentor),
  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
    (std::string)("segmentor_d0.987.tflite") modelName,
    (int)(1) minNeighbors,
    (bool)(true) useMajorityClass,
    (std::vector<float>)() minPercentages,
    (int)(3) maxComponents,
    (float)(0.2) robotSizeHighValidity,
    (int)(250) distanceEstimateMm
  )
);

// has to be in the same order as the classes predicted by the model
enum SegmentationClass
{
  Background,
  Robot,
  Self,
  Ball,
  Last = Ball,
  Line,
  Field
};

class Segmentor : public SegmentorBase
{
public:
  Segmentor();

private:
  std::unique_ptr<tflite::FlatBufferModel> model;
  std::unique_ptr<tflite::Interpreter> interpreter;

  tflite::ops::builtin::BuiltinOpResolver resolver;
  int mapW;
  int mapH;
  float sizeFactorX;
  float sizeFactorY;
  Vector2f sizeFactor;

  void declareDebug();
  void initModel(std::string path, std::unique_ptr<tflite::Interpreter>& interpreter, std::unique_ptr<tflite::FlatBufferModel>& model);
  void execute(tf::Subflow&);
  void update(BallHypothesesSegmentor& theBallHypothesesSegmentor);
  void update(RobotsHypothesesSegmentor& theRobotsHypothesesSegmentor);
  void zeroMaps();
  void interpretModel();
  void smoothMaps();
  int getNeighbors(Eigen::MatrixXi& map, int w, int h, int max) const;
  SegmentationClass getMajorityClass(int w, int h);
  void drawMaps();
  std::vector<Vector2i> grahamScan(Eigen::MatrixXi&, bool);
  std::tuple<std::vector<Vector2i>, int> computeConvexHull(Eigen::MatrixXi&);
  float dfs(const Eigen::MatrixXi& matrix, Eigen::MatrixXi& component, int x, int y, Eigen::MatrixXi& visited);
  std::vector<std::tuple<Eigen::MatrixXi, float>> extractComponents(const Eigen::MatrixXi& matrix, float minSize);
  Vector2f toScreenCoordinates(Vector2i&);

  std::map<SegmentationClass, ColorRGBA> classColorMap{{SegmentationClass::Line, ColorRGBA::white},
      {SegmentationClass::Self, ColorRGBA::red},
      {SegmentationClass::Ball, ColorRGBA::yellow},
      {SegmentationClass::Field, ColorRGBA::green},
      {SegmentationClass::Robot, ColorRGBA::blue},
      {SegmentationClass::Background, ColorRGBA(0, 0, 0, 0)}};

  std::map<SegmentationClass, Eigen::MatrixXi> segmentationMaps{};
};
