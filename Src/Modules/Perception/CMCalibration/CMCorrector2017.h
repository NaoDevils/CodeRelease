#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/DebugImages.h"

MODULE(CMCorrector2017,
{ ,
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  USES(CLIPFieldLinesPercept),
  REQUIRES(JointSensorData),
  REQUIRES(FrameInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(TorsoMatrix),
  REQUIRES(KeyStates),
  PROVIDES(CameraCalibration),
  PROVIDES(HeadControlRequest),
  PROVIDES(MotionRequest),
  PROVIDES(RobotPose),
  PROVIDES(RobotInfo),
  LOADS_PARAMETERS(
  {,
    (float) wAngleRelative,
    (float) wDistRobot,
    (float) farLineDistance,
    (float[6]) errorLimits,
    (unsigned int) minSamples,
    (int[2]) minVLineLength,
    (int[2]) minHLineLength,
    (int) waitTimeAfterHeadMove,
  }),
});

class CMCorrector2017 : public CMCorrector2017Base
{
public:
  CMCorrector2017();

  ENUM(CalibrationState,
  {,
    inactive,
    standUp,
    capture,
    calibrate,
    finished,
  });

  ENUM(HeadPosition,
  {,
    leftUpper,
    centerUpper,
    rightUpper,
    leftLower,
    centerLower,
    rightLower,
  });

private:
  void update(CameraCalibration &cameraCalibration);
  void update(HeadControlRequest &headControlRequest);
  void update(MotionRequest &motionRequest);
  void update(RobotPose &robotPose);
  void update(RobotInfo &robotInfo);

  void debug();
  
  void start(HeadPosition position = leftUpper);
  void stop();
  void load();
  void save();

  void runStateMachine();

  bool captureData();

  void printCurrentError(HeadPosition position);

  bool optimizeUpper();
  bool optimizeLower();
  Vector2a optimizeFunction(const Vector2a& min, const Vector2a& max, const std::vector<Vector2a>& stepSizes, float& bestValue, std::function<float(const Vector2a&)> func);

  std::vector<Geometry::Line> linesFromImageToRobot(const std::vector<CLIPFieldLinesPercept::FieldLine>& lines, HeadPosition position, const CameraCalibration& calibration);
  
  float calcError(HeadPosition position, const CameraCalibration& calibration);
  bool verifyError(float bestError, HeadPosition position);

  float getTotalError(const std::vector<Geometry::Line>& fieldLines, const std::vector<Geometry::Line>& verticalFieldLines = std::vector<Geometry::Line>());

  std::vector<CLIPFieldLinesPercept::FieldLine> horizontalLines[6];
  std::vector<CLIPFieldLinesPercept::FieldLine> verticalLines[2];

  Vector2f headPosition[6];
  TorsoMatrix torsoMatrix[6];

  CalibrationState state;
  HeadPosition currentPosition;
  unsigned stateBeginTimeStamp;

  CameraCalibration localCalibration;
};