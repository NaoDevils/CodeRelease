#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Module/ModuleManager.h"
#include "Modules/MotionControl/DortmundWalkingEngine/CSConverter2019.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/JointError.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Tools/RingBufferWithSum.h"
#include <algorithm>

constexpr unsigned IMU_BUFFER_LENGTH = static_cast<unsigned>(0.5 * 83);

MODULE(AutoCalibrator,
  REQUIRES(FrameInfo),
  REQUIRES(BehaviorData),
  REQUIRES(RobotPose),
  REQUIRES(JoinedIMUData),
  REQUIRES(KeySymbols),
  REQUIRES(RawGameInfo),
  USES(JointError),
  PROVIDES(WalkCalibration),
  PROVIDES(MotionRequest),
  PROVIDES(HeadControlRequest),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (Angle)(0.01_deg) gyroMaxVariance,
    (float)(0.005) targetBodyGravityMaxDiff,
    (float)(0.03f) bodyGravityCorrectionP,
    (float)(0.90f) minValidityForRobotPose,
    (bool)(true) useFieldInclinationFromFile,
    (bool)(false) mirrorFieldInclination
  )
);

class AutoCalibrator : public AutoCalibratorBase
{
public:
  AutoCalibrator(void);

private:
  ENUM(CalibrationState,
    inactive,
    bodyAngle,
    walk
  );

  ENUM(BodyAngleCalibrationState,
    adjustingBodyAngle,
    fieldLeveling
  );

  ENUM(FieldLevelingState,
    waitingForManualPositioning,
    adjustingFootPitch,
    waitingForGoodLocalization
  );

  ENUM(WalkCalibrationState,
    adjustingSensorControl,
    adjustingFixedCoMOffsets
  );

  void update(WalkCalibration& walkCalibration);
  void update(MotionRequest& motionRequest);
  void update(HeadControlRequest& headControlRequest);
  void execute(tf::Subflow&);
  void reset();
  void saveCalibration();
  void loadCalibration();
  void calibrateBodyGravity();
  void calibrateWalk();

  WalkCalibration localWalkCalibration;
  Vector2a fieldInclinationFromConfig = Vector2a::Zero();
  CSConverter2019Base::Parameters csConverterParams;
  BehaviorData::BehaviorState lastBehaviorState = BehaviorData::BehaviorState::game;

  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersX;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersY;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersZ;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersX;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersY;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersZ;
  std::array<RingBufferWithSum<Angle, 2 * IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> angleDataBuffersX;
  std::array<RingBufferWithSum<Angle, 2 * IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> angleDataBuffersY;

  std::vector<std::tuple<Vector2f, Vector2a>> fieldLevelingPoints;

  Vector2a lastFieldInclination = Vector2a::Zero();

  bool pressedHeadFront = false;
  bool opponentHalf = false;
  unsigned timeCalibrationStart = 0;
  unsigned timeInternalStateSwitch = 0;
  CalibrationState calibrationState = CalibrationState::inactive;
  CalibrationState lastCalibrationState = CalibrationState::inactive;
  BodyAngleCalibrationState bodyAngleCalibrationState = BodyAngleCalibrationState::adjustingBodyAngle;
  FieldLevelingState fieldLevelingState = FieldLevelingState::waitingForManualPositioning;
  WalkCalibrationState walkCalibrationState = WalkCalibrationState::adjustingSensorControl;
  bool calibrationLoaded = false;
  ModuleManager::Configuration oldConfig; /**< Needed to save the previous Module Configuration while switching for calibration. */
  bool moduleConfigSwitched = false;
  bool manualCalibrationTrigger = false;
};
