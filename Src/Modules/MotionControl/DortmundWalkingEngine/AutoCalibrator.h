#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Module/ModuleManager.h"
#include "Modules/MotionControl/DortmundWalkingEngine/CSConverter2019.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/RingBufferWithSum.h"
#include <algorithm>

constexpr unsigned IMU_BUFFER_LENGTH = static_cast<unsigned>(0.5 * 83);

MODULE(AutoCalibrator,
  REQUIRES(FrameInfo),
  USES(WalkingInfo),
  USES(SpeedInfo),
  REQUIRES(BehaviorData),
  REQUIRES(WalkingEngineParams),
  REQUIRES(RobotDimensions),
  REQUIRES(JointSensorData),
  REQUIRES(MassCalibration),
  REQUIRES(RobotModel),
  REQUIRES(ZMPModel),
  REQUIRES(JoinedIMUData),
  USES(JointRequest),
  USES(KinematicOutput),
  PROVIDES(WalkCalibration),
  PROVIDES(MotionRequest),
  PROVIDES(HeadControlRequest),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (Angle)(0.01_deg) gyroMaxVariance,
    (float)(0.005) targetBodyGravityMaxDiff,
    (float)(0.5f) bodyAngleCorrectionP,
    (float)(0.1f) targetBodyCoMXMaxDiff, // in mm
    (float)(0.f) targetBodyCoMX, // in mm
    (float)(0.5f) bodyCoMCorrectionP,
    (float)(0.5f) bodyCoMCorrectionD
  )
);

class AutoCalibrator : public AutoCalibratorBase
{
public:
  AutoCalibrator(void);
  ~AutoCalibrator(void);

private:
  ENUM(CalibrationState,
    inactive,
    bodyAngle,
    walk
  );

  ENUM(BodyAngleCalibrationState,
    adjustingBodyAngle,
    checkBodyAngleWithSensorControl,
    adjustingCoM,
    checkCoMWithSensorControl
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
  CSConverter2019Base::Parameters csConverterParams;

  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersX;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersY;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersZ;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersX;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersY;
  std::array<RingBufferWithSum<float, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> accDataBuffersZ;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> angleDataBuffersX;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> angleDataBuffersY;

  Vector3f lastCoMError = Vector3f::Zero();

  unsigned timeCalibrationStart = 0;
  unsigned timeInternalStateSwitch = 0;
  CalibrationState calibrationState = CalibrationState::inactive;
  CalibrationState lastCalibrationState = CalibrationState::inactive;
  BodyAngleCalibrationState bodyAngleCalibrationState = BodyAngleCalibrationState::adjustingBodyAngle;
  WalkCalibrationState walkCalibrationState = WalkCalibrationState::adjustingSensorControl;
  bool calibrationLoaded = false;
  ModuleManager::Configuration oldConfig; /**< Needed to save the previous Module Configuration while switching for calibration. */
  bool moduleConfigSwitched = false;
  bool manualCalibrationTrigger = false;
};
