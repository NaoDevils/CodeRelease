/**
* @file SimpleFallDownStateDetector.h
*
* This file declares a module that provides information about the current state of the robot's body.
*
* @author <a href="mailto:dominik.braemer@tu.dortmund.de">Dominik Brämer</a>
*/

#pragma once

#include <queue>
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"


MODULE(SimpleFallDownStateDetector,
  USES(MotionInfo),
  USES(BehaviorData),
  REQUIRES(GameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(JoinedIMUData),
  REQUIRES(GroundContactState),
  REQUIRES(WalkingEngineParams), 
  PROVIDES(FallDownState),
  LOADS_PARAMETERS(,
    (Angle)(24_deg) fallDownAngleFront,
    (Angle)(24_deg) fallDownAngleSide,
    (Angle)(-17_deg) fallDownAngleBack,
    (Angle)(10_deg) standingUpTolerance,
    (float)(1.f) secondsOnGround,
    (float)(1.f) secondsUntilStandUp,
    (float)(2.f) secondsUntilHeldOnShoulder,
    (bool)(true) enableFlyingDetection,
    (Angle)(5_deg) minHeldOnShoulderAngle,
    (Angle)(30_deg) maxHeldOnShoulderAngle,
    (Angle)(10_deg) heldOnShoulderTolerance,
    (float)(1.0f) fsrThreshold,
    (float)(0.75f) fsrWeightOnGround,
    (float)(0.3f) accZThreshold, // in seconds
    (float)(3.f) fsrDbgThreshold, // in seconds
    (Angle)(60_deg) uprightAngleThreshold,
    (bool)(true) standUpOnlyWhenLyingStill,
    (float)(1.25f) secondsUntilLyingStill,
    (float)(10.f) maxLyingStillThreshold,
    (int)(85) directionGyroThreshold,
    (bool)(false) enableForcedUprightDebugMode,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) gyrosource,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) accsource
  )
);


/**
* @class SimpleFallDownStateDetector
*
* A module for computing the current body state from sensor data
*/
class SimpleFallDownStateDetector : public SimpleFallDownStateDetectorBase
{
public:
  /** Default constructor */
  SimpleFallDownStateDetector();

private:
  /** Executes this module
  * @param fallDownState The data structure that is filled by this module
  */
  void update(FallDownState& fallDownState);
  void updateVariable(FallDownState& fallDownState);
  void detectFlying(FallDownState& fallDownState);
  void detectHeldOnOneShoulder(FallDownState& fallDownState);
  void detectFalling(FallDownState& fallDownState);
  void detectDiving(FallDownState& fallDownState);
  void detectOnGround(FallDownState& fallDownState);
  void detectLyiningStill(FallDownState& fallDownState);
  void detectDirection(FallDownState& fallDownState);
  void detectStandingUp(FallDownState& fallDownState);
  void detectUpright(FallDownState& fallDownState);
  void checkFsrSum(FallDownState& fallDownState);
  void resolveFallDownState(FallDownState& fallDownState);

  bool mightFlying();
  bool notOnGround();

  std::queue<FallDownState::State> stateQueue;

  FallDownState::Direction lastDirection;
  FallDownState::State lastState;

  Angle angleXZ = 0;
  Angle angleYZ = 0;
  float gyroX = 0.f;
  float gyroY = 0.f;
  bool mightUpright = true;
  unsigned onGroundCounter = 0;
  unsigned standUpCounter = 0;
  unsigned lyingStillCounter = 0;
  unsigned heldOnShoulderCounter = 0;
  unsigned accZCounter = 0;
  unsigned fsrDbgCounter = 0;
  float fsrSum = INFINITY;
  float fsrMin = INFINITY;
  bool doubleSupport = true;
  bool fsrDbg = true;
};
