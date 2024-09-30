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
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/FsrModelData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Debugging/DebugDrawings.h"


MODULE(SimpleFallDownStateDetector,
  USES(MotionInfo),
  USES(BehaviorData),
  USES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(JoinedIMUData),
  PROVIDES(FallDownState),
  PROVIDES(FsrModelData),
  LOADS_PARAMETERS(,
    (Angle)(24_deg) fallDownAngleFront,
    (Angle)(24_deg) fallDownAngleSide,
    (Angle)(-17_deg) fallDownAngleBack,
    (Angle)(10_deg) standingUpTolerance,
    (float)(1.f) secondsOnGround,
    (float)(1.f) secondsUntilStandUp,
    (float)(2.f) secondsUntilHeldOnShoulder,
    (float)(2.f) secondsUntilWideStanceFlying,
    (bool)(true) enableFlyingDetection,
    (Angle)(5_deg) minHeldOnShoulderAngle,
    (Angle)(30_deg) maxHeldOnShoulderAngle,
    (Angle)(10_deg) heldOnShoulderTolerance,
    (float)(0.3f) fsrFlyingThreshold,
    (float)(0.75f) fsrOnGroundThreshold,
    (float)(0.1f) fsrCheckThreshold,
    (unsigned int)(72) fsrAttack,
    (unsigned int)(9) fsrBufferSize,
    (unsigned int)(2000) minAirTime, // in milliseconds
    (float)(3.f) fsrDbgThreshold, // in seconds
    (float)(10.f) fsrModelSettleTime, // in seconds
    (int)(50) forceDisturbanceThreshold,
    (int)(10) forceSilenceThreshold,
    (Angle)(60_deg) uprightAngleThreshold,
    (bool)(true) standUpOnlyWhenLyingStill,
    (float)(1.25f) secondsUntilLyingStill,
    (float)(10.f) maxLyingStillThreshold,
    (int)(85) directionGyroThreshold,
    (bool)(false) enableForcedUprightDebugMode,
    (float)(5.5f) naoWeightinKg,
    (float)(1.5f) positiveSlope,
    (float)(0.025f) negativeSlope,
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
  void detectWideStance(FallDownState& fallDownState);
  void detectOnGround(FallDownState& fallDownState);
  void detectLyiningStill(FallDownState& fallDownState);
  void detectDirection(FallDownState& fallDownState);
  void detectStandingUp(FallDownState& fallDownState);
  void detectUpright(FallDownState& fallDownState);
  void detectDisturbanceOfTheForce();
  void detectSilenceOfTheForce();
  void checkFsrSum(FallDownState& fallDownState);
  void resolveFallDownState(FallDownState& fallDownState);

  bool mightFlying();
  bool notOnGround();

  void update(FsrModelData& fsrModelData);

  void calculateFsrModelData(FsrModelData& fsrModelData, float fsrCorrectionValue = 1.f);
  void recalculateFsrModelData(FsrModelData& fsrModelData, float fsrCorrectionValue) { calculateFsrModelData(fsrModelData, fsrCorrectionValue); };
  void smoothingFsrModelData(FsrModelData& fsrModelData);
  FsrModelData localFsrModelData;

  std::queue<FallDownState::State> stateQueue;

  FallDownState::Direction lastDirection;
  FallDownState::State lastState;

  Angle angleXZ = 0;
  Angle angleYZ = 0;
  float gyroX = 0.f;
  float gyroY = 0.f;
  bool mightUpright = true;
  bool notLying = true;
  unsigned onGroundCounter = 0;
  unsigned standUpCounter = 0;
  unsigned lyingStillCounter = 0;
  unsigned heldOnShoulderCounter = 0;
  unsigned wideStanceFlyingCounter = 0;
  unsigned lastOnGroundTimestamp = 0;
  unsigned fsrDbgCounter = 0;
  float fsrSumR = INFINITY;
  float fsrMinR = INFINITY;
  float fsrSumL = INFINITY;
  float fsrMinL = INFINITY;
  bool fsrDbg = true;
  bool fsrBroken = false;
  bool fsrMessageActive = false;
  bool fsrModelReady = false;
  bool senseDisturbanceOfTheForce = false;
  bool senseSilenceOfTheForce = false;
  unsigned fsrModelReadyCounter = 0;

  float leftFsrMinValue = INFINITY;
  unsigned int leftFsrMinAttack = 0;
  float leftFsrMaxValue = -INFINITY;
  unsigned int leftFsrMaxAttack = 0;
  float rightFsrMinValue = INFINITY;
  unsigned int rightFsrMinAttack = 0;
  float rightFsrMaxValue = -INFINITY;
  unsigned int rightFsrMaxAttack = 0;

  std::array<RingBufferWithSum<float, 9>, FsrSensorData::numOfFsrSensorPositions> leftFsrBufferArray;
  std::array<RingBufferWithSum<float, 9>, FsrSensorData::numOfFsrSensorPositions> rightFsrBufferArray;
};
