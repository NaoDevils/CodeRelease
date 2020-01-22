/**
* @file CSConverter.h
* 
* Converts the foot positions from world coordinate system to robot
* coordinate system by using the target center of mass in the world
* coordinate system and the actual center of mass in the robot 
* coordinate system.
*
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a> 
* based on work of:
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include "Tools/Streams/RobotParameters.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

#include <list>
#include <algorithm>
#include "StepData.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Configuration/RobotDimensions.h"

#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/RobotPose.h"

STREAMABLE(Odometry,
{
  ENUM(OdometryVariant,
  { ,
    ODOMETRY_FROM_INERTIA_MATRIX,
    ODOMETRY_FROM_WALKING_ENGINE,
  }),
});

MODULE(CSConverter2019,
{ ,
  REQUIRES(FrameInfo),
  REQUIRES(TargetCoM),
  REQUIRES(WalkingEngineParams),
  REQUIRES(ActualCoMRCS),
  REQUIRES(BodyTilt),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotModel),
  REQUIRES(RobotInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(IMUModel),
  REQUIRES(InertialSensorData),
  REQUIRES(FallDownState),
  REQUIRES(Footpositions),
  REQUIRES(FootSteps),
  REQUIRES(ArmContact),
  REQUIRES(RobotDimensions),
  REQUIRES(ZMPModel),
  PROVIDES(KinematicRequest),
  PROVIDES(WalkingInfo),

  LOADS_PARAMETERS(
  { ,
    ((Odometry) OdometryVariant) odometryVariant,
    (bool)(false) useIMUModel,
    (bool)(false) useAngleBuffer,
    (bool)(false) useGyroBuffer,
    (bool)(false) useLegJointBalancing,
    (BalanceParameters) legJointBalanceParams,
    (COMShiftParameters) comShiftParameters,
    (bool)(false) rotateLegWithBody,
    (float)(0.f) comShiftInfluenceOnSensorError,
    (float)(0.f) ankleOffsetInfluenceOnSensorError,
    (float)(0.f) fsrInfluenceOnSensorError,
  }),
});

class CSConverter2019 : public CSConverter2019Base
{
public:
  CSConverter2019();
  ~CSConverter2019();

  void update(KinematicRequest &kinematicRequest);
  void update(WalkingInfo &walkingInfo);

  /** Resets the converter. */
  void reset();

private:
  /** 
    * Converts the foot positions from world coordinate system to robot
    * coordinate system by using the target center of mass in the world
    * coordinate system and the actual center of mass in the robot 
    * coordinate system.
    * @param kinematicRequest Filled with the foot positions in robot
    * coordinate system and has to be sent to the inverse kinematics.
    */
  void updateKinematicRequest(KinematicRequest &kinematicRequest);

  /**
    * Updates some information about the current walk.
    * @param walkingInfo Filled with some information about the walk.
    */
  void updateWalkingInfo(WalkingInfo &walkingInfo);

  /** Calculate the conversion with the given data
    *	@param requiredOffset Filled with the foot positions in robot coordiaten system.
    *  @param newCoMTarget Target position of the CoM in world coordinates.
    *  @param curPos Target foot positions in world coordinates.
    *  @param CoM Actual CoM in robot coordinate system.
    */
  void toRobotCoords(StepData *requiredOffset, Point &newCoMTarget, Footposition &curPos, Point CoM);

  void clearKinematicRequest(KinematicRequest & kinematicRequest);
  KinematicRequest::KinematicType determineKinematicType();
  void determineRunningState();
  void applySpeedDependentTilt(Footpositions & fp);
  void determineFootInAir(const Footposition & curPos);
  void applyFootPitchRollPD(Footposition & curPos);
  void calculateOdometry(const Footposition & curPos);
  void applyComShift(float &xOffset, float &yOffset);
  Point handleArmContactState();
  void applyOffsets(KinematicRequest& kinematicRequest);
  void resetSensorControl();

  float speedDependentTilt;
  bool isRunning; /**< Is the walking engine currently running? */
  bool isInstantKickRunning;

  StepData currentStep;
  int footInAir;
  Point robotPosition; /**< Position of robot body in world coordinate system. */
  Point lastTargetCoM;
  Point lastSpeed;
  Point acc;
  Vector2f sensorError = Vector2f::Zero();

  // Odometry //
  Point odometry; /**< Odometry data. */
  Point offsetToRobotPoseAfterPreview; /**< Future position of the robot when all steps in the preview are executed. */
  Point originWCS; /**< The origin of the TorsoMatrix coordinate system in world coordinate system */
  Point lastPositionBetweenFeet;
  Pose3f lastTorsoMatrix; /**< The last inertia matrix for calculating the odometry offset. */

  // AccXAlpha // 
  RingBufferWithSum<float, 5> accXBuffer; 
  RingBufferWithSum<float, 5> gyroXBuffer;
  RingBufferWithSum<float, 5> gyroYBuffer;
  RingBufferWithSum<float, 5> angleXBuffer;
  RingBufferWithSum<float, 5> angleYBuffer;
  float lastSpeedX = 0.f;
  float lastStepAccX = 0.f;
  unsigned interpolStartTime = 0;
  float interpolRatio = 0.f;

  // LegJointBalancer
  float pidGyroX_sum = 0.f;
  float pidGyroY_sum = 0.f;
  float pidAngleX_sum = 0.f;
  float pidAngleY_sum = 0.f;
  float pidGyroX_last = 0.f;
  float pidGyroY_last = 0.f;
  float pidAngleX_last = 0.f;
  float pidAngleY_last = 0.f;
  float lastComX = 0.f;

  //typedef std::list<Footposition *> FootList;

  //Point robotPosition; /**< Position of robot body in world coordinate system. */
  //Point odometry; /**< Odometry data. */
  //Point offsetToRobotPoseAfterPreview; /**< Future position of the robot when all steps in the preview are executed. */
  //Point originWCS; /**< The origin of the TorsoMatrix coordinate system in world coordinate system */
  //Point lastPos, lastTargetCoM, lastSpeed, acc, lastOffset[2];
  //StepData currentStep;
  //bool isInstantKickRunning;
  //float bodyPitch[2];
  //float speedDependentTilt;
  //bool bodyTiltApplied;
  //RingBufferWithSum<float,5> accXBuffer;
  //
  //bool fallingDown; /**< Is the robot falling? */
  //bool lastFootPositionsValid;
  //bool isLeavingPossible; /**< Is it possible to leave the walking engine without falling? */
  //
  //
  //KinematicRequest lastRequest;
  //
  //Pose3f lastTorsoMatrix; /**< The last inertia matrix for calculating the odometry offset. */

};


