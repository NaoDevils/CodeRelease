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
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/ArmContact.h"

#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/JointError.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingInfo.h"
MODULE(CSConverter2019,
  REQUIRES(FrameInfo),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  REQUIRES(TargetCoM),
  REQUIRES(WalkCalibration),
  REQUIRES(ActualCoMRCS),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotModel),
  REQUIRES(FsrSensorData),
  REQUIRES(JoinedIMUData),
  USES(JointError),
  REQUIRES(FallDownState),
  REQUIRES(Footpositions),
  REQUIRES(FootSteps),
  REQUIRES(ArmContact),
  REQUIRES(SpeedRequest),
  PROVIDES(KinematicRequest),
  PROVIDES(WalkingInfo),

  LOADS_PARAMETERS(
    ENUM(OdometryVariant,
      ODOMETRY_FROM_INERTIA_MATRIX,
      ODOMETRY_FROM_WALKING_ENGINE
    ),
    (OdometryVariant) odometryVariant,
    (bool)(true) determineSupportFootByFSR
  )
);

class CSConverter2019 : public CSConverter2019Base
{
public:
  CSConverter2019();
  ~CSConverter2019();

  void update(KinematicRequest& kinematicRequest);
  void update(WalkingInfo& walkingInfo);

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
  void updateKinematicRequest(KinematicRequest& kinematicRequest);

  /**
    * Updates some information about the current walk.
    * @param walkingInfo Filled with some information about the walk.
    */
  void updateWalkingInfo(WalkingInfo& walkingInfo);

  /** Calculate the conversion with the given data
    *	@param requiredOffset Filled with the foot positions in robot coordiaten system.
    *  @param newCoMTarget Target position of the CoM in world coordinates.
    *  @param curPos Target foot positions in world coordinates.
    *  @param CoM Actual CoM in robot coordinate system.
    */
  void toRobotCoords(StepData* requiredOffset, Point& newCoMTarget, Footposition& curPos, Point CoM);

  void clearKinematicRequest(KinematicRequest& kinematicRequest);
  KinematicRequest::KinematicType determineKinematicType();
  void determineRunningState();
  void applySpeedDependentTilt(Footpositions& fp);
  void determineFootInAir(const Footposition& curPos);
  void applyFootPitchRollPD(Footposition& curPos);
  void calculateOdometry(const Footposition& curPos);
  void applyComShift(float& xOffset, float& yOffset);
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

  // Odometry //
  Point odometry; /**< Odometry data. */
  Point offsetToRobotPoseAfterPreview; /**< Future position of the robot when all steps in the preview are executed. */
  Point originWCS; /**< The origin of the TorsoMatrix coordinate system in world coordinate system */
  Point lastPositionBetweenFeet;
  Pose3f lastTorsoMatrix; /**< The last inertia matrix for calculating the odometry offset. */

  // AccXAlpha //
  RingBufferWithSum<float, 3> accXBuffer;

  float lastSpeedX = 0.f;
  float lastStepAccX = 0.f;
  unsigned interpolStartTime = 0;
  float interpolRatio = 0.f;

  // LegJointBalancer
  float pidAngleXAnkle_sum = 0.f, pidAngleXHip_sum = 0.f;
  float pidAngleYAnkle_sum = 0.f, pidAngleYHip_sum = 0.f;
  float pidAngleXAnkle_last = 0.f, pidAngleXHip_last = 0.f;
  float pidAngleYAnkle_last = 0.f, pidAngleYHip_last = 0.f;

  JointAngles jointError_sum, jointError_last;
  float lastComX = 0.f;

  std::vector<float> offsetSpline;
  unsigned lastPolygonHash = 0;

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
