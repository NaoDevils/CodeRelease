/**
* @file Modules/MotionControl/MotionCombinator.h
* This file declares a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author (arm upgrade) Jesse Richter-Klug
*/

#pragma once

#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Module/Module.h"

MODULE(MotionCombinator,
{ ,
  REQUIRES(OdometryCorrectionTables),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(InertialSensorData),
  REQUIRES(JointSensorData),
  REQUIRES(RobotModel),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  REQUIRES(MotionSelection),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StandEngineOutput),
  REQUIRES(RobotInfo),
  REQUIRES(StiffnessSettings),
  REQUIRES(MotionSettings),
  REQUIRES(LegJointSensorControlParameters),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkingInfo),
  REQUIRES(SpeedInfo),
  PROVIDES(JointRequest),
  REQUIRES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  PROVIDES(MotionInfo),
  PROVIDES(OdometryData),
  LOADS_PARAMETERS(
  { ,
    (bool) emergencyOffEnabled,
    (unsigned) recoveryTime, /**< The number of frames to interpolate after emergency-stop. */
    (bool) useBalancing, // use yOffset and balanceParams for balancing upper body
    (float) yOffset, // upper body y offset (for asymmetry)
  }),
});



class MotionCombinator : public MotionCombinatorBase
{
private:
  NonArmeMotionEngineOutput theNonArmeMotionEngineOutput;

  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2f specialActionOdometry; /**< workaround for accumulating special action odometry. */

  unsigned currentRecoveryTime;

  bool headJawInSavePosition;
  bool headPitchInSavePosition;
  bool isFallingStarted;
  unsigned fallingFrame;

  // for balancing
  SpecialActionRequest::SpecialActionID lastBalancedSpecialAction = SpecialActionRequest::stand;
  unsigned timeWhenSpecialActionStarted = 0;
  bool wasInBalance = false;
  float lastComX = 0;

  OdometryData lastOdometryData;
  JointRequest lastJointRequest;
  float pidGyroX_sum = 0;
  float pidGyroY_sum = 0;
  float pidGyroX_last = 0;
  float pidGyroY_last = 0;
  float pidAngleX_sum = 0;
  float pidAngleY_sum = 0;
  float pidAngleX_last = 0;
  float pidAngleY_last = 0;

  float spid_sumX = 0;
  float spid_sumY = 0;

public:
  /**
  * Default constructor.
  */
  MotionCombinator();

private:
  void update(OdometryData& odometryData);
  void update(JointRequest& jointRequest);
  void update(MotionInfo& motionInfo) { motionInfo = this->motionInfo; }

  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);
  void centerArms(JointRequest& jointRequest);
  void centerArm(JointRequest& jointRequest, bool left);
  void balanceUpperBody(JointRequest& jointRequest);

  /**
  * The method copies all joint angles from one joint request to another,
  * but only those that should not be ignored.
  * @param source The source joint request. All angles != JointAngles::ignore will be copied.
  * @param target The target joint request.
  */
  void copy(const JointRequest& source, JointRequest& target,
    const Joints::Joint startJoint = static_cast<Joints::Joint>(0),
    const Joints::Joint endJoint = static_cast<Joints::Joint>(Joints::numOfJoints - 1)) const;

  /**
  * The method interpolates between two joint requests.
  * @param from The first source joint request. This one is the starting point.
  * @param to The second source joint request. This one has to be reached over time.
  * @param fromRatio The ratio of "from" in the target joint request.
  * @param target The target joint request.
  * @param interpolateStiffness Whether to interpolate stiffness.
  */
  void interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateStiffness,
    const Joints::Joint startJoint = static_cast<Joints::Joint>(0),
    const Joints::Joint endJoint = static_cast<Joints::Joint>(Joints::numOfJoints - 1)) const;

  void calcUpperBodyCOM(Vector3f& com);
  Vector3f rotateVectorOverLine(Vector3f vec, Vector3f sv, Vector3f rv, float angle);
};
