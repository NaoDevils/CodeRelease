/**
* @file Modules/MotionControl/MotionCombinator.h
* This file declares a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author (arm upgrade) Jesse Richter-Klug
*/

#pragma once

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Module/Module.h"


MODULE(MotionCombinator,
  REQUIRES(OdometryCorrectionTables),
  REQUIRES(FallDownState),
  REQUIRES(Footpositions),
  REQUIRES(FrameInfo),
  REQUIRES(HeadJointRequest),
  REQUIRES(JoinedIMUData),
  REQUIRES(JointSensorData),
  REQUIRES(RobotModel),
  REQUIRES(KickEngineOutput),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSelection),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StandEngineOutput),
  REQUIRES(RobotInfo),
  REQUIRES(StiffnessSettings),
  REQUIRES(MotionSettings),
  REQUIRES(JointCalibration),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkingInfo),
  REQUIRES(SpeedInfo),
  PROVIDES(JointRequest),
  REQUIRES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  PROVIDES(MotionInfo),
  PROVIDES(OdometryData),
  LOADS_PARAMETERS(,
    (bool)(true) emergencyOffEnabled,
    (bool)(false) freezeJointRequestOnError, /**< If true, the last valid jointRequest will be used if an invalid is encountered. */
    (unsigned)(70) recoveryTime, /**< The number of frames to interpolate after emergency-stop. */
    (bool)(true) useJointAccLimit, // use below acc limit for joints?
    (Angle)(180_deg) maxJointAcceleration, // Used to counter the softbank issue with the V6. Angle/(s^2)
    (Angle)(35_deg) JointDiffArmsStuck, // Angle Diff to check if arms are stuck 
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (unsigned)(20) durationCenterArmFirstPhase,
    (unsigned)(20) durationCenterArmSecondPhase,
    (unsigned)(20) durationCenterArmThirdPhase
  )
);


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

#ifndef NDEBUG
  SpecialActionRequest::SpecialActionID lastSpecialAction = SpecialActionRequest::playDead;
  SpecialActionRequest::SpecialActionID currentSpecialAction = SpecialActionRequest::playDead;
#endif
  OdometryData lastOdometryData;
  JointRequest lastJointRequest;
  JointAngles lastJointDiff = JointAngles();

public:
  /**
  * Default constructor.
  */
  MotionCombinator();

private:
  void update(OdometryData& odometryData);
  void update(JointRequest& jointRequest);
  void update(MotionInfo& motionInfo)
  {
    motionInfo = this->motionInfo;
  }

  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);
  void sitFront(JointRequest& jointRequest);
  void sit(JointRequest& jointRequest);
  void centerArms(JointRequest& jointRequest);
  void saveArms(JointRequest& jointRequest);
  void armsFront(JointRequest& jointRequest);
  void armsBehind(JointRequest& jointRequest);
  void armsStand(JointRequest& jointRequest);
  void centerArm(JointRequest& jointRequest, bool left);

  /**
  * The method copies all joint angles from one joint request to another,
  * but only those that should not be ignored.
  * @param source The source joint request. All angles != JointAngles::ignore will be copied.
  * @param target The target joint request.
  */
  void copy(const JointRequest& source,
      JointRequest& target,
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
  void interpolate(const JointRequest& from,
      const JointRequest& to,
      float fromRatio,
      JointRequest& target,
      bool interpolateStiffness,
      const Joints::Joint startJoint = static_cast<Joints::Joint>(0),
      const Joints::Joint endJoint = static_cast<Joints::Joint>(Joints::numOfJoints - 1)) const;
};
