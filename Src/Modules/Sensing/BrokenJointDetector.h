/**
* @file BrokenJointDetector.h
*
* This file declares a module that provides information about the current status of the robot's joints.
*
* @author <a href="mailto:diana.kleingarn@tu.dortmund.de">Diana Kleingarn</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/JointError.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/BrokenJointState.h"
#include "Representations/Sensing/GroundContactState.h"


MODULE(BrokenJointDetector,
  USES(MotionInfo),
  USES(SpecialActionsOutput),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  REQUIRES(JointSensorData),
  REQUIRES(RawJointRequest),
  REQUIRES(JointError),
  REQUIRES(WalkingEngineParams), 
  PROVIDES(BrokenJointState),
  LOADS_PARAMETERS(,
    (bool)(true) enableName,
    (int)(60) stiffnessThreshold, // Min stiffness for the legs to check for a motor malfunction. 
    (int)(40) stiffnessThresholdArms, // Min stiffness for the arms to check for a motor malfunction. 
    (int)(5) flagsThreshold, // Min counter for a detected error. 
    (int)(450) currentsThreshold, // Min counter for a detected error. 
    (int)(300) checkWaitTime, // Wait time between motor malfunction checks.
    (Angle)(4_deg) minJointDiffNormalJoints, // Min difference in jointRequest and jointAngles for the legs, to detect a defect. 
    (Angle)(8_deg) minJointDiffNormalJointsNoGroundConntact, // Min difference in jointRequest and jointAngles when no ground contact is detected, to detect a defect. 
    (Angle)(10_deg) minJointDiffAnkleRoll, // Min difference in jointRequest and jointAngles for the ankleRolls, to detect a defect. This value must be high, because the current can be 0 at high differences. 
    (Angle)(6_deg) minJointDiffArms // Min difference in jointRequest and jointAngles for the arms, to detect a defect. 
  )
);


/**
* @class BrokenJointDetector
*
* A module for computing the current body state from sensor data
*/
class BrokenJointDetector : public BrokenJointDetectorBase
{
public:
  /** Default constructor */
  BrokenJointDetector();

private:
  /** Executes this module
  * @param brokenJointState The data structure that is filled by this module
  */
  void update(BrokenJointState& brokenJointState);
  void detectBrokenJoints(BrokenJointState& brokenJointState);
  void detectStuckJoints(BrokenJointState& brokenJointState);

  std::vector<int> flagsBroken; // counts possible defects per joint
  std::vector<int> flagsStuck; // counts possible defects per joint
  std::vector<int> averageCurrents;
  std::vector<RingBufferWithSum<int, 20>> currents;
  std::string robotName;
  unsigned int timestampLastCheckBroken;
  unsigned int timestampLastCheckStuck;
};
