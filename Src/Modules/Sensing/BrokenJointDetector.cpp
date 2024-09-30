/**
* @file BrokenJointDetector.cpp
*
* This file implements a module that provides information about the current status of the robot's joints based on B-Humans "checkMotorMalfunction" by Philip Reichenberg.
*
* @author Diana Kleingarn
*/

#include "BrokenJointDetector.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Build.h"
#include "Tools/Settings.h"
#include <cmath>

BrokenJointDetector::BrokenJointDetector()
{
  currents.resize(Joints::numOfJoints);
  flagsBroken.resize(Joints::numOfJoints);
  flagsStuck.resize(Joints::numOfJoints);
  averageCurrents.resize(Joints::numOfJoints);
  fill(flagsBroken.begin(), flagsBroken.end(), 0);
  fill(flagsStuck.begin(), flagsStuck.end(), 0);
  timestampLastCheckBroken = 0;
  timestampLastCheckStuck = 0;
}


void BrokenJointDetector::update(BrokenJointState& brokenJointState)
{
  robotName = enableName ? Global::getSettings().robotName + "! " : "";
  // get the currents-> power
  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    short value = theJointSensorData.currents[i];
    currents[i].push_front(static_cast<int>(value == SensorData::off ? 1.f : value));
    averageCurrents[i] = currents[i].average();
  }
  detectBrokenJoints(brokenJointState);
  detectStuckJoints(brokenJointState);
  // ToDo:If the gyro is stuck, we assume the whole connection to the robot disconnected. In such a case, we can not detect a motor malfunction.
}


void BrokenJointDetector::detectBrokenJoints(BrokenJointState& brokenJointState)
{
  if (stuckJointMalfunction)
    return;

  if (theFrameInfo.getTimeSince(timestampLastCheckBroken) > checkWaitTime)
  {
    timestampLastCheckBroken = theFrameInfo.time;
    for (size_t i = 0; i < currents.size(); i++)
    {
      // Decide which threshold to use
      Angle jointDiff = theGroundContactState.contact ? minJointDiffNormalJoints : minJointDiffNormalJointsNoGroundConntact;
      int stiffness = stiffnessThreshold;
      if (i == Joints::lAnkleRoll || i == Joints::rAnkleRoll)
        jointDiff = minJointDiffAnkleRoll;
      else if (i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
      {
        stiffness = stiffnessThresholdArms;
        jointDiff = minJointDiffArms;
      }
      // If current is 0, the stiffness high enough and the jointRequest and jointAngle difference is high enough, increase the counter
      if (averageCurrents[i] == 0 && theRawJointRequest.stiffnessData.stiffnesses[i] >= stiffness && std::abs(theJointError.angles[i]) >= jointDiff)
        flagsBroken[i] = std::min(flagsBroken[i] + 1, 100);
      else
        flagsBroken[i] = std::max(flagsBroken[i] - 1, 0);
    }
    //result
    int brokenJointCounter = 0;
    for (size_t i = 0; i < flagsBroken.size(); i++)
    { // Are enough possible motor malfunction detected?
      if (flagsBroken[i] >= flagsThreshold)
      {
        brokenJointState.brokenJointStatus[i] = true;
        brokenJointCounter++;
      }
      else
        brokenJointState.brokenJointStatus[i] = false;
    }
    if (brokenJointCounter > 0)
    {
      brokenJointState.jointState = BrokenJointState::malfunction;
      brokenJointMalfunction = true;
    }
    else
    {
      brokenJointState.jointState = BrokenJointState::alright;
      brokenJointMalfunction = false;
    }
  }
}
void BrokenJointDetector::detectStuckJoints(BrokenJointState& brokenJointState)
{
  if (brokenJointMalfunction)
    return;

  if (theFrameInfo.getTimeSince(timestampLastCheckStuck) > checkWaitTime)
  {
    timestampLastCheckStuck = theFrameInfo.time;
    for (size_t i = 0; i < currents.size(); i++)
    {
      // Decide which threshold to use
      Angle jointDiff = theGroundContactState.contact ? minJointDiffNormalJoints : minJointDiffNormalJointsNoGroundConntact;
      if (i == Joints::lAnkleRoll || i == Joints::rAnkleRoll)
        jointDiff = minJointDiffAnkleRoll;
      else if (i >= Joints::firstArmJoint && i < Joints::firstLegJoint)
      {
        jointDiff = minJointDiffArms;
      }
      // If current is high, the stiffness high enough and the jointRequest and jointAngle difference is high enough, increase the counter
      if (averageCurrents[i] > currentsThreshold && theRawJointRequest.stiffnessData.stiffnesses[i] != 0 && std::abs(theJointError.angles[i]) >= jointDiff)
        flagsStuck[i] = std::min(flagsStuck[i] + 1, 100);
      else
        flagsStuck[i] = std::max(flagsStuck[i] - 1, 0);
    }
    //result
    int stuckJointCounter = 0;
    for (size_t i = 0; i < flagsStuck.size(); i++)
    { // Are enough possible motor malfunction detected?
      if (flagsStuck[i] >= flagsThreshold)
      {
        brokenJointState.stuckJointStatus[i] = true;
        stuckJointCounter++;
      }
      else
        brokenJointState.stuckJointStatus[i] = false;
    }
    if (stuckJointCounter > 0)
    {
      brokenJointState.jointState = BrokenJointState::malfunction;
      stuckJointMalfunction = true;
    }
    else
    {
      brokenJointState.jointState = BrokenJointState::alright;
      stuckJointMalfunction = false;
    }
  }
}


MAKE_MODULE(BrokenJointDetector, sensing)
