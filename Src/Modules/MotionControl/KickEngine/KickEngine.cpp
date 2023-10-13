/*
 * @file KickEngine.cpp
 * This file implements a module that creates motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include "KickEngine.h"
#include "KickEngineParameters.h"
#include "Platform/File.h"
#include "Representations/MotionControl/KickRequest.h"
#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>

#include <cstdio>
#include <cstring>

MAKE_MODULE(KickEngine, motionControl)

KickEngine::KickEngine()
{
  params.reserve(10);
  for (const auto& kickEngineParameters : KicksProvider::loadKickEngineParameters())
  {
    params.push_back(kickEngineParameters);
  }

  //This is needed for adding new kicks
  KickEngineParameters newKickMotion;
  strcpy(newKickMotion.name, "newKick");
  params.push_back(newKickMotion);
};

void KickEngine::update(KickEngineOutput& kickEngineOutput)
{
  if (theMotionSelection.ratios[MotionRequest::kick] > 0.f)
  {
    data.setCycleTime(theFrameInfo.cycleTime);

    const JointRequest& lastMotionOutput = (theMotionSelection.ratios[MotionRequest::specialAction] > 0) ? (JointRequest&)theSpecialActionsOutput : (JointRequest&)theWalkingEngineOutput;

    if (theMotionSelection.ratios[MotionRequest::kick] < 1.f && !compensated)
      compensate = true;

    data.setRobotModel(theRobotModel);

    if (data.sitOutTransitionDisturbance(compensate, compensated, theJoinedIMUData.imuData[anglesource], kickEngineOutput, lastMotionOutput, theFrameInfo))
    {
      if (data.activateNewMotion(theMotionRequest.kickRequest, kickEngineOutput.isLeavingPossible) && theMotionRequest.motion == MotionRequest::kick)
      {
        data.initData(theFrameInfo, theMotionRequest, params, theJointSensorData, theTorsoMatrix);
        data.setCurrentKickRequest(theMotionRequest);
        data.setExecutedKickRequest(kickEngineOutput.executedKickRequest);

        data.internalIsLeavingPossible = false;
        kickEngineOutput.isLeavingPossible = false;

        kickEngineOutput.odometryOffset = Pose2f();

        for (int i = Joints::lShoulderPitch; i < Joints::numOfJoints; ++i)
          kickEngineOutput.stiffnessData.stiffnesses[i] = 100;

        kickEngineOutput.isStable = true;
      } //this gotta go to config file and be more common

      if (data.checkPhaseTime(theFrameInfo, theJointSensorData, theTorsoMatrix))
      {
        data.calcPhaseState();
        data.calcPositions();
        data.setStaticReference();
        timeSinceLastPhase = theFrameInfo.time;
      }
      else
      {
        kickEngineOutput.isLeavingPossible = true;
        data.internalIsLeavingPossible = true;
      }

      //  if(data.isMotionAlmostOver()) //last three phases are unstable
      //    kickEngineOutput.isStable = false;

      if (data.calcJoints(kickEngineOutput, theRobotDimensions, theHeadJointRequest))
      {
        data.balanceCOM(kickEngineOutput, theRobotDimensions, theMassCalibration, theJoinedIMUData.imuData[anglesource]);
        data.calcJoints(kickEngineOutput, theRobotDimensions, theHeadJointRequest);
        data.mirrorIfNecessary(kickEngineOutput);
      }
      data.addGyroBalance(kickEngineOutput, theJointCalibration, theJoinedIMUData.imuData[anglesource], theMotionSelection.ratios[MotionRequest::kick]);
    }
  }
  else
  {
    compensated = false;
  }

  data.setEngineActivation(theMotionSelection.ratios[MotionRequest::kick]);
  data.ModifyData(theMotionRequest.kickRequest, kickEngineOutput, params);
}
