/*
 * @file KickEngine.cpp
 * This file implements a module that creates motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include "KickEngine.h"
#include "KickEngineParameters.h"
#include "Platform/File.h"
#include "Representations/MotionControl/KickRequest.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <filesystem>

MAKE_MODULE(KickEngine, motionControl)

KickEngine::KickEngine()
{
  params.reserve(10);

  const std::string dirname = std::string(File::getBHDir()) + "/Config/KickEngine/";
  for (const auto& file : std::filesystem::directory_iterator(dirname))
  {
    const std::string filename = file.path().filename().string();
    if (file.is_regular_file() && filename.substr(filename.size() - 4) == ".kmc")
    {
      InMapFile stream(file.path().string());
      ASSERT(stream.exists());

      KickEngineParameters parameters;
      stream >> parameters;

      strcpy(parameters.name, filename.substr(0, filename.size() - 4).c_str());

      if (KickRequest::getKickMotionFromName(parameters.name) < KickRequest::none)
        params.push_back(parameters);
      else
      {
        OUTPUT_TEXT("Warning: KickRequest is missing the id for " << parameters.name);
        fprintf(stderr, "Warning: KickRequest is missing the id for %s \n", parameters.name);
      }
    }
  }

  for (int i = 0; i < KickRequest::numOfKickMotionIDs - 2; ++i)
  {
    int id = -1;
    for (unsigned int p = 0; p < params.size(); ++p)
    {
      if (KickRequest::getKickMotionFromName(&params[p].name[0]) == i)
      {
        id = i;
        break;
      }
    }
    if (id == -1)
    {
      OUTPUT_TEXT("Warning: The kick motion file for id " << KickRequest::getName((KickRequest::KickMotionID)i) << " is missing.");
      fprintf(stderr, "Warning: The kick motion file for id %s is missing. \n", KickRequest::getName((KickRequest::KickMotionID)i));
    }
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
