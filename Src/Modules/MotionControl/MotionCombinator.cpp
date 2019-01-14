/**
* @file Modules/MotionControl/MotionCombinator.cpp
* This file implements a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author Jesse Richter-Klug
*/

#include "MotionCombinator.h"
#include "Tools/SensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/RotationMatrix.h"

using std::abs;

MAKE_MODULE(MotionCombinator, motionControl)

MotionCombinator::MotionCombinator() : theNonArmeMotionEngineOutput()
{
  currentRecoveryTime = recoveryTime + 1;
  headJawInSavePosition = false;
  headPitchInSavePosition = false;
  isFallingStarted = false;
  fallingFrame = 0;
}

void MotionCombinator::update(JointRequest& jointRequest)
{
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.x");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.y");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.z");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.x");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.y");
  DECLARE_PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.z");

  //sensorplots
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:gyroX");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:gyroY");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:angleX");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:angleY");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetX");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetY");

  specialActionOdometry += theSpecialActionsOutput.odometryOffset;

  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::Motion::walk] = &theWalkingEngineOutput;
  jointRequests[MotionRequest::Motion::kick] = &theKickEngineOutput;
  jointRequests[MotionRequest::Motion::specialAction] = &theSpecialActionsOutput;
  jointRequests[MotionRequest::Motion::stand] = &theStandEngineOutput;
  
  jointRequest.angles[Joints::headYaw] = theHeadJointRequest.pan;
  jointRequest.angles[Joints::headPitch] = theHeadJointRequest.tilt;

  copy(*jointRequests[theMotionSelection.targetMotion], jointRequest);

  ASSERT(jointRequest.isValid());

  motionInfo.customStepKickInPreview = theSpeedInfo.customStepKickInPreview;

  // Find fully active motion and set MotionInfo
  if (theMotionSelection.ratios[theMotionSelection.targetMotion] == 1.f)
  {
    Pose2f odometryOffset;
    // default values
    motionInfo.motion = theMotionSelection.targetMotion;
    motionInfo.isMotionStable = true;
    motionInfo.offsetToRobotPoseAfterPreview = Pose2f();

    lastJointAngles = theJointAngles;

    switch (theMotionSelection.targetMotion)
    {
    case MotionRequest::walk:
      odometryOffset = OdometryCorrection::correct(
        theWalkingEngineOutput.speed,
        theWalkingEngineOutput.odometryOffset,
        theOdometryCorrectionTables.backCorrectionTable,
        theOdometryCorrectionTables.forwardCorrectionTable,
        theOdometryCorrectionTables.sideCorrectionTable,
        theOdometryCorrectionTables.rotCorrectionTable,
        theOdometryCorrectionTables.rot2DCorrectionTable);
      motionInfo.walkRequest.request = theWalkingEngineOutput.speed;
      motionInfo.walkRequest.requestType = WalkRequest::speed;
      motionInfo.walkRequest.stepRequest = theSpeedInfo.currentCustomStep;
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.x", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.x());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.y", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.y());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.r", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.rotation);
      motionInfo.offsetToRobotPoseAfterPreview = OdometryCorrection::correctPreview(
        theWalkingEngineOutput.speed,
        theWalkingEngineOutput.offsetToRobotPoseAfterPreview,
        theOdometryCorrectionTables.backCorrectionTablePreview,
        theOdometryCorrectionTables.forwardCorrectionTablePreview,
        theOdometryCorrectionTables.sideCorrectionTablePreview);
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.x", motionInfo.offsetToRobotPoseAfterPreview.translation.x());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.y", motionInfo.offsetToRobotPoseAfterPreview.translation.y());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreviewCorrected.r", motionInfo.offsetToRobotPoseAfterPreview.rotation);
      break;
    case MotionRequest::kick:
      odometryOffset = theKickEngineOutput.odometryOffset;
      motionInfo.kickRequest = theKickEngineOutput.executedKickRequest;
      motionInfo.isMotionStable = theKickEngineOutput.isStable;
      break;
    case MotionRequest::specialAction:
      odometryOffset = specialActionOdometry;
      specialActionOdometry = Pose2f();
      motionInfo.specialActionRequest = theSpecialActionsOutput.executedSpecialAction;
      motionInfo.isMotionStable = theSpecialActionsOutput.isMotionStable;
      break;
    default:
      break;
    }

    if (theMotionSelection.targetMotion != MotionRequest::walk && theRobotInfo.hasFeature(RobotInfo::zGyro) && theFallDownState.state == FallDownState::upright)
    {
      Vector3f rotatedGyros = theInertialData.orientation * theInertialData.gyro.cast<float>();
      odometryOffset.rotation = rotatedGyros.z() * theFrameInfo.cycleTime;
    }

    odometryData += odometryOffset;
    ASSERT(jointRequest.isValid());
  }
  else // interpolate motions
  {
    const bool interpolateStiffness = !(theMotionSelection.targetMotion != MotionRequest::specialAction && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
      theMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
    for (int i = 0; i < MotionRequest::numOfMotions; ++i)
      if (i != theMotionSelection.targetMotion && theMotionSelection.ratios[i] > 0.)
      {
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], jointRequest, interpolateStiffness, Joints::headYaw, Joints::headPitch);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], jointRequest, interpolateStiffness, Joints::lShoulderPitch, Joints::lHand);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], jointRequest, interpolateStiffness, Joints::rShoulderPitch, Joints::rHand);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], jointRequest, interpolateStiffness, Joints::lHipYawPitch, Joints::rAnkleRoll);
      }
  }

  ASSERT(jointRequest.isValid());

  /*auto combinateArmMotions = [&](Arms::Arm const arm)
  {
  const int ratioIndexOffset = arm * theArmMotionSelection.rightArmRatiosOffset;
  const Joints::Joint startJoint = arm == Arms::left ? Joints::lShoulderPitch : Joints::rShoulderPitch;
  const Joints::Joint endJoint = arm == Arms::left ? Joints::lHand : Joints::rHand;

  if(theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] != 1.f)
  {
  if(theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] > 0 &&
  ArmMotionRequest::none != theArmMotionSelection.targetArmMotion[arm])
  copy(jointRequest, theNonArmeMotionEngineOutput, startJoint, endJoint);

  if(ArmMotionRequest::none != theArmMotionSelection.targetArmMotion[arm])
  copy(*armJointRequests[theArmMotionSelection.targetArmMotion[arm]], jointRequest, startJoint, endJoint);
  }

  if(theArmMotionSelection.armRatios[ratioIndexOffset + theArmMotionSelection.targetArmMotion[arm]] == 1.f)
  {
  armMotionInfo.armMotion[arm] = theArmMotionSelection.targetArmMotion[arm];

  switch(theArmMotionSelection.targetArmMotion[arm])
  {
  case ArmMotionRequest::keyFrame:
  armMotionInfo.armKeyFrameRequest = theArmMotionSelection.armKeyFrameRequest;
  break;
  case ArmMotionRequest::none:
  default:
  break;
  }
  }
  else
  {
  const bool interpolateStiffness = !(theMotionSelection.targetMotion != MotionRequest::specialAction &&
  theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
  theMotionSelection.ratios[MotionRequest::specialAction] > 0.f &&
  theArmMotionSelection.armRatios[ratioIndexOffset + ArmMotionRequest::none] > 0);

  const JointRequest toJointRequest = theArmMotionSelection.targetArmMotion[arm] == ArmMotionRequest::none ?
  *jointRequests[theMotionSelection.targetMotion] : *armJointRequests[theArmMotionSelection.targetArmMotion[arm]];

  for(int i = 0; i < ArmMotionRequest::numOfArmMotions; ++i)
  {
  if(i != theArmMotionSelection.targetArmMotion[arm] && theArmMotionSelection.armRatios[ratioIndexOffset + i] > 0)
  {
  interpolate(*armJointRequests[i], toJointRequest, theArmMotionSelection.armRatios[ratioIndexOffset + i], jointRequest, interpolateStiffness, startJoint, endJoint);
  }
  }
  }

  ASSERT(jointRequest.isValid());
  };

  combinateArmMotions(Arms::left);
  combinateArmMotions(Arms::right);*/

  if (useBalancing)
    balanceUpperBody(jointRequest);

  if (emergencyOffEnabled)
  {
    if (theFallDownState.state == FallDownState::falling && motionInfo.motion != MotionRequest::specialAction)
    {
      saveFall(jointRequest);
      centerHead(jointRequest);
      centerArms(jointRequest);
      currentRecoveryTime = 0;

      ASSERT(jointRequest.isValid());
    }
    else if ((theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::onGround) && (motionInfo.motion != MotionRequest::specialAction))
    {
      centerHead(jointRequest);

      ASSERT(jointRequest.isValid());
    }
    else
    {
      if (theFallDownState.state == FallDownState::upright)
      {
        headJawInSavePosition = false;
        headPitchInSavePosition = false;
        isFallingStarted = false;
      }

      if (currentRecoveryTime < recoveryTime)
      {
        currentRecoveryTime += 1;
        float ratio = (1.f / float(recoveryTime)) * currentRecoveryTime;
        for (int i = 0; i < Joints::numOfJoints; i++)
        {
          jointRequest.stiffnessData.stiffnesses[i] = 30 + int(ratio * float(jointRequest.stiffnessData.stiffnesses[i] - 30));
        }
      }
    }
  }

  float sum(0);
  int count(0);
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if (jointRequest.angles[i] != JointAngles::off && jointRequest.angles[i] != JointAngles::ignore && lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:legsOnly", sum / count);
  for (int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if (jointRequest.angles[i] != JointAngles::off && jointRequest.angles[i] != JointAngles::ignore && lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:all", sum / count);

  sum = 0;
  count = 0;
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if (lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(lastJointRequest.angles[i] - theJointAngles.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:differenceToJointData:legsOnly", sum / count);

  for (int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if (lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(lastJointRequest.angles[i] - theJointAngles.angles[i]);
      count++;
    }
  }
  lastJointRequest = jointRequest;
  PLOT("module:MotionCombinator:differenceToJointData:all", sum / count);
  /*
  for(int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
  char name[100];
  sprintf(name, "module:MotionCombinator:angles[%d]", i);
  PLOT(name, jointRequest.angles[i]);
  sprintf(name, "module:MotionCombinator:angles[%d]", i);
  PLOT(name, jointRequest.stiffnessData.[i]);
  }*/
#ifndef NDEBUG
  if (!jointRequest.isValid())
  {
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "jointRequest.log");
      stream << jointRequest;
      OutMapFile stream2(logDir + "motionSelection.log");
      stream2 << theMotionSelection;
    }
    ASSERT(false);
  }
#endif
  PLOT("module:MotionCombinator:leftLegAngles[0]", jointRequest.angles[Joints::lHipYawPitch + 0]);
  PLOT("module:MotionCombinator:leftLegAngles[1]", jointRequest.angles[Joints::lHipYawPitch + 1]);
  PLOT("module:MotionCombinator:leftLegAngles[2]", jointRequest.angles[Joints::lHipYawPitch + 2]);
  PLOT("module:MotionCombinator:leftLegAngles[3]", jointRequest.angles[Joints::lHipYawPitch + 3]);
  PLOT("module:MotionCombinator:leftLegAngles[4]", jointRequest.angles[Joints::lHipYawPitch + 4]);
  PLOT("module:MotionCombinator:leftLegAngles[5]", jointRequest.angles[Joints::lHipYawPitch + 5]);

  PLOT("module:MotionCombinator:rightLegAngles[0]", jointRequest.angles[Joints::lHipYawPitch + 6]);
  PLOT("module:MotionCombinator:rightLegAngles[1]", jointRequest.angles[Joints::lHipYawPitch + 7]);
  PLOT("module:MotionCombinator:rightLegAngles[2]", jointRequest.angles[Joints::lHipYawPitch + 8]);
  PLOT("module:MotionCombinator:rightLegAngles[3]", jointRequest.angles[Joints::lHipYawPitch + 9]);
  PLOT("module:MotionCombinator:rightLegAngles[4]", jointRequest.angles[Joints::lHipYawPitch + 10]);
  PLOT("module:MotionCombinator:rightLegAngles[5]", jointRequest.angles[Joints::lHipYawPitch + 11]);

  PLOT("module:MotionCombinator:leftLegStiffness[0]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 0]);
  PLOT("module:MotionCombinator:leftLegStiffness[1]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 1]);
  PLOT("module:MotionCombinator:leftLegStiffness[2]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 2]);
  PLOT("module:MotionCombinator:leftLegStiffness[3]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 3]);
  PLOT("module:MotionCombinator:leftLegStiffness[4]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 4]);
  PLOT("module:MotionCombinator:leftLegStiffness[5]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 5]);

  PLOT("module:MotionCombinator:rightLegStiffness[0]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 6]);
  PLOT("module:MotionCombinator:rightLegStiffness[1]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 7]);
  PLOT("module:MotionCombinator:rightLegStiffness[2]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 8]);
  PLOT("module:MotionCombinator:rightLegStiffness[3]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 9]);
  PLOT("module:MotionCombinator:rightLegStiffness[4]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 10]);
  PLOT("module:MotionCombinator:rightLegStiffness[5]", jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 11]);
}

void MotionCombinator::update(OdometryData& odometryData)
{
  if (!theRobotInfo.hasFeature(RobotInfo::zGyro) || theFallDownState.state != FallDownState::upright)
    this->odometryData.rotation += theFallDownState.odometryRotationOffset;
  this->odometryData.rotation.normalize();

  odometryData = this->odometryData;

  Pose2f odometryOffset(odometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionCombinator:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionCombinator:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionCombinator:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = odometryData;
}

void MotionCombinator::saveFall(JointRequest& jointRequest)
{
  for (int i = 0; i < Joints::numOfJoints; i++)
    jointRequest.stiffnessData.stiffnesses[i] = 30;
}

void MotionCombinator::centerArms(JointRequest& jointRequest)
{
  if (!isFallingStarted)
  {
    isFallingStarted = true;
    fallingFrame = 0;
  }

  /*if (theArmMotionSelection.armRatios[ArmMotionRequest::keyFrame] > 0)
    centerArm(jointRequest, true);

  if (theArmMotionSelection.armRatios[ArmMotionRequest::keyFrame + theArmMotionSelection.rightArmRatiosOffset] > 0)
    centerArm(jointRequest, false);*/
}

void MotionCombinator::centerArm(JointRequest& jointRequest, bool left)
{
  const int sign(-1 + 2 * left); // sign = 1 for left arm, for right its -1
  int i(left ? Joints::lShoulderPitch : Joints::rShoulderPitch);
  int j(i);

  if (fallingFrame < 20)
  {
    jointRequest.angles[i++] = 119.5_deg;
    jointRequest.angles[i++] = sign * 25_deg;
    jointRequest.angles[i++] = sign * 45_deg;
    jointRequest.angles[i++] = sign * -11.5_deg;

    while (i < j)
      jointRequest.stiffnessData.stiffnesses[j++] = 100;
  }
  else if (fallingFrame < 80)
  {
    jointRequest.angles[i++] = 90_deg;
    jointRequest.angles[i++] = sign * 11.5_deg;
    jointRequest.angles[i++] = sign * -90_deg;
    jointRequest.angles[i++] = sign * -11.5_deg;

    if (fallingFrame < 40)
      while (i < j)
        jointRequest.stiffnessData.stiffnesses[j++] = 100;
    else
      while (j < i)
        jointRequest.stiffnessData.stiffnesses[j++] = 0;
  }
}

void MotionCombinator::centerHead(JointRequest& jointRequest)
{
  jointRequest.angles[Joints::headYaw] = 0;
  jointRequest.angles[Joints::headPitch] = 0;
  if (theFallDownState.direction == FallDownState::front)
    jointRequest.angles[Joints::headPitch] = -0.65f;
  else if (theFallDownState.direction == FallDownState::back)
    jointRequest.angles[Joints::headPitch] = 0.5f;
  if (abs(theJointAngles.angles[Joints::headYaw]) > 0.1f && !headJawInSavePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 100;
  else
  {
    headJawInSavePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 25;
  }
  if (abs(theJointAngles.angles[Joints::headPitch] - jointRequest.angles[Joints::headPitch]) > 0.1f && !headPitchInSavePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 100;
  else
  {
    headPitchInSavePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 50;
  }
}

void MotionCombinator::balanceUpperBody(JointRequest& jointRequest)
{
  if (theFallDownState.state != FallDownState::upright)
    return;
  float newComX = theRobotModel.centerOfMass.x();

  for (int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; i++)
    if (jointRequest.angles[i] == JointAngles::ignore || jointRequest.angles[i] == JointAngles::off)
      return;

  bool isSpecialAction = theMotionSelection.ratios[MotionRequest::specialAction] == 1.f;
  bool isWalk = theMotionSelection.ratios[MotionRequest::walk] == 1.f;

  if (!isWalk && !isSpecialAction)
    return;
  
  SpecialActionRequest::SpecialActionID specialAction = theMotionSelection.specialActionRequest.specialAction;
  if (isSpecialAction)
  {
    int startBalanceAfter = -1;
    int timeForSpecialAction = -1;
    Angle maxAngleXForBalance = 0_deg;
    Angle maxAngleYForBalance = 0_deg;
    bool balancedSpecialAction = false;
    for (size_t i = 0; i < theMotionSettings.specialActionBalanceList.specialActionBalanceEntries.size(); i++)
    {
      const SpecialActionBalanceList::SpecialActionBalanceEntry &entry = theMotionSettings.specialActionBalanceList.specialActionBalanceEntries[i];
      if (specialAction == entry.specialAction)
      {
        balancedSpecialAction = true;
        lastBalancedSpecialAction = specialAction;
        startBalanceAfter = entry.balanceStartTime;
        timeForSpecialAction = entry.specialActionDuration;
        maxAngleXForBalance = entry.maxXAngle + (wasInBalance ? 10_deg : 0_deg);
        maxAngleYForBalance = entry.maxYAngle + (wasInBalance ? 10_deg : 0_deg);
        if (theFrameInfo.getTimeSince(timeWhenSpecialActionStarted) > timeForSpecialAction)
          timeWhenSpecialActionStarted = theFrameInfo.time;
        break;
      }
    }

    if (!balancedSpecialAction)
    {
      wasInBalance = false;
      timeWhenSpecialActionStarted = 0;
      return;
    }

    int timeSinceSpecialAction = theFrameInfo.getTimeSince(timeWhenSpecialActionStarted);

    // only balance in this time windows, if body angles are within parameter range
    if (timeSinceSpecialAction <= timeForSpecialAction && timeSinceSpecialAction > startBalanceAfter
      && std::abs(theInertialSensorData.angle.y()) < maxAngleYForBalance && std::abs(theInertialSensorData.angle.x()) < maxAngleXForBalance)
    {
      // balance!
      wasInBalance = true;
    }
    else
    {
      wasInBalance = false;
      return;
    }
  }
  else
  {
    wasInBalance = false;
    timeWhenSpecialActionStarted = 0;
  }

  const BalanceParameters& currentBalanceParams = (isWalk && 
    (theSpeedInfo.speed.translation.norm() > 0.001f || std::abs(theSpeedInfo.speed.rotation) > 0.001f)) ? 
    theLegJointSensorControlParameters.walkBalanceParams : theLegJointSensorControlParameters.specialActionBalanceParams;

  PLOT("module:MotionCombinator:UpperBodyBalancer:gyroX",theInertialSensorData.gyro.x());
  PLOT("module:MotionCombinator:UpperBodyBalancer:gyroY",theInertialSensorData.gyro.y());
  PLOT("module:MotionCombinator:UpperBodyBalancer:angleX",theInertialSensorData.angle.x());
  PLOT("module:MotionCombinator:UpperBodyBalancer:angleY",theInertialSensorData.angle.y());

  if(currentBalanceParams.activateUpperBodyBalancing)
  {
    if(!currentBalanceParams.activateUpperBodyPID)
    {
        //old upper body balancing
        Angle targetAngle = currentBalanceParams.targetAngleX;
        Angle angleErrorY = (targetAngle - theInertialSensorData.angle.y());

        float gyroX = theInertialSensorData.gyro.x();
        float gyroY = theInertialSensorData.gyro.y();
        float comX = (newComX - lastComX);

        float filteredAngleOffsetX = gyroX * currentBalanceParams.gyroX_p  * (1 - currentBalanceParams.angleGyroRatioX) + theInertialSensorData.angle.x() * currentBalanceParams.angleX_p * currentBalanceParams.angleGyroRatioX;
        float filteredAngleOffsetY = gyroY * currentBalanceParams.gyroY_p + angleErrorY * currentBalanceParams.angleY_p + comX * currentBalanceParams.comX_p;
        // sine function like sensor control influence?
        // filteredAngleOffsetY *= std::sin(std::min(pi2, pi2*theInertialSensorData.angle.y()));
        filteredAngleOffsetX += theInertialSensorData.angle.x() * currentBalanceParams.angleX_i;
        filteredAngleOffsetY += angleErrorY * currentBalanceParams.angleY_i;
        filteredAngleOffsetX += gyroX * currentBalanceParams.gyroX_d;
        filteredAngleOffsetY += gyroY * currentBalanceParams.gyroY_d;

        PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetX",filteredAngleOffsetX);
        PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetY",filteredAngleOffsetY);

        JointRequest old;
        copy(jointRequest, old,Joints::lHipYawPitch);

        jointRequest.angles[Joints::lHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
        jointRequest.angles[Joints::rHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
        jointRequest.angles[Joints::lKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
        jointRequest.angles[Joints::rKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
        jointRequest.angles[Joints::lAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;
        jointRequest.angles[Joints::rAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;

        jointRequest.angles[Joints::lHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
        jointRequest.angles[Joints::rHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
        jointRequest.angles[Joints::lAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;
        jointRequest.angles[Joints::rAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;

        if (!jointRequest.isValid())
        {
          copy(old, jointRequest, Joints::lHipYawPitch);
          {
            std::string logDir = "";
      #ifdef TARGET_ROBOT
            logDir = "../logs/";
      #endif
            OutMapFile stream(logDir + "balancing.log");
            stream << filteredAngleOffsetX << "\n";
            stream << filteredAngleOffsetY << "\n";
          }
        }

        lastComX = newComX;
    }
    //upper body balancing with full PID controller
    else
    {
        Angle angleErrorX = (currentBalanceParams.targetAngleX - theInertialSensorData.angle.x());
        Angle angleErrorY = (currentBalanceParams.targetAngleY - theInertialSensorData.angle.y());

        float gyroX = theInertialSensorData.gyro.x();
        float gyroY = theInertialSensorData.gyro.y();

        pidGyroX_sum += gyroX;
        pidAngleX_sum += angleErrorX;
        pidGyroY_sum += gyroY;
        pidAngleY_sum += angleErrorY;

        float filteredAngleOffsetX_gyro = gyroX * currentBalanceParams.pidGyroX_p  + pidGyroX_sum * currentBalanceParams.pidGyroX_i + (pidGyroX_last - gyroX) * currentBalanceParams.pidGyroX_d;
        float filteredAngleOffsetX_angle = angleErrorX * currentBalanceParams.pidAngleX_p  + pidAngleX_sum * currentBalanceParams.pidAngleX_i + (pidAngleX_last - angleErrorX) * currentBalanceParams.pidAngleX_d;
        float filteredAngleOffsetX = filteredAngleOffsetX_gyro * (1 - currentBalanceParams.pidAngleGyroRatioX) + filteredAngleOffsetX_angle * currentBalanceParams.pidAngleGyroRatioX;


        float filteredAngleOffsetY_gyro = gyroY * currentBalanceParams.pidGyroY_p  + pidGyroY_sum * currentBalanceParams.pidGyroY_i + (pidGyroY_last - gyroY) * currentBalanceParams.pidGyroY_d;
        float filteredAngleOffsetY_angle = angleErrorY * currentBalanceParams.pidAngleY_p  + pidAngleY_sum * currentBalanceParams.pidAngleY_i + (pidAngleY_last - angleErrorY) * currentBalanceParams.pidAngleY_d;
        float filteredAngleOffsetY = filteredAngleOffsetY_gyro * (1 - currentBalanceParams.pidAngleGyroRatioY) + filteredAngleOffsetY_angle * currentBalanceParams.pidAngleGyroRatioY;

        PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetX",filteredAngleOffsetX);
        PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetY",filteredAngleOffsetY);

        pidAngleX_last = angleErrorX;
        pidGyroX_last = gyroX;
        pidAngleY_last = angleErrorY;
        pidGyroY_last = gyroY;

        JointRequest old;
        copy(jointRequest, old,Joints::lHipYawPitch);

        jointRequest.angles[Joints::lHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
        jointRequest.angles[Joints::rHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
        jointRequest.angles[Joints::lKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
        jointRequest.angles[Joints::rKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
        jointRequest.angles[Joints::lAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;
        jointRequest.angles[Joints::rAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;

        jointRequest.angles[Joints::lHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
        jointRequest.angles[Joints::rHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
        jointRequest.angles[Joints::lAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;
        jointRequest.angles[Joints::rAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;

        if (!jointRequest.isValid())
        {
          copy(old, jointRequest, Joints::lHipYawPitch);
          {
            std::string logDir = "";
      #ifdef TARGET_ROBOT
            logDir = "../logs/";
      #endif
            OutMapFile stream(logDir + "balancing.log");
            stream << filteredAngleOffsetX << "\n";
            stream << filteredAngleOffsetY << "\n";
          }
        }
        lastComX = newComX;
    }
  }
  //all sensors in one PID upper body balancing
  if(currentBalanceParams.activateSinglePIDAllSensors)
  {
      float filteredAngleOffsetX = currentBalanceParams.spidX_p * theInertialSensorData.angle.x() + currentBalanceParams.spidX_i * spid_sumX + currentBalanceParams.spidX_d * theInertialSensorData.gyro.x();
      float filteredAngleOffsetY = currentBalanceParams.spidY_p * theInertialSensorData.angle.y() + currentBalanceParams.spidY_i * spid_sumY + currentBalanceParams.spidY_d * theInertialSensorData.gyro.y();
      spid_sumX += theInertialData.angle.x();
      spid_sumY += theInertialData.angle.y();


      PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetX",filteredAngleOffsetX);
      PLOT("module:MotionCombinator:UpperBodyBalancer:filteredOffsetY",filteredAngleOffsetY);

      JointRequest old;
      copy(jointRequest, old,Joints::lHipYawPitch);

      jointRequest.angles[Joints::lHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
      jointRequest.angles[Joints::rHipPitch] += filteredAngleOffsetY * currentBalanceParams.hipPitchFactor;
      jointRequest.angles[Joints::lKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
      jointRequest.angles[Joints::rKneePitch] += std::abs(filteredAngleOffsetY) * currentBalanceParams.kneeFactor;
      jointRequest.angles[Joints::lAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;
      jointRequest.angles[Joints::rAnklePitch] += filteredAngleOffsetY * currentBalanceParams.footPitchFactor;

      jointRequest.angles[Joints::lHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
      jointRequest.angles[Joints::rHipRoll] += filteredAngleOffsetX * currentBalanceParams.hipRollFactor;
      jointRequest.angles[Joints::lAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;
      jointRequest.angles[Joints::rAnkleRoll] += filteredAngleOffsetX * currentBalanceParams.footRollFactor;

      if (!jointRequest.isValid())
      {
        copy(old, jointRequest, Joints::lHipYawPitch);
        {
          std::string logDir = "";
    #ifdef TARGET_ROBOT
          logDir = "../logs/";
    #endif
          OutMapFile stream(logDir + "balancing.log");
          stream << filteredAngleOffsetX << "\n";
          stream << filteredAngleOffsetY << "\n";
        }
      }
  }
}

void MotionCombinator::copy(const JointRequest& source, JointRequest& target,
  const Joints::Joint startJoint, const Joints::Joint endJoint) const
{
  for (int i = startJoint; i <= endJoint; ++i)
  {
    if (source.angles[i] != JointAngles::ignore)
      target.angles[i] = source.angles[i];
    target.stiffnessData.stiffnesses[i] = target.angles[i] != JointAngles::off ? source.stiffnessData.stiffnesses[i] : 0;
    if (target.stiffnessData.stiffnesses[i] == StiffnessData::useDefault)
      target.stiffnessData.stiffnesses[i] = theStiffnessSettings.stiffnesses[i];
  }
}

void MotionCombinator::interpolate(const JointRequest& from, const JointRequest& to,
  float fromRatio, JointRequest& target, bool interpolateStiffness,
  const Joints::Joint startJoint, const Joints::Joint endJoint) const
{
  for (int i = startJoint; i <= endJoint; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if (t == JointAngles::ignore && f == JointAngles::ignore)
      continue;

    if (t == JointAngles::ignore)
      t = target.angles[i];
    if (f == JointAngles::ignore)
      f = target.angles[i];

    int fStiffness = f != JointAngles::off ? from.stiffnessData.stiffnesses[i] : 0;
    int tStiffness = t != JointAngles::off ? to.stiffnessData.stiffnesses[i] : 0;
    if (fStiffness == StiffnessData::useDefault)
      fStiffness = theStiffnessSettings.stiffnesses[i];
    if (tStiffness == StiffnessData::useDefault)
      tStiffness = theStiffnessSettings.stiffnesses[i];

    if (t == JointAngles::off || t == JointAngles::ignore)
      t = lastJointAngles.angles[i];
    if (f == JointAngles::off || f == JointAngles::ignore)
      f = lastJointAngles.angles[i];
    if (target.angles[i] == JointAngles::off || target.angles[i] == JointAngles::ignore)
      target.angles[i] = lastJointAngles.angles[i];

    ASSERT(target.angles[i] != JointAngles::off && target.angles[i] != JointAngles::ignore);
    ASSERT(t != JointAngles::off && t != JointAngles::ignore);
    ASSERT(f != JointAngles::off && f != JointAngles::ignore);

    target.angles[i] += -fromRatio * t + fromRatio * f;
    if (interpolateStiffness)
      target.stiffnessData.stiffnesses[i] += int(-fromRatio * float(tStiffness) + fromRatio * float(fStiffness));
    else
      target.stiffnessData.stiffnesses[i] = tStiffness;
  }
}
