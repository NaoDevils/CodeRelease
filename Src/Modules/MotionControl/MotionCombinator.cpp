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
#include "Tools/Debugging/Annotation.h"

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
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Ankle");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Ankle");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:errorX_Hip");
  DECLARE_PLOT("module:MotionCombinator:UpperBodyBalancer:errorY_Hip");

  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::Motion::walk] = &theWalkingEngineOutput;
  jointRequests[MotionRequest::Motion::kick] = &theKickEngineOutput;
  jointRequests[MotionRequest::Motion::specialAction] = &theSpecialActionsOutput;
  jointRequests[MotionRequest::Motion::stand] = &theStandEngineOutput;

  jointRequest.angles[Joints::headYaw] = theHeadJointRequest.pan;
  jointRequest.angles[Joints::headPitch] = theHeadJointRequest.tilt;

  copy(*jointRequests[theMotionSelection.targetMotion], jointRequest);

  if (freezeJointRequestOnError && !jointRequest.isValid())
  {
    ANNOTATION("MotionCombinator", "JointRequest invalid from motion type" << theMotionSelection.targetMotion);
    copy(lastJointRequest, jointRequest);
  }
  else
    ASSERT(jointRequest.isValid());

  motionInfo.customStepKickInPreview = theSpeedInfo.customStepKickInPreview;
  motionInfo.walkKicking = theFootpositions.inKick;

  // Find fully active motion and set MotionInfo
  if (theMotionSelection.ratios[theMotionSelection.targetMotion] == 1.f)
  {
    Pose2f odometryOffset;
    // default values
    motionInfo.motion = theMotionSelection.targetMotion;
    motionInfo.isMotionStable = true;
    motionInfo.offsetToRobotPoseAfterPreview = Pose2f();

    lastJointAngles = theJointSensorData;

    switch (theMotionSelection.targetMotion)
    {
    case MotionRequest::walk:
      odometryOffset = OdometryCorrection::correct(theWalkingEngineOutput.speed,
          theWalkingEngineOutput.odometryOffset,
          theOdometryCorrectionTables.backCorrectionTable,
          theOdometryCorrectionTables.forwardCorrectionTable,
          theOdometryCorrectionTables.sideCorrectionTable,
          theOdometryCorrectionTables.rotCorrectionTable,
          theOdometryCorrectionTables.rot2DCorrectionTable);
      motionInfo.walkRequest.request = theWalkingEngineOutput.speed;
      motionInfo.walkRequest.requestType = WalkRequest::speed;
      motionInfo.walkRequest.stepRequest = theSpeedInfo.currentCustomStep;
      motionInfo.kickRequest.kickTarget = theMotionRequest.kickRequest.kickTarget;
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.x", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.x());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.y", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.translation.y());
      PLOT("module:MotionCombinator:offsetToRobotPoseAfterPreview.r", theWalkingEngineOutput.offsetToRobotPoseAfterPreview.rotation);
      motionInfo.offsetToRobotPoseAfterPreview = OdometryCorrection::correctPreview(theWalkingEngineOutput.speed,
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
      motionInfo.kickRequest.kickTarget = theMotionRequest.kickRequest.kickTarget;
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

    if (theRobotInfo.hasFeature(RobotInfo::zGyro))
    {
      RotationMatrix rot(theJoinedIMUData.imuData[anglesource].angle.x(), theJoinedIMUData.imuData[anglesource].angle.y(), 0_deg);
      Vector3f rotatedGyros = rot * theJoinedIMUData.imuData[anglesource].gyro.cast<float>();
      odometryOffset.rotation = rotatedGyros.z() * theFrameInfo.cycleTime;
    }

    odometryData += odometryOffset;
    ASSERT(jointRequest.isValid());
  }
  else // interpolate motions
  {
    const bool interpolateStiffness = !(theMotionSelection.targetMotion != MotionRequest::specialAction
        && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead && theMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
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

  if (emergencyOffEnabled)
  {
    if (theFallDownState.state == FallDownState::falling && motionInfo.motion != MotionRequest::specialAction)
    {
      saveFall(jointRequest);
      centerHead(jointRequest);
      saveArms(jointRequest); //centerArms(jointRequest);
      fallingFrame++;
      currentRecoveryTime = 0;

      ASSERT(jointRequest.isValid());
    }
    else if ((motionInfo.motion == MotionRequest::stand) && theFallDownState.state == FallDownState::onGround) // when fixed stand request and no stand up
    {
      saveFall(jointRequest);
      centerHead(jointRequest);
      saveArms(jointRequest);
    }
    else if ((theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::onGround) && (motionInfo.motion != MotionRequest::specialAction))
    {
      centerHead(jointRequest);
      if (theFallDownState.state == FallDownState::onGround && theFallDownState.direction == FallDownState::back) //check arm position
      {
        if (((120_deg - JointDiffArmsStuck) > theJointSensorData.angles[Joints::lShoulderPitch]) || ((120_deg - JointDiffArmsStuck) > theJointSensorData.angles[Joints::rShoulderPitch]))
        {
          ANNOTATION("MotionCombinator", "arms stuck");
          jointRequest.stiffnessData.stiffnesses[Joints::lShoulderPitch] = 90;
          jointRequest.stiffnessData.stiffnesses[Joints::rShoulderPitch] = 90;
          jointRequest.stiffnessData.stiffnesses[Joints::lElbowRoll] = 70;
          jointRequest.stiffnessData.stiffnesses[Joints::rElbowRoll] = 70;
          armsStand(jointRequest);
        }
      }
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
      sum += abs(lastJointRequest.angles[i] - theJointSensorData.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:differenceToJointData:legsOnly", sum / count);

  for (int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if (lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(lastJointRequest.angles[i] - theJointSensorData.angles[i]);
      count++;
    }
  }
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

  Angle maxJointAccelerationPerFrame = theFrameInfo.cycleTime * maxJointAcceleration;
  if (theMotionSelection.targetMotion == MotionRequest::specialAction && theMotionSelection.specialActionRequest.specialAction != currentSpecialAction)
  {
    lastSpecialAction = currentSpecialAction;
    currentSpecialAction = theMotionSelection.specialActionRequest.specialAction;
  }
  if (useJointAccLimit)
  {
    for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
    {
      if (lastJointRequest.angles[i] == JointAngles::off || lastJointRequest.angles[i] == JointAngles::ignore || jointRequest.angles[i] == JointAngles::off
          || jointRequest.angles[i] == JointAngles::ignore)
        continue;
      if (jointRequest.angles[i] > theJointCalibration.joints[i].maxAngle && jointRequest.angles[i] > theJointSensorData.angles[i])
      {
        if (theMotionSelection.ratios[MotionRequest::specialAction] != 1.f || lastSpecialAction != SpecialActionRequest::playDead)
          OUTPUT_WARNING("Joint limit broken for joint " << Joints::getName((Joints::Joint)i) << ", " << jointRequest.angles[i] << " requested");
        jointRequest.angles[i] = theJointCalibration.joints[i].maxAngle;
      }
      else if (jointRequest.angles[i] < theJointCalibration.joints[i].minAngle && jointRequest.angles[i] < theJointSensorData.angles[i])
      {
        if (theMotionSelection.ratios[MotionRequest::specialAction] != 1.f || lastSpecialAction != SpecialActionRequest::playDead)
          OUTPUT_WARNING("Joint limit broken for joint " << Joints::getName((Joints::Joint)i) << ", " << jointRequest.angles[i] << " requested");
        jointRequest.angles[i] = theJointCalibration.joints[i].minAngle;
      }
      Angle jointDiff = jointRequest.angles[i] - lastJointRequest.angles[i];
      Angle jointAcc = jointDiff - lastJointDiff.angles[i];
      if (jointAcc > maxJointAccelerationPerFrame)
        jointRequest.angles[i] = lastJointRequest.angles[i] + jointDiff - (jointAcc - maxJointAccelerationPerFrame);
      else if (jointAcc < -maxJointAccelerationPerFrame)
        jointRequest.angles[i] = lastJointRequest.angles[i] + jointDiff - (jointAcc + maxJointAccelerationPerFrame);
      lastJointDiff.angles[i] = jointRequest.angles[i] - lastJointRequest.angles[i];
    }
  }

#endif
  lastJointRequest = jointRequest;

  PLOT("module:MotionCombinator:leftLegAngles[0]", jointRequest.angles[Joints::lHipYawPitch + 0].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[1]", jointRequest.angles[Joints::lHipYawPitch + 1].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[2]", jointRequest.angles[Joints::lHipYawPitch + 2].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[3]", jointRequest.angles[Joints::lHipYawPitch + 3].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[4]", jointRequest.angles[Joints::lHipYawPitch + 4].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[5]", jointRequest.angles[Joints::lHipYawPitch + 5].toDegrees());

  PLOT("module:MotionCombinator:rightLegAngles[0]", jointRequest.angles[Joints::lHipYawPitch + 6].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[1]", jointRequest.angles[Joints::lHipYawPitch + 7].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[2]", jointRequest.angles[Joints::lHipYawPitch + 8].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[3]", jointRequest.angles[Joints::lHipYawPitch + 9].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[4]", jointRequest.angles[Joints::lHipYawPitch + 10].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[5]", jointRequest.angles[Joints::lHipYawPitch + 11].toDegrees());

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

  // check last stand up motion
  if (motionInfo.specialActionRequest.specialAction == theMotionSettings.standUpMotionBack)
  {
    if (theMotionSelection.ratios[MotionRequest::specialAction] == 0.f && theFallDownState.state == FallDownState::upright)
      motionInfo.lastStandUpSafeBack = true;
    else if (theMotionSelection.ratios[MotionRequest::specialAction] != 0.f)
      motionInfo.lastStandUpSafeBack = false;
  }
  else
    motionInfo.lastStandUpSafeBack = true;

  if (motionInfo.specialActionRequest.specialAction == theMotionSettings.standUpMotionFront)
  {
    if (theMotionSelection.ratios[MotionRequest::specialAction] == 0.f && theFallDownState.state == FallDownState::upright)
      motionInfo.lastStandUpSafeFront = true;
    else if (theMotionSelection.ratios[MotionRequest::specialAction] != 0.f)
      motionInfo.lastStandUpSafeFront = false;
  }
  else
    motionInfo.lastStandUpSafeFront = true;
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
{ // set stiffness for all joints
  for (int i = 0; i < Joints::firstArmJoint; i++)
    jointRequest.stiffnessData.stiffnesses[i] = 30;
  for (int i = Joints::firstArmJoint; i < Joints::numOfJoints; i++)
    jointRequest.stiffnessData.stiffnesses[i] = 5;

  if (theFallDownState.direction == FallDownState::front)
    sitFront(jointRequest);
  else
    sit(jointRequest);
}

void MotionCombinator::sitFront(JointRequest& jointRequest)
{
  jointRequest.angles[Joints::lKneePitch] = 123_deg;
  jointRequest.angles[Joints::rKneePitch] = 123_deg;
  jointRequest.angles[Joints::lHipPitch] = -24_deg;
  jointRequest.angles[Joints::rHipPitch] = -24_deg;
  jointRequest.angles[Joints::lHipRoll] = 0_deg;
  jointRequest.angles[Joints::rHipRoll] = 0_deg;
  jointRequest.angles[Joints::lAnklePitch] = -75_deg;
  jointRequest.angles[Joints::rAnklePitch] = -75_deg;
  jointRequest.angles[Joints::lAnkleRoll] = 0_deg;
  jointRequest.angles[Joints::rAnkleRoll] = 0_deg;
}
void MotionCombinator::sit(JointRequest& jointRequest)
{
  // Sit down to reduce the impact-force
  jointRequest.angles[Joints::lKneePitch] = 123_deg;
  jointRequest.angles[Joints::rKneePitch] = 123_deg;
  jointRequest.angles[Joints::lHipPitch] = -90_deg;
  jointRequest.angles[Joints::rHipPitch] = -90_deg;
  jointRequest.angles[Joints::lHipRoll] = 0_deg;
  jointRequest.angles[Joints::rHipRoll] = 0_deg;
  jointRequest.angles[Joints::lAnklePitch] = -45_deg;
  jointRequest.angles[Joints::rAnklePitch] = -45_deg;
  jointRequest.angles[Joints::lAnkleRoll] = 0_deg;
  jointRequest.angles[Joints::rAnkleRoll] = 0_deg;
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

void MotionCombinator::saveArms(JointRequest& jointRequest)
{
  if (!isFallingStarted)
  {
    isFallingStarted = true;
    fallingFrame = 0;
  }
  centerArm(jointRequest, true);
  centerArm(jointRequest, false);
}

void MotionCombinator::armsFront(JointRequest& jointRequest)
{ // copy B-Human
  jointRequest.angles[Joints::lShoulderRoll] = 10_deg;
  jointRequest.angles[Joints::lShoulderPitch] = 20_deg;
  jointRequest.angles[Joints::lElbowYaw] = 100_deg;
  jointRequest.angles[Joints::lElbowRoll] = -60_deg;
  jointRequest.angles[Joints::lWristYaw] = -90_deg;
  jointRequest.angles[Joints::rShoulderRoll] = -10_deg;
  jointRequest.angles[Joints::rShoulderPitch] = 20_deg;
  jointRequest.angles[Joints::rElbowYaw] = -100_deg;
  jointRequest.angles[Joints::rElbowRoll] = 60_deg;
  jointRequest.angles[Joints::rWristYaw] = 90_deg;
  /*jointRequest.angles[Joints::rShoulderRoll] = -7_deg;
  jointRequest.angles[Joints::rShoulderPitch] = 60_deg;
  jointRequest.angles[Joints::rElbowYaw] = -96_deg;
  jointRequest.angles[Joints::rElbowRoll] = 0_deg;
  jointRequest.angles[Joints::rWristYaw] = 90_deg;*/
}
void MotionCombinator::armsBehind(JointRequest& jointRequest)
{ // copy B-Human
  jointRequest.angles[Joints::lShoulderRoll] = 8_deg;
  jointRequest.angles[Joints::lShoulderPitch] = 123_deg;
  jointRequest.angles[Joints::lElbowYaw] = 17_deg;
  jointRequest.angles[Joints::lElbowRoll] = -78_deg;
  jointRequest.angles[Joints::lWristYaw] = -90_deg;
  jointRequest.angles[Joints::rShoulderRoll] = -8_deg;
  jointRequest.angles[Joints::rShoulderPitch] = 123_deg;
  jointRequest.angles[Joints::rElbowYaw] = -17_deg;
  jointRequest.angles[Joints::rElbowRoll] = 78_deg;
  jointRequest.angles[Joints::rWristYaw] = 90_deg;
  jointRequest.stiffnessData.stiffnesses[Joints::lElbowRoll] = 20;
  jointRequest.stiffnessData.stiffnesses[Joints::rElbowRoll] = 20;
}
void MotionCombinator::armsStand(JointRequest& jointRequest)
{ // copy B-Human
  jointRequest.angles[Joints::lShoulderRoll] = 12_deg;
  jointRequest.angles[Joints::lShoulderPitch] = 90_deg;
  jointRequest.angles[Joints::lElbowYaw] = 90_deg;
  jointRequest.angles[Joints::lElbowRoll] = 0_deg;
  jointRequest.angles[Joints::lWristYaw] = -90_deg;
  jointRequest.angles[Joints::rShoulderRoll] = -12_deg;
  jointRequest.angles[Joints::rShoulderPitch] = 90_deg;
  jointRequest.angles[Joints::rElbowYaw] = -90_deg;
  jointRequest.angles[Joints::rElbowRoll] = 0_deg;
  jointRequest.angles[Joints::rWristYaw] = 90_deg;
}


void MotionCombinator::centerArm(JointRequest& jointRequest, bool left)
{
  const int sign(-1 + 2 * left); // sign = 1 for left arm, for right its -1
  int i(left ? Joints::lShoulderPitch : Joints::rShoulderPitch);
  int j(i);

  if (fallingFrame < durationCenterArmFirstPhase)
  {
    jointRequest.angles[i++] = 119.5_deg;
    jointRequest.angles[i++] = sign * 25_deg;
    jointRequest.angles[i++] = sign * 45_deg;
    jointRequest.angles[i++] = sign * -11.5_deg;

    while (j < i)
      jointRequest.stiffnessData.stiffnesses[j++] = 100;
  }
  else if (fallingFrame < durationCenterArmFirstPhase + durationCenterArmSecondPhase)
  {
    jointRequest.angles[i++] = 90_deg;
    jointRequest.angles[i++] = sign * 11.5_deg;
    jointRequest.angles[i++] = sign * -90_deg;
    jointRequest.angles[i++] = sign * -11.5_deg;

    while (j < i)
      jointRequest.stiffnessData.stiffnesses[j++] = 100;
  }
  else
  {
    if (theFallDownState.direction == FallDownState::front)
    {
      armsFront(jointRequest);
    }
    else if (theFallDownState.direction == FallDownState::back)
    {
      armsBehind(jointRequest);
    }
    if (fallingFrame < durationCenterArmFirstPhase + durationCenterArmSecondPhase + durationCenterArmThirdPhase)
      while (j < i)
        jointRequest.stiffnessData.stiffnesses[j++] = 100;
    else
      while (i < j)
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
  else // direction unknown
    jointRequest.angles[Joints::headPitch] = 0; //ok?

  if (abs(theJointSensorData.angles[Joints::headYaw]) > 0.1f && !headJawInSavePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 100;
  else
  {
    headJawInSavePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 25;
  }

  if (abs(theJointSensorData.angles[Joints::headPitch] - jointRequest.angles[Joints::headPitch]) > 0.1f && !headPitchInSavePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 100;
  else
  {
    headPitchInSavePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 50;
  }
}

void MotionCombinator::copy(const JointRequest& source, JointRequest& target, const Joints::Joint startJoint, const Joints::Joint endJoint) const
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

void MotionCombinator::interpolate(
    const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateStiffness, const Joints::Joint startJoint, const Joints::Joint endJoint) const
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
