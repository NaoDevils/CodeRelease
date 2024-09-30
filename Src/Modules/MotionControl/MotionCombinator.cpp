/**
* @file Modules/MotionControl/MotionCombinator.cpp
* This file implements a module that combines the motions created by the different modules.
* @author Dominik Brämer
* @author Diana Kleingarn
*/

#include "MotionCombinator.h"
#include "Tools/SensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"

using std::abs;

MAKE_MODULE(MotionCombinator, motionControl)

MotionCombinator::MotionCombinator() : theNonArmeMotionEngineOutput()
{
  currentRecoveryTime = recoveryTime + 1;
  headJawInSavePosition = false;
  headPitchInSavePosition = false;
  //lastJointRequest = theJointRequest;
}

void MotionCombinator::update(RawJointRequest& rawJointRequest)
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

  rawJointRequest.angles[Joints::headYaw] = theHeadJointRequest.pan;
  rawJointRequest.angles[Joints::headPitch] = theHeadJointRequest.tilt;

  copy(*jointRequests[theMotionSelection.targetMotion], rawJointRequest);

  if (freezeJointRequestOnError && !rawJointRequest.isValid())
  {
    ANNOTATION("MotionCombinator", "JointRequest invalid from motion type" << theMotionSelection.targetMotion);
    copy(lastJointRequest, rawJointRequest);
  }
  else
    ASSERT(rawJointRequest.isValid());

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
          theOdometryCorrectionTables.odometryCorrectionFactor,
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
      motionInfo.offsetToRobotPoseAfterPreview = OdometryCorrection::correct(theWalkingEngineOutput.speed,
          theWalkingEngineOutput.offsetToRobotPoseAfterPreview,
          theOdometryCorrectionTables.odometryCorrectionFactor,
          theOdometryCorrectionTables.backCorrectionTable,
          theOdometryCorrectionTables.forwardCorrectionTable,
          theOdometryCorrectionTables.sideCorrectionTable,
          theOdometryCorrectionTables.rotCorrectionTable,
          theOdometryCorrectionTables.rot2DCorrectionTable);
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

    // rotation will be applied later using IMU
    odometryOffset.rotation = 0_deg;

    odometryData += odometryOffset;
    ASSERT(rawJointRequest.isValid());
  }
  else // interpolate motions
  {
    const bool interpolateStiffness = !(theMotionSelection.targetMotion != MotionRequest::specialAction
        && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead && theMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
    for (int i = 0; i < MotionRequest::numOfMotions; ++i)
      if (i != theMotionSelection.targetMotion && theMotionSelection.ratios[i] > 0.)
      {
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], rawJointRequest, interpolateStiffness, Joints::headYaw, Joints::headPitch);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], rawJointRequest, interpolateStiffness, Joints::lShoulderPitch, Joints::lHand);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], rawJointRequest, interpolateStiffness, Joints::rShoulderPitch, Joints::rHand);
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], rawJointRequest, interpolateStiffness, Joints::lHipYawPitch, Joints::rAnkleRoll);
      }
  }

  // Use 3D rotation directly from IMU model if enabled
  if (anglesource == JoinedIMUData::InertialDataSource::imuModel)
  {
    odometryData.rotation = theIMUModel.rotation.z();
  }
  else
  {
    RotationMatrix rot(theJoinedIMUData.imuData[anglesource].angle.x(), theJoinedIMUData.imuData[anglesource].angle.y(), 0_deg);
    Vector3f rotatedGyros = rot * theJoinedIMUData.imuData[anglesource].gyro.cast<float>();
    odometryData.rotation += rotatedGyros.z() * theFrameInfo.cycleTime;
  }

  ASSERT(rawJointRequest.isValid());

  float sum(0);
  int count(0);
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if (rawJointRequest.angles[i] != JointAngles::off && rawJointRequest.angles[i] != JointAngles::ignore && lastJointRequest.angles[i] != JointAngles::off
        && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(rawJointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
      float angleChange = abs(rawJointRequest.angles[i] - lastJointRequest.angles[i]);
      if (angleChange > degPerFrame[i] && theMotionSelection.targetMotion != MotionRequest::walk)
        if (textOutput == true)
          OUTPUT_TEXT("Angle increment for joint" << i << " is greater than allowed in motion" << theMotionSelection.targetMotion);
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:legsOnly", sum / count);
  for (int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if (rawJointRequest.angles[i] != JointAngles::off && rawJointRequest.angles[i] != JointAngles::ignore && lastJointRequest.angles[i] != JointAngles::off
        && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += abs(rawJointRequest.angles[i] - lastJointRequest.angles[i]);
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
  if (!rawJointRequest.isValid())
  {
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "jointRequest.log");
      stream << rawJointRequest;
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
      if (lastJointRequest.angles[i] == JointAngles::off || lastJointRequest.angles[i] == JointAngles::ignore || rawJointRequest.angles[i] == JointAngles::off
          || rawJointRequest.angles[i] == JointAngles::ignore)
        continue;
      if (rawJointRequest.angles[i] > theJointCalibration.joints[i].maxAngle && rawJointRequest.angles[i] > theJointSensorData.angles[i])
      {
        if (theMotionSelection.ratios[MotionRequest::specialAction] != 1.f || lastSpecialAction != SpecialActionRequest::playDead)
          OUTPUT_WARNING("Joint limit broken for joint " << Joints::getName((Joints::Joint)i) << ", " << rawJointRequest.angles[i] << " requested");
        rawJointRequest.angles[i] = theJointCalibration.joints[i].maxAngle;
      }
      else if (rawJointRequest.angles[i] < theJointCalibration.joints[i].minAngle && rawJointRequest.angles[i] < theJointSensorData.angles[i])
      {
        if (theMotionSelection.ratios[MotionRequest::specialAction] != 1.f || lastSpecialAction != SpecialActionRequest::playDead)
          OUTPUT_WARNING("Joint limit broken for joint " << Joints::getName((Joints::Joint)i) << ", " << rawJointRequest.angles[i] << " requested");
        rawJointRequest.angles[i] = theJointCalibration.joints[i].minAngle;
      }
      Angle jointDiff = rawJointRequest.angles[i] - lastJointRequest.angles[i];
      Angle jointAcc = jointDiff - lastJointDiff.angles[i];
      if (jointAcc > maxJointAccelerationPerFrame)
        rawJointRequest.angles[i] = lastJointRequest.angles[i] + jointDiff - (jointAcc - maxJointAccelerationPerFrame);
      else if (jointAcc < -maxJointAccelerationPerFrame)
        rawJointRequest.angles[i] = lastJointRequest.angles[i] + jointDiff - (jointAcc + maxJointAccelerationPerFrame);
      lastJointDiff.angles[i] = rawJointRequest.angles[i] - lastJointRequest.angles[i];
    }
  }

#endif
  lastJointRequest = rawJointRequest;

  // compensate joint error by increasing stiffness
  if (useDynamicStiffness)
  {
    for (int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; i++)
    {
      if (rawJointRequest.stiffnessData.stiffnesses[i] > 0)
      {
        rawJointRequest.stiffnessData.stiffnesses[i] += static_cast<int>(std::abs(theJointError.angles[i].toDegrees() * 2.f));
        rawJointRequest.stiffnessData.stiffnesses[i] = std::min(100, rawJointRequest.stiffnessData.stiffnesses[i]);
      }
    }
  }

  PLOT("module:MotionCombinator:leftLegAngles[0]", rawJointRequest.angles[Joints::lHipYawPitch + 0].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[1]", rawJointRequest.angles[Joints::lHipYawPitch + 1].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[2]", rawJointRequest.angles[Joints::lHipYawPitch + 2].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[3]", rawJointRequest.angles[Joints::lHipYawPitch + 3].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[4]", rawJointRequest.angles[Joints::lHipYawPitch + 4].toDegrees());
  PLOT("module:MotionCombinator:leftLegAngles[5]", rawJointRequest.angles[Joints::lHipYawPitch + 5].toDegrees());

  PLOT("module:MotionCombinator:rightLegAngles[0]", rawJointRequest.angles[Joints::lHipYawPitch + 6].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[1]", rawJointRequest.angles[Joints::lHipYawPitch + 7].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[2]", rawJointRequest.angles[Joints::lHipYawPitch + 8].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[3]", rawJointRequest.angles[Joints::lHipYawPitch + 9].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[4]", rawJointRequest.angles[Joints::lHipYawPitch + 10].toDegrees());
  PLOT("module:MotionCombinator:rightLegAngles[5]", rawJointRequest.angles[Joints::lHipYawPitch + 11].toDegrees());

  PLOT("module:MotionCombinator:leftLegStiffness[0]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 0]);
  PLOT("module:MotionCombinator:leftLegStiffness[1]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 1]);
  PLOT("module:MotionCombinator:leftLegStiffness[2]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 2]);
  PLOT("module:MotionCombinator:leftLegStiffness[3]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 3]);
  PLOT("module:MotionCombinator:leftLegStiffness[4]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 4]);
  PLOT("module:MotionCombinator:leftLegStiffness[5]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 5]);

  PLOT("module:MotionCombinator:rightLegStiffness[0]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 6]);
  PLOT("module:MotionCombinator:rightLegStiffness[1]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 7]);
  PLOT("module:MotionCombinator:rightLegStiffness[2]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 8]);
  PLOT("module:MotionCombinator:rightLegStiffness[3]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 9]);
  PLOT("module:MotionCombinator:rightLegStiffness[4]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 10]);
  PLOT("module:MotionCombinator:rightLegStiffness[5]", rawJointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch + 11]);

  if (motionInfo.specialActionRequest.specialAction == SpecialActionRequest::testUnstiff)
  {
    for (int i = 0; i < Joints::numOfJoints; i++)
    {
      rawJointRequest.angles[i] = JointSensorData::off;
    }
  }
}

void MotionCombinator::update(OdometryData& odometryData)
{
  this->odometryData.rotation.normalize();

  odometryData = this->odometryData;

  Pose2f odometryOffset(odometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionCombinator:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionCombinator:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionCombinator:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = odometryData;
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
