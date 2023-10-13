/**
 * @file Modules/MotionControl/JointErrorCalc.cpp
 * This file declares a module that calculates present joint error and join play error.
 *
 * @author Philip Reichenberg
 * @author <a href="mailto:mrunal.hatwar@tu-dortmund.de">Mrunal Hatwar</a>
 */

#include "JointErrorCalc.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Debugging/Annotation.h"


void JointErrorCalc::update(JointError& jointError)
{
  init(jointError);
  for (int j = 0; j < Joints::numOfJoints; j++)
    jointAngleBuffer[currentJointAngleID].angles[j] = theRawJointRequest.angles[j];
  currentJointAngleID++;
  currentJointAngleID = currentJointAngleID % 5;
  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    jointError.angles[i] = jointAngleBuffer[theWalkingEngineParams.jointSensorDelayFrames - 2].angles[i] - theJointSensorData.angles[i];
  }

  jointPlayCalc(jointError);
}

void JointErrorCalc::init(JointError& jointError)
{
  if (initialized == false)
  {
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < Joints::numOfJoints; j++)
        jointAngleBuffer[i].angles[j] = theRawJointRequest.angles[j];
    for (std::size_t i = 0; i < JointPlayTrack::numOfJointPlayTracks; i++)
    {
      bufferValue[i] = 0.f;
      // TODO is this even needed?
      bufferRequest[i].clear();
    }
    initialized = true;
  }
}

void JointErrorCalc::jointPlayCalc(JointError& jointError)
{
  const bool isWalkingNow = theMotionRequest.motion == MotionRequest::walk && theMotionInfo.motion == MotionRequest::walk && theGroundContactState.contact;
  if ((isWalkingNow && !isWalking) || startWalkingTimestamp > theFrameInfo.time)
    startWalkingTimestamp = theFrameInfo.time;
  isWalking = isWalkingNow;
  const bool skipBuffer = theFrameInfo.getTimeSince(startWalkingTimestamp) < minWalkTime || !isWalking;
  if (!skipBuffer)
    timeSpendWalking += theFrameInfo.cycleTime;
  const float ratio = Rangef::ZeroOneRange().limit(timeSpendWalking / interpolateLowpassFilterTime);
  const float useLowPassFilterFactor = lowpassFilterFactor.min * ratio + lowpassFilterFactor.max * (1.f - ratio);
  Angle jointPlaySum = 0.f;
  const float minSpeedRatio = minForwardSpeed / theWalkingEngineParams.speedLimits.xForward;
  const float speedRatio = theSpeedRequest.translation.norm() == 0 ? 0.f : (theSpeedRequest.translation.x()) / theWalkingEngineParams.speedLimits.xForward;
  const Angle jointPlayOffset = (1.f - Rangef::ZeroOneRange().limit((speedRatio - minSpeedRatio) / (1.f - minSpeedRatio))) * jointPlayScalingWalkingSpeed;
  // 2. Filter
  //FOREACH_ENUM(JointPlayTrack, joint)
  for (JointPlayTrack joint = JointPlayTrack::lhyp; joint < JointPlayTrack::numOfJointPlayTracks; joint = JointPlayTrack(joint + 1))
  {
    // Buffer joint request, because of motion delay
    bufferRequest[joint].push_front(theRawJointRequest.angles[getJoint(joint)]);
    if (!bufferRequest[joint].full())
      continue;
    if (skipBuffer)
      continue;
    // Filter differences
    // TODO Check if jointPlayList can be deleted. Seems to be just a useless copy
    const Angle useJointPlayOffset = joint == lap || joint == rap ? jointPlayOffset : 0_deg;
    bufferValue[joint] = bufferValue[joint] * (1.f - useLowPassFilterFactor)
        + (std::abs(theJointSensorData.angles[getJoint(joint)] - bufferRequest[joint].back()) - (maxJointPlay[joint] - useJointPlayOffset)) * useLowPassFilterFactor;
    bufferValueShortTerm[joint] = bufferValueShortTerm[joint] * (1.f - lowpassFilterFactor.max)
        + (std::abs(theJointSensorData.angles[getJoint(joint)] - bufferRequest[joint].back()) - (maxJointPlay[joint] - useJointPlayOffset)) * lowpassFilterFactor.max;

    jointPlayList[joint] = bufferValue[joint];
    jointPlaySum += std::max(jointPlayList[joint], 0_deg) * maxJointPlayRatio[joint];
  }

  if (!skipBuffer)
    jointError.qualityOfRobotHardware = 1.f - Rangef::ZeroOneRange().limit((jointPlaySum - jointPlayScalingmin) / jointPlayScalingmax);

  PLOT("module:JointErrorCalc:lhyp", jointPlayList[JointPlayTrack::lhyp].toDegrees());
  PLOT("module:JointErrorCalc:lhr", jointPlayList[JointPlayTrack::lhr].toDegrees());
  PLOT("module:JointErrorCalc:lhp", jointPlayList[JointPlayTrack::lhp].toDegrees());
  PLOT("module:JointErrorCalc:lkp", jointPlayList[JointPlayTrack::lkp].toDegrees());
  PLOT("module:JointErrorCalc:lap", jointPlayList[JointPlayTrack::lap].toDegrees());
  PLOT("module:JointErrorCalc:lar", jointPlayList[JointPlayTrack::lar].toDegrees());
  PLOT("module:JointErrorCalc:rhyp", jointPlayList[JointPlayTrack::rhyp].toDegrees());
  PLOT("module:JointErrorCalc:rhr", jointPlayList[JointPlayTrack::rhr].toDegrees());
  PLOT("module:JointErrorCalc:rhp", jointPlayList[JointPlayTrack::rhp].toDegrees());
  PLOT("module:JointErrorCalc:rkp", jointPlayList[JointPlayTrack::rkp].toDegrees());
  PLOT("module:JointErrorCalc:rap", jointPlayList[JointPlayTrack::rap].toDegrees());
  PLOT("module:JointErrorCalc:rar", jointPlayList[JointPlayTrack::rar].toDegrees());

  PLOT("module:JointErrorCalc:lhps", bufferValueShortTerm[JointPlayTrack::lhp].toDegrees());
  PLOT("module:JointErrorCalc:lkps", bufferValueShortTerm[JointPlayTrack::lkp].toDegrees());
  PLOT("module:JointErrorCalc:laps", bufferValueShortTerm[JointPlayTrack::lap].toDegrees());
  PLOT("module:JointErrorCalc:rhps", bufferValueShortTerm[JointPlayTrack::rhp].toDegrees());
  PLOT("module:JointErrorCalc:rkps", bufferValueShortTerm[JointPlayTrack::rkp].toDegrees());
  PLOT("module:JointErrorCalc:raps", bufferValueShortTerm[JointPlayTrack::rap].toDegrees());

  PLOT("module:JointErrorCalc:sum", jointPlaySum.toDegrees());
  PLOT("module:JointErrorCalc:qualityOfRobotHardware", jointError.qualityOfRobotHardware);
}

Joints::Joint JointErrorCalc::getJoint(JointPlayTrack joint)
{
  switch (joint)
  {
  case lhyp:
    return Joints::lHipYawPitch;
  case lhr:
    return Joints::lHipRoll;
  case lhp:
    return Joints::lHipPitch;
  case lkp:
    return Joints::lKneePitch;
  case lap:
    return Joints::lAnklePitch;
  case lar:
    return Joints::lAnkleRoll;
  case rhyp:
    return Joints::rHipYawPitch;
  case rhr:
    return Joints::rHipRoll;
  case rhp:
    return Joints::rHipPitch;
  case rkp:
    return Joints::rKneePitch;
  case rap:
    return Joints::rAnklePitch;
  case rar:
    return Joints::rAnkleRoll;
  default:
    ASSERT(false);
    return Joints::headYaw;
  }
}


MAKE_MODULE(JointErrorCalc, motionControl)
