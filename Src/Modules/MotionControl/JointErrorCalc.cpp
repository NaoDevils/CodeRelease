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
  DECLARE_PLOT("module:JointErrorCalc:sum");

  init(jointError);
  for (int j = 0; j < Joints::numOfJoints; j++)
    jointAngleBuffer[currentJointAngleID].angles[j] = theRawJointRequest.angles[j];
  currentJointAngleID++;
  currentJointAngleID = currentJointAngleID % 5;
  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    jointError.angles[i] = jointAngleBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1].angles[i] - theJointSensorData.angles[i];
    if (std::abs(jointError.angles[i]) < 0.1_deg)
    {
      jointError.angles[i] = 0_deg;
    }
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
  const bool isWalkingNow = theMotionRequest.motion == MotionRequest::walk && (theSpeedRequest.translation.norm() > 0.f || std::abs(theSpeedRequest.rotation) > 0.f)
      && theFallDownState.state == FallDownState::upright;

  if ((isWalkingNow && !isWalking) || startWalkingTimestamp > theFrameInfo.time)
    startWalkingTimestamp = theFrameInfo.time;
  isWalking = isWalkingNow;
  const bool skipBuffer = theFrameInfo.getTimeSince(startWalkingTimestamp) < minWalkTime || !isWalking;
  if (!skipBuffer)
    jointError.timeSpendWalking += theFrameInfo.cycleTime;
  const float ratio = Rangef::ZeroOneRange().limit(jointError.timeSpendWalking / interpolateLowpassFilterTime);
  const float useLowPassFilterFactor = lowpassFilterFactor.min * ratio + lowpassFilterFactor.max * (1.f - ratio);
  Angle jointPlaySum = 0_deg;
  const float minSpeedRatio = minForwardSpeed / theWalkingEngineParams.speedLimits.xForward;
  const float speedRatio = theSpeedRequest.translation.norm() == 0 ? 0.f : (theSpeedRequest.translation.x()) / theWalkingEngineParams.speedLimits.xForward;
  const Angle jointPlayOffset = (1.f - Rangef::ZeroOneRange().limit((speedRatio - minSpeedRatio) / (1.f - minSpeedRatio + std::numeric_limits<float>::epsilon()))) * jointPlayScalingWalkingSpeed;
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
    const Angle jointPlay = std::max(jointPlayList[joint], 0_deg) * maxJointPlayRatio[joint];
    setJointPlayAvg(jointError, joint, jointPlay);
    jointPlaySum += jointPlay;
    ASSERT(!std::isnan(jointPlaySum));
  }

  if (!skipBuffer)
  {
    PLOT("module:JointErrorCalc:sum", jointPlaySum.toDegrees());
    jointError.qualityOfRobotHardware = 1.f - Rangef::ZeroOneRange().limit((jointPlaySum - jointPlayScalingMin) / jointPlayScalingMax);
  }


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

  PLOT("module:JointErrorCalc:qualityOfRobotHardware", jointError.qualityOfRobotHardware);
}

void JointErrorCalc::setJointPlayAvg(JointError& jointError, JointPlayTrack joint, Angle avgJointPlay)
{
  switch (joint)
  {
  case lhyp:
    jointError.averageJointPlay.lHipYawPitch = avgJointPlay;
    break;
  case lhr:
    jointError.averageJointPlay.lHipRoll = avgJointPlay;
    break;
  case lhp:
    jointError.averageJointPlay.lHipPitch = avgJointPlay;
    break;
  case lkp:
    jointError.averageJointPlay.lKneePitch = avgJointPlay;
    break;
  case lap:
    jointError.averageJointPlay.lAnklePitch = avgJointPlay;
    break;
  case lar:
    jointError.averageJointPlay.lAnkleRoll = avgJointPlay;
    break;
  case rhyp:
    jointError.averageJointPlay.rHipYawPitch = avgJointPlay;
    break;
  case rhr:
    jointError.averageJointPlay.rHipRoll = avgJointPlay;
    break;
  case rhp:
    jointError.averageJointPlay.rHipPitch = avgJointPlay;
    break;
  case rkp:
    jointError.averageJointPlay.rKneePitch = avgJointPlay;
    break;
  case rap:
    jointError.averageJointPlay.rAnklePitch = avgJointPlay;
    break;
  case rar:
    jointError.averageJointPlay.rAnkleRoll = avgJointPlay;
    break;
  default:
    ASSERT(false);
  }
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
