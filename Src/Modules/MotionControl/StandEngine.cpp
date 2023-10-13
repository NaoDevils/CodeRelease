/**
 * @file StandEngine.cpp
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "StandEngine.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"
#include "Tools/Build.h"
#include "Platform/File.h"

MAKE_MODULE(StandEngine, motionControl)

StandEngine::StandEngine()
{
  if constexpr (Build::targetRobot())
  {
    InTextFile standEngineOffsetFile(File::getPersistentDir() + "standEngine.value");
    if (standEngineOffsetFile.exists())
      standEngineOffsetFile >> targetAngleOffset;
  }
}

StandEngine::~StandEngine()
{
  if constexpr (Build::targetRobot())
  {
    OutTextFile standEngineOffsetFile(File::getPersistentDir() + "standEngine.value");
    if (standEngineOffsetFile.exists())
      standEngineOffsetFile << targetAngleOffset;
  }
}

void StandEngine::update(StandEngineOutput& jointRequest)
{
  DECLARE_DEBUG_RESPONSE("module:StandEngine:ignoreFsr");
  DECLARE_PLOT("module:StandEngine:angleCorrection");
  DECLARE_PLOT("module:StandEngine:angleError");
  DECLARE_PLOT("module:StandEngine:pitchOffset");

  MODIFY("module:StandEngine:targetAngleOffset", targetAngleOffset);

  float fsrSum = theFsrSensorData.leftTotal + theFsrSensorData.rightTotal;
  if (fsrMin > fsrSum)
    fsrMin = fsrSum;
  if (fsrMax < fsrSum)
    fsrMax = fsrSum;

  jointRequest.stiffnessTransition = 0.f;

  jointRequest.angles[Joints::headPitch] = JointAngles::ignore;
  jointRequest.angles[Joints::headYaw] = JointAngles::ignore;
  jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = StiffnessData::useDefault;
  jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = StiffnessData::useDefault;

  for (int i = Joints::firstArmJoint; i < Joints::numOfJoints; ++i)
  {
    jointRequest.angles[i] = jointAngles[i];
    jointRequest.stiffnessData.stiffnesses[i] = defaultStiffnesses[i];
  }

  // reset pitchOffset if module is inactive
  if (theMotionSelection.ratios[MotionRequest::Motion::stand] == 0.f)
  {
    pitchOffset = 0_deg;
  }

  if (theRobotInfo.transitionToFramework < 1.f)
  {
    angleSum = 0_deg;
    angleOldError = 0_deg;
    jointRequest.isLeavingPossible = true;
    return;
  }

  Angle angleError = (targetAngle + targetAngleOffset) - (theJoinedIMUData.imuData[anglesource].angle.y());

  // sum error as long as robot has stiffness
  if (stablePositionTimestamp == 0 || theFrameInfo.getTimeSince(stablePositionTimestamp) <= transitionStart + transitionTime)
  {
    angleSum += angleError;
  }

  // reset sum and old error if module is not fully active
  if (theMotionSelection.ratios[MotionRequest::Motion::stand] < 1.f)
  {
    angleSum = 0_deg;
    angleOldError = 0_deg;
  }

  // calc pid
  Angle angleCorrection = angleYpid[0] * angleError + angleYpid[1] * angleSum + angleYpid[2] * (angleError - angleOldError);


  bool positionOkay = theMotionRequest.motion == MotionRequest::Motion::stand;

  // apply sensor control if stiffness is on and module is active
  if (positionOkay && theMotionSelection.ratios[MotionRequest::Motion::stand] == 1.f
      && (stablePositionTimestamp == 0 || theFrameInfo.getTimeSince(stablePositionTimestamp) <= transitionStart + transitionTime))
  {
    angleOldError = angleError;

    pitchOffset += angleCorrection;

    pitchOffset = std::min(pitchOffset, pitchOffsetMax);
    pitchOffset = std::max(pitchOffset, pitchOffsetMin);

    // leaving is not possible if pitchOffset is not 0
    jointRequest.isLeavingPossible = false;
  }

  DEBUG_RESPONSE_NOT("module:StandEngine:ignoreFsr")
  {
    // enable stiffness if no ground contact
    if ((fsrSum - fsrMin) < 1.f)
    {
      angleSum = 0_deg;
      angleOldError = 0_deg;
      if (pitchOffset > 0.1_deg)
        pitchOffset -= 0.04_deg;
      else if (pitchOffset < -0.1_deg)
        pitchOffset += 0.04_deg;
      positionOkay = false;
    }
  }

  PLOT("module:StandEngine:angleCorrection", angleCorrection);
  PLOT("module:StandEngine:angleError", angleError);
  PLOT("module:StandEngine:pitchOffset", pitchOffset);

  // apply offset
  jointRequest.angles[Joints::rAnklePitch] += pitchOffset / 2.f;
  jointRequest.angles[Joints::lAnklePitch] += pitchOffset / 2.f;
  jointRequest.angles[Joints::rHipPitch] += pitchOffset / 2.f;
  jointRequest.angles[Joints::lHipPitch] += pitchOffset / 2.f;

  // enable stiffness if body angle is out of limits
  if (std::abs(angleError) > targetAngleDeviationNoStiffness)
  {
    positionOkay = false;

    // increase / decrease targetAngleOffset if robots starts to fall immediately after turning off stiffness
    if (theFrameInfo.getTimeSince(stablePositionTimestamp) > (transitionStart + transitionTime - 1000)
        && theFrameInfo.getTimeSince(stablePositionTimestamp) < (transitionStart + transitionTime + 1500))
    {
      if (angleError > targetAngleDeviationNoStiffness)
        targetAngleOffset += targetAngleOffsetStep;
      else
        targetAngleOffset -= targetAngleOffsetStep;


      targetAngleOffset = std::min(targetAngleOffset, targetAngleOffsetMax);
      targetAngleOffset = std::min(targetAngleOffset, targetAngleOffsetMax);
      targetAngleOffset = std::max(targetAngleOffset, -targetAngleOffsetMax);

      OUTPUT_TEXT("Using targetAngleOffset " << targetAngleOffset << " now!");
    }
  }

  // do not disable stiffness if body angle is too unstable
  if ((stablePositionTimestamp == 0 || theFrameInfo.getTimeSince(stablePositionTimestamp) <= transitionStart) && std::abs(angleError) > targetAngleDeviation)
  {
    positionOkay = false;
  }

  if (!positionOkay && stablePositionTimestamp > 0)
    stablePositionTimestamp = 0;

  if (positionOkay)
  {
    if (stablePositionTimestamp == 0)
      stablePositionTimestamp = theFrameInfo.time;

    // wait until transitionStart
    if (theFrameInfo.getTimeSince(stablePositionTimestamp) > transitionStart)
    {
      // reduce stiffness
      jointRequest.stiffnessTransition = static_cast<float>(theFrameInfo.getTimeSince(stablePositionTimestamp) - transitionStart) / transitionTime;
      float expTransition = 1.f - std::exp(-4.f * jointRequest.stiffnessTransition);
      expTransition = std::min(expTransition, 1.f);
      expTransition = std::max(expTransition, 0.f);
      bool stiffnessOff = true;
      for (int i = Joints::firstArmJoint; i < Joints::numOfJoints; ++i)
      {
        float stiffness = static_cast<float>(jointRequest.stiffnessData.stiffnesses[i]);
        stiffness *= (1.f - expTransition);
        stiffness += expTransition * lowStiffnesses[i];
        jointRequest.stiffnessData.stiffnesses[i] = static_cast<int>(std::round(stiffness));

        if (jointRequest.stiffnessData.stiffnesses[i] > 0)
          stiffnessOff = false;
      }

      if (stiffnessOff)
      {
        angleSum = 0_deg;
        angleOldError = 0_deg;
      }
    }
  }

  // reduce pitchOffset back to zero and set isLeavingPossible
  if (jointRequest.stiffnessTransition == 0.f && theMotionRequest.motion != MotionRequest::Motion::stand)
  {
    Angle step = leaveTransitionSpeed * theFrameInfo.cycleTime;
    if (pitchOffset > step)
      pitchOffset -= step;
    else if (pitchOffset < -step)
      pitchOffset += step;
    else
      jointRequest.isLeavingPossible = true;
  }
}
