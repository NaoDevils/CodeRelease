#include "WalkParamsProvider.h"
#include "Tools/Debugging/Annotation.h"
#include <Platform/Common/Text2Speech.h>
#include "Tools/Build.h"

void WalkParamsProvider::update(WalkingEngineParams& walkingEngineParams)
{
  if (!initializedWP)
  {
    load(walkingEngineParams);
    load(originalParams, true);

    if (speakOutChanges)
    {
      OUTPUT_TEXT("[SpeedLimits] xBackward: " << walkingEngineParams.speedLimits.xBackward);
      OUTPUT_TEXT("[SpeedLimits] xForward: " << walkingEngineParams.speedLimits.xForward);
      OUTPUT_TEXT("[SpeedLimits] xForwardArmContact: " << walkingEngineParams.speedLimits.xForwardArmContact);
      OUTPUT_TEXT("[SpeedLimits] xForwardOmni: " << walkingEngineParams.speedLimits.xForwardOmni);
      OUTPUT_TEXT("[SpeedLimits] y: " << walkingEngineParams.speedLimits.y);
      OUTPUT_TEXT("[SpeedLimits] yArmContact: " << walkingEngineParams.speedLimits.yArmContact);
    }
  }

  if (theBehaviorData.behaviorState == BehaviorData::calibrationStarted)
    walkingEngineParams.speedLimits = originalParams.speedLimits;

  if (!lastWalkCalibrated && theWalkCalibration.walkCalibrated)
    setMinMax(walkingEngineParams);

  for (int i = 0; i < 12; i++)
    walkingEngineParams.jointCalibration.jointCalibration[i] = originalParams.jointCalibration.jointCalibration[i];

  lastXForwardUpdated = false;
  lastXBackwardUpdated = false;
  lastYUpdated = false;

  if (theMotionState.walkingStatus.speedFactorForward != 1.f)
  {
    walkingEngineParams.speedLimits.xForward = std::clamp(
        walkingEngineParams.speedLimits.xForward * theMotionState.walkingStatus.speedFactorForward, walkingEngineParams.minSpeedLimits.xForward, walkingEngineParams.maxSpeedLimits.xForward);
    walkingEngineParams.speedLimits.xForwardOmni = std::clamp(walkingEngineParams.speedLimits.xForwardOmni * theMotionState.walkingStatus.speedFactorForward,
        walkingEngineParams.minSpeedLimits.xForwardOmni,
        walkingEngineParams.maxSpeedLimits.xForwardOmni);
    walkingEngineParams.speedLimits.xForwardArmContact = std::clamp(walkingEngineParams.speedLimits.xForwardArmContact * theMotionState.walkingStatus.speedFactorForward,
        walkingEngineParams.minSpeedLimits.xForwardArmContact,
        walkingEngineParams.maxSpeedLimits.xForwardArmContact);

    if (!lastXForwardUpdated)
    {
      ANNOTATION("MotionState", "xForward updated to " << walkingEngineParams.speedLimits.xForward);
      if (speakOutChanges)
      {
        OUTPUT_TEXT("[MotionState] xForward updated to " << walkingEngineParams.speedLimits.xForward);
        char buffer[200];
        sprintf(buffer, "Forward %.0f", walkingEngineParams.speedLimits.xForward);
        SystemCall::text2Speech(buffer);
      }
    }
    lastXForwardUpdated = true;
  }

  if (theMotionState.walkingStatus.speedFactorBackward != 1.f)
  {
    walkingEngineParams.speedLimits.xBackward = std::clamp(walkingEngineParams.speedLimits.xBackward * theMotionState.walkingStatus.speedFactorBackward,
        walkingEngineParams.minSpeedLimits.xBackward,
        walkingEngineParams.maxSpeedLimits.xBackward);
    if (!lastXBackwardUpdated)
    {
      ANNOTATION("MotionState", "xBackward updated to " << walkingEngineParams.speedLimits.xBackward);
      if (speakOutChanges)
      {
        OUTPUT_TEXT("[MotionState] xBackward updated to " << walkingEngineParams.speedLimits.xBackward);
        char buffer[200];
        sprintf(buffer, "Beckword %.0f", walkingEngineParams.speedLimits.xBackward);
        SystemCall::text2Speech(buffer);
      }
    }
    lastXBackwardUpdated = true;
  }

  if (theMotionState.walkingStatus.speedFactorLeft != 1.f || theMotionState.walkingStatus.speedFactorRight != 1.f)
  {
    float maxSpeedFactor = std::max<float>(theMotionState.walkingStatus.speedFactorLeft, theMotionState.walkingStatus.speedFactorRight);
    walkingEngineParams.speedLimits.y = std::clamp(walkingEngineParams.speedLimits.y * maxSpeedFactor, walkingEngineParams.minSpeedLimits.y, walkingEngineParams.maxSpeedLimits.y);
    walkingEngineParams.speedLimits.yArmContact =
        std::clamp(walkingEngineParams.speedLimits.yArmContact * maxSpeedFactor, walkingEngineParams.minSpeedLimits.yArmContact, walkingEngineParams.maxSpeedLimits.yArmContact);
    if (!lastYUpdated)
    {
      ANNOTATION("MotionState", "y updated to " << walkingEngineParams.speedLimits.y);
      if (speakOutChanges)
      {
        OUTPUT_TEXT("[MotionState] y updated to " << walkingEngineParams.speedLimits.y);
        char buffer[200];
        sprintf(buffer, "Seydwards %.0f", walkingEngineParams.speedLimits.y);
        SystemCall::text2Speech(buffer);
      }
    }
    lastYUpdated = true;
  }

  lastWalkCalibrated = theWalkCalibration.walkCalibrated;

  MODIFY("annotation:motion:text", annotation);
  DEBUG_RESPONSE_ONCE("annotation:motion")
  {
    ANNOTATION("Console", annotation);
  }
}

void WalkParamsProvider::update(SensorControlParams& sensorControlParams)
{
  if (!initializedSCP)
  {
    load(sensorControlParams);
  }
}

void WalkParamsProvider::load(WalkingEngineParams& walkingEngineParams, bool original)
{
  InMapFile file("walkingParamsFLIPM.cfg");
  if (file.exists())
    file >> walkingEngineParams;
  else
  {
    ASSERT(false);
  }

  if constexpr (Build::targetRobot())
  {
    InMapFile motionMindfulnessSpeedLimitsFile(File::getPersistentDir() + "speedLimits.cfg");
    if (motionMindfulnessSpeedLimitsFile.exists() && !original)
    {
      motionMindfulnessSpeedLimitsFile >> walkingEngineParams.speedLimits;
      ANNOTATION("MotionState", "Loaded " << File::getPersistentDir() << "speedLimits.cfg");
    }
  }
  setMinMax(walkingEngineParams);
  initializedWP = true;
}

void WalkParamsProvider::setMinMax(WalkingEngineParams& walkingEngineParams)
{
  // clang-format off
  walkingEngineParams.minSpeedLimits.r = walkingEngineParams.speedLimits.r;
  walkingEngineParams.maxSpeedLimits.r = walkingEngineParams.speedLimits.r;

  walkingEngineParams.minSpeedLimits.rOnly = walkingEngineParams.speedLimits.rOnly;
  walkingEngineParams.maxSpeedLimits.rOnly = walkingEngineParams.speedLimits.rOnly;

  walkingEngineParams.minSpeedLimits.xBackward = minXBackward;
  walkingEngineParams.maxSpeedLimits.xBackward = minXBackward + (maxXBackward - minXBackward) * std::abs(theWalkCalibration.qualityOfRobotHardware);
  walkingEngineParams.speedLimits.xBackward = std::clamp(walkingEngineParams.speedLimits.xBackward,  walkingEngineParams.minSpeedLimits.xBackward, walkingEngineParams.maxSpeedLimits.xBackward);

  walkingEngineParams.minSpeedLimits.xForward = minXForward;
  walkingEngineParams.maxSpeedLimits.xForward = minXForward + (maxXForward - minXForward) * std::abs(theWalkCalibration.qualityOfRobotHardware);
  walkingEngineParams.speedLimits.xForward = std::clamp(walkingEngineParams.speedLimits.xForward, walkingEngineParams.minSpeedLimits.xForward, walkingEngineParams.maxSpeedLimits.xForward);

  walkingEngineParams.minSpeedLimits.xForwardArmContact = minXForwardArmContact;
  walkingEngineParams.maxSpeedLimits.xForwardArmContact = minXForwardArmContact + (maxXForwardArmContact - minXForwardArmContact) * std::abs(theWalkCalibration.qualityOfRobotHardware);
  walkingEngineParams.speedLimits.xForwardArmContact = std::clamp(walkingEngineParams.speedLimits.xForwardArmContact, walkingEngineParams.minSpeedLimits.xForwardArmContact, walkingEngineParams.maxSpeedLimits.xForwardArmContact);

  walkingEngineParams.minSpeedLimits.xForwardOmni = minXForwardOmni;
  walkingEngineParams.maxSpeedLimits.xForwardOmni = minXForwardOmni + (maxXForwardOmni - minXForwardOmni) * std::abs(theWalkCalibration.qualityOfRobotHardware);
  walkingEngineParams.speedLimits.xForwardOmni = std::clamp(walkingEngineParams.speedLimits.xForwardOmni,walkingEngineParams.minSpeedLimits.xForwardOmni, walkingEngineParams.maxSpeedLimits.xForwardOmni);

  walkingEngineParams.minSpeedLimits.y = minY;
  walkingEngineParams.maxSpeedLimits.y = minY + (maxY - minY) * std::abs(theWalkCalibration.qualityOfRobotHardware) ;
  walkingEngineParams.speedLimits.y = std::clamp(walkingEngineParams.speedLimits.y, walkingEngineParams.minSpeedLimits.y, walkingEngineParams.maxSpeedLimits.y);

  walkingEngineParams.minSpeedLimits.yArmContact = minYArmContact;
  walkingEngineParams.maxSpeedLimits.yArmContact = minYArmContact + (maxYArmContact - minYArmContact) * std::abs(theWalkCalibration.qualityOfRobotHardware);
  walkingEngineParams.speedLimits.yArmContact = std::clamp(walkingEngineParams.speedLimits.yArmContact, walkingEngineParams.minSpeedLimits.yArmContact, walkingEngineParams.maxSpeedLimits.yArmContact);
  // clang-format on
}

void WalkParamsProvider::load(SensorControlParams& sensorControlParams)
{
  InMapFile file("sensorControlParams.cfg");
  if (file.exists())
    file >> sensorControlParams;
  else
  {
    ASSERT(false);
  }
  initializedSCP = true;
}

MAKE_MODULE(WalkParamsProvider, dortmundWalkingEngine)
