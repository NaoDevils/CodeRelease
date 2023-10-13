#include "WalkParamsProvider.h"
#include "Tools/Debugging/Annotation.h"
#include <Platform/Common/Text2Speech.h>
#include "Tools/Build.h"

void WalkParamsProvider::update(WalkingEngineParams& walkingEngineParams)
{
  if (!initializedWP)
  {
    load(walkingEngineParams);
    load(originalParams);

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

  for (int i = 0; i < 12; i++)
    walkingEngineParams.jointCalibration.jointCalibration[i] = originalParams.jointCalibration.jointCalibration[i];

  if (theMotionState.walkingStatus.speedFactorForward != 1.f)
  {
    walkingEngineParams.speedLimits.xForward = std::max<float>(minXForward, std::min<float>(maxXForward, walkingEngineParams.speedLimits.xForward * theMotionState.walkingStatus.speedFactorForward));
    walkingEngineParams.speedLimits.xForwardOmni =
        std::max<float>(minXForwardOmni, std::min<float>(maxXForwardOmni, walkingEngineParams.speedLimits.xForwardOmni * theMotionState.walkingStatus.speedFactorForward));
    walkingEngineParams.speedLimits.xForwardArmContact = std::max<float>(
        minXForwardArmContact, std::min<float>(maxXForwardArmContact, walkingEngineParams.speedLimits.xForwardArmContact * theMotionState.walkingStatus.speedFactorForward));

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
  else
  {
    lastXForwardUpdated = false;
  }

  if (theMotionState.walkingStatus.speedFactorBackward != 1.f)
  {
    walkingEngineParams.speedLimits.xBackward =
        std::max<float>(minXBackward, std::min<float>(maxXBackward, walkingEngineParams.speedLimits.xBackward * theMotionState.walkingStatus.speedFactorBackward));
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
  else
  {
    lastXBackwardUpdated = false;
  }

  if (theMotionState.walkingStatus.speedFactorLeft != 1.f || theMotionState.walkingStatus.speedFactorRight != 1.f)
  {
    walkingEngineParams.speedLimits.y = std::max<float>(minY,
        std::min<float>(maxY, walkingEngineParams.speedLimits.y * std::max<float>(theMotionState.walkingStatus.speedFactorLeft, theMotionState.walkingStatus.speedFactorRight)));
    walkingEngineParams.speedLimits.yArmContact = std::max<float>(minY,
        std::min<float>(maxY, walkingEngineParams.speedLimits.yArmContact * std::max<float>(theMotionState.walkingStatus.speedFactorLeft, theMotionState.walkingStatus.speedFactorRight)));
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
  else
  {
    lastYUpdated = false;
  }

  MODIFY("annotation:motion:text", annotation);
  DEBUG_RESPONSE_ONCE("annotation:motion")
  {
    ANNOTATION("Console", annotation);
  }
}

void WalkParamsProvider::update(FLIPMObserverGains& flipmObserverGains)
{
  if (!initializedFOP)
  {
    load(flipmObserverGains);
  }
}

void WalkParamsProvider::load(WalkingEngineParams& walkingEngineParams)
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
    InTextFile motionMindfulnessSpeedLimitsFile(File::getPersistentDir() + "speedLimits.value");
    if (motionMindfulnessSpeedLimitsFile.exists())
    {
      motionMindfulnessSpeedLimitsFile >> walkingEngineParams.speedLimits;
      ANNOTATION("MotionState", "Loaded " << File::getPersistentDir() << "speedLimits.value");
    }
  }

  walkingEngineParams.speedLimits.xBackward = std::max<float>(minXBackward, std::min<float>(maxXBackward, walkingEngineParams.speedLimits.xBackward));
  walkingEngineParams.speedLimits.xForward = std::max<float>(minXForward, std::min<float>(maxXForward, walkingEngineParams.speedLimits.xForward));
  walkingEngineParams.speedLimits.xForwardArmContact = std::max<float>(minXForwardArmContact, std::min<float>(maxXForwardArmContact, walkingEngineParams.speedLimits.xForwardArmContact));
  walkingEngineParams.speedLimits.xForwardOmni = std::max<float>(minXForwardOmni, std::min<float>(maxXForwardOmni, walkingEngineParams.speedLimits.xForwardOmni));
  walkingEngineParams.speedLimits.y = std::max<float>(minY, std::min<float>(maxY, walkingEngineParams.speedLimits.y));
  walkingEngineParams.speedLimits.yArmContact = std::max<float>(minYArmContact, std::min<float>(maxYArmContact, walkingEngineParams.speedLimits.yArmContact));

  initializedWP = true;
}

void WalkParamsProvider::load(FLIPMObserverGains& flipmObserverGains)
{
  InMapFile file("flipmObserverGains.cfg");
  if (file.exists())
    file >> flipmObserverGains;
  else
  {
    ASSERT(false);
  }
  initializedFOP = true;
}

MAKE_MODULE(WalkParamsProvider, dortmundWalkingEngine)
