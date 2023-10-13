#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

class WalkingEngineKick : public Kick
{
  WalkRequest::StepRequest walkKickType;

public:
  explicit WalkingEngineKick(const CustomStepsFile& customStepsFile)
      : Kick(customStepsFile.name,
          customStepsFile.kickDistance[0] * 1000.f,
          customStepsFile.kickDistance[1] * 1000.f,
          customStepsFile.distanceAdjustable,
          calculateKickTime(customStepsFile),
          customStepsFile.horizontalInaccuracy * 1000.f,
          customStepsFile.kickBlind,
          customStepsFile.kickWithLeftCondition,
          customStepsFile.switchKickFoot,
          customStepsFile.kickAngle,
          customStepsFile.ballOffset.x() * 1000.f,
          customStepsFile.ballOffset.y() * 1000.f)
  {
    walkKickType = customStepsFile.stepRequest;
  }

  static float calculateKickTime(const CustomStepsFile& customStepsFile)
  {
    int time = 0;
    for (const auto& step : customStepsFile.steps)
    {
      if (step.kick)
      {
        break;
      }
      time += step.duration;
    }
    return (float)time / 10.f;
  }

  void perform(PositioningAndKickSymbols& pakSymbols, const Pose2f& kickPose, const bool kickPoseMirrored, const Vector2f& targetPosition, const bool start) const override
  {
    ThresholdUtils::setThresholdsForAnyKick(pakSymbols);
    pakSymbols.optPosition = kickPose;
    pakSymbols.kickTarget = targetPosition;
    pakSymbols.kickType = MotionRequest::KickType::walkKick;
    pakSymbols.walkKickType = walkKickType;
    pakSymbols.mirrorKick = kickPoseMirrored;
    pakSymbols.kickBlind = kickBlind;
  }
};
