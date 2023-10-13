#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"

class KickEngineKick : public Kick
{
  KickRequest::KickMotionID longKickType;

public:
  explicit KickEngineKick(const KickEngineParameters& kickEngineParameters)
      : Kick(kickEngineParameters.name,
          kickEngineParameters.kickDistance[0] * 1000.f,
          kickEngineParameters.kickDistance[1] * 1000.f,
          kickEngineParameters.distanceAdjustable,
          calculateKickTime(kickEngineParameters),
          kickEngineParameters.horizontalInaccuracy * 1000.f,
          kickEngineParameters.kickBlind,
          kickEngineParameters.kickWithLeftCondition,
          kickEngineParameters.switchKickFoot,
          kickEngineParameters.kickAngle,
          kickEngineParameters.ballOffset.x() * 1000.f,
          kickEngineParameters.ballOffset.y() * 1000.f)
  {
    longKickType = kickEngineParameters.type;
  }

  static float calculateKickTime(const KickEngineParameters& kickEngineParameters)
  {
    unsigned int time = 0;
    for (const auto& phase : kickEngineParameters.phaseParameters)
    {
      if (phase.kick)
      {
        break;
      }
      time += phase.duration;
    }
    return (float)time / 1000.f;
  }

  void perform(PositioningAndKickSymbols& pakSymbols, const Pose2f& kickPose, const bool kickPoseMirrored, const Vector2f& targetPosition, const bool start) const override
  {
    ThresholdUtils::setThresholdsForLongKick(pakSymbols);
    pakSymbols.optPosition = kickPose;
    pakSymbols.kickTarget = targetPosition;
    pakSymbols.kickType = MotionRequest::KickType::longKick;
    pakSymbols.longKickType = longKickType;
    pakSymbols.mirrorKick = kickPoseMirrored;
    pakSymbols.kickBlind = kickBlind;
  }
};
