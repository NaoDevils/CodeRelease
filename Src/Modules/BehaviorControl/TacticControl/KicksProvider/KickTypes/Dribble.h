#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"

class Dribble : public Kick
{

public:
  explicit Dribble() : Kick("Dribble", 1.f, 5.f, 600.f, 600.f, 500.f, 15_deg, false, false, 0_deg, 200.f, 0.f) {}

  void perform(PositioningAndKickSymbols& pakSymbols, const Pose2f& kickPose, const bool kickPoseMirrored, const Vector2f& targetPosition) const override
  {
    ThresholdUtils::setThresholdsForDribble(pakSymbols);
    pakSymbols.optPosition = kickPose;
    pakSymbols.kickTarget = targetPosition;
    pakSymbols.kickType = MotionRequest::KickType::dribble;
    pakSymbols.longKickType = KickRequest::KickMotionID::kickMiddleFast;
    pakSymbols.mirrorKick = kickPoseMirrored;
    pakSymbols.kickBlind = kickBlind;
  };
};
