#pragma once

#include "Kick.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>

class KickHackLong : public Kick
{
public:
  explicit KickHackLong() : Kick("KickHackLong", 1000.f, 1300.f, false, 1.21f, 350.f, false, 0_deg, 180.f, 55.f) {}

  void perform(Ballchaser& ballchaser, const Pose2f& kickPose, const Vector2f& targetPosition, const bool start) const override
  {
    ThresholdUtils::setThresholdsForAnyKick(ballchaser);
    ballchaser.optPosition = kickPose;
    ballchaser.kickType = MotionRequest::KickType::walkKick;
    ballchaser.kickTarget = targetPosition;
    ballchaser.walkKickType = start ? WalkRequest::StepRequest::any : WalkRequest::StepRequest::none;
  }

  [[nodiscard]] Pose2f getKickPose(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, bool leftFootClosestToBall) const override
  {
    return KickUtils::getKickPose(getOptAngleField(playerPose.translation, ballPosition, targetPosition), ballPosition, leftFootClosestToBall, optXDistanceToBall, optYDistanceToBall);
  }
};
