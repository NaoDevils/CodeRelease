#pragma once

#include "Kick.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>

class FastLongKick : public Kick
{

public:
  explicit FastLongKick() : Kick("FastLongKick", 2500.f, 3400.f, false, 3.f, 1800.f, false, 0_deg, 190.f, 55.f) {}

  void perform(Ballchaser& ballchaser, const Pose2f& kickPose, const Vector2f& targetPosition, const bool start) const override
  {
    ThresholdUtils::setThresholdsForLongKick(ballchaser);
    ballchaser.optPosition = kickPose;
    ballchaser.kickTarget = targetPosition;
    ballchaser.kickType = MotionRequest::KickType::longKick;
    ballchaser.longKickType = KickRequest::KickMotionID::kickMiddleFast;
  };

  [[nodiscard]] Pose2f getKickPose(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, bool leftFootClosestToBall) const override
  {
    return KickUtils::getKickPose(getOptAngleField(playerPose.translation, ballPosition, targetPosition), ballPosition, leftFootClosestToBall, optXDistanceToBall, optYDistanceToBall);
  }
};
