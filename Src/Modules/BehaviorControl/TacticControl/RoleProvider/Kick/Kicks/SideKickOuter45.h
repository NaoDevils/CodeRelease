#pragma once

#include "Kick.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>

class SideKickOuter45 : public Kick
{

public:
  explicit SideKickOuter45() : Kick("SideKickOuter45", 900.f, 1250.f, false, 1.3f, 400.f, true, 45_deg, 120.f, 140.f) {}

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
    const bool playerOnLeftSideOfKickDirection = Geometry::isPointLeftOfLine(playerPose.translation, ballPosition, targetPosition);

    return KickUtils::getKickPose(ballPosition, targetPosition, !playerOnLeftSideOfKickDirection, playerOnLeftSideOfKickDirection, optAngle, optXDistanceToBall, optYDistanceToBall);
  }
};
