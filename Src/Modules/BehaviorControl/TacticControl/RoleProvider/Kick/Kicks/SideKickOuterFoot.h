#pragma once

#include "Kick.h"
#include "Tools/Math/Transformation.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>

class SideKickOuterFoot : public Kick
{

public:
  explicit SideKickOuterFoot() : Kick("SideKickOuterFoot", 1100.f, 1300.f, false, 1.5f, 450.f, true, 90_deg, 0.f, 180.f) {}

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
