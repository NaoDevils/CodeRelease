#pragma once

#include "Kick.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>

class Rotate45Kick : public Kick
{

public:
  explicit Rotate45Kick() : Kick("Rotate45Kick", 1100.f, 1350.f, false, 1.3f, 400.f, false, 45_deg, 175.f, 20.f) {}

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

    const Pose2f ignoreCurrentRotation_playerPose = {(ballPosition - playerPose.translation).angle(), playerPose.translation};
    const Vector2f ballToTarget = targetPosition - ballPosition;
    const Vector2f robotView_ballToTarget = Transformation::fieldToRobot(ignoreCurrentRotation_playerPose, ballToTarget);
    const Angle robotView_ballToTargetAngle = robotView_ballToTarget.angle();
    bool kickWithRight = robotView_ballToTargetAngle > 0;

    return KickUtils::getKickPose(ballPosition, targetPosition, !playerOnLeftSideOfKickDirection, kickWithRight, optAngle, optXDistanceToBall, optYDistanceToBall);
  }
};
