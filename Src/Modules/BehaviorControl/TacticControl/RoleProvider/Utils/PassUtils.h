#pragma once

#include <Representations/BehaviorControl/PositioningSymbols.h>
#include "Representations/Modeling/RobotMap.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "FieldUtils.h"

class PassUtils
{

public:
  /**
   * @param passPosition Position of the player who shall receive the pass
  */
  static std::tuple<Pose2f, Vector2f, Angle> getPassInfo(
      const Vector2f& ballPosition, const Vector2f& passTarget, const bool kickWithLeft, const BehaviorConfiguration& theBehaviorConfiguration, const FieldDimensions& theFieldDimensions, const RobotPoseAfterPreview& theRobotPoseAfterPreview)
  {
    // TODO: Check free space / kick corridor / team mate target position
    // TODO: check stability

    const Vector2f goalCenter = FieldUtils::getOpponentGoalCenter(theFieldDimensions);
    const Vector2f passTargetToGoal{goalCenter - passTarget};
    const Vector2f kickTarget{passTarget + passTargetToGoal.normalized(400.f)}; //TODO Constant

    const Vector2f minPassTarget{kickTarget - passTargetToGoal.normalized(200.f)}; //TODO Constant
    const Vector2f maxPassTarget{kickTarget + passTargetToGoal.normalized(200.f)}; //TODO Constant
    const Angle minPassTargetToBallAngle = (minPassTarget - ballPosition).angle();
    const Angle maxPassTargetToBallAngle = (maxPassTarget - ballPosition).angle();
    const Rangef kickAngleLimit(std::min(minPassTargetToBallAngle, maxPassTargetToBallAngle), std::max(minPassTargetToBallAngle, maxPassTargetToBallAngle));

    const Pose2f kickFootPose = {}; //todo theRobotPoseAfterPreview * Vector2f{ 0.f, (kickWithLeft ? +1.f : -1.f) * theBehaviorConfiguration.optDistanceToBallY };
    Angle kickTargetAngle = (ballPosition - kickFootPose.translation).angle();
    kickTargetAngle = kickAngleLimit.limit(kickTargetAngle); // TODO Isnt the result always the kickTargetAngle?

    return {kickFootPose, kickTarget, kickTargetAngle};
  }

  static std::tuple<Pose2f, Vector2f> getPassInfo(const Vector2f& ballPosition,
      const Vector2f& passTarget,
      const bool kickWithLeft,
      const float optXDistanceToBall,
      const float optYDistanceToBall,
      const BehaviorConfiguration& theBehaviorConfiguration,
      const FieldDimensions& theFieldDimensions,
      const RobotPoseAfterPreview& theRobotPoseAfterPreview)
  {
    auto [kickFootPose, kickTarget, kickTargetAngle] = getPassInfo(ballPosition, passTarget, kickWithLeft, theBehaviorConfiguration, theFieldDimensions, theRobotPoseAfterPreview);
    Pose2f kickPose = getKickPose(ballPosition, kickTargetAngle, kickWithLeft, optXDistanceToBall, optYDistanceToBall);
    return {kickPose, kickTarget};
  }

  static Pose2f getKickPose(const Vector2f& ballPosition, const Angle kickAngle, const bool kickWithLeft, const float optXDistanceToBall, const float optYDistanceToBall)
  {
    Pose2f kickPose = Pose2f(kickAngle, ballPosition);
    kickPose.translate(-optXDistanceToBall, (kickWithLeft ? -1.f : +1.f) * optYDistanceToBall);
    return kickPose;
  }

  /**
   * @return a pass targetPosition if possible, or leaves the vector as is otherwise
  */
  static bool calcPassTarget(Vector2f& targetPosition,
      const float& maxKickDistance,
      const float& kickoffPassXOffset,
      const GameSymbols& theGameSymbols,
      const RobotMap& theRobotMap,
      const RobotPoseAfterPreview& theRobotPoseAfterPreview,
      const TeammateData& theTeammateData)
  {
    // search for good positioned team mate
    bool targetFound = false;
    for (auto& mate : theTeammateData.teammates)
    {
      const bool ownKickOff = theGameSymbols.kickoffInProgress && theGameSymbols.ownKickOff;
      const float safeXPosition = std::min(0.f, theRobotPoseAfterPreview.translation.x()) - (ownKickOff ? 500.f : 0.f);
      const bool matePositionSafe = mate.pose.translation.x() > safeXPosition;
      const bool isKeeper = mate.behaviorData.role == BehaviorData::keeper || mate.behaviorData.role == BehaviorData::replacementKeeper;
      const bool positionReachable = (mate.pose.translation - theRobotPoseAfterPreview.translation).norm() < maxKickDistance;

      if (!isKeeper && mate.status == Teammate::FULLY_ACTIVE && mate.pose.validity > 0.6 && matePositionSafe && positionReachable)
      {
        Vector2f toTarget(mate.pose.translation - theRobotPoseAfterPreview.translation);
        float angleToTarget = toTarget.angle();
        float distanceToTarget = toTarget.norm();
        bool foundObstacle = false;
        for (auto& robot : theRobotMap.robots)
        {
          Vector2f toRobot(robot.pose.translation - theRobotPoseAfterPreview.translation);
          float angleToRobot = toRobot.angle();
          float distanceToRobot = toRobot.norm();
          if ((distanceToTarget < distanceToRobot - 500) && std::abs(Angle::normalize(angleToTarget - angleToRobot)) < 30_deg)
          {
            foundObstacle = true;
            break;
          }
        }
        if (!foundObstacle && (!targetFound || targetPosition.x() < mate.pose.translation.x()))
        {
          targetPosition = mate.pose.translation;
          targetFound = true;
        }
      }
    }
    // no good pass targetPosition, do not touch original targetPosition!

    if (targetFound && theGameSymbols.kickoffInProgress)
      targetPosition.x() += kickoffPassXOffset;

    return targetFound;
  }
};
