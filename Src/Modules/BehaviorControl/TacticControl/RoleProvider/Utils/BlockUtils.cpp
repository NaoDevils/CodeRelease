#include "BlockUtils.h"

#include <optional>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"

bool BlockUtils::isKickBlocked(
    const Vector2f& ballPosition, const Vector2f& targetPosition, const float minKickWidth, const bool hysteresis, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  const float hysteresisMult = hysteresis ? 0.9f : 1.f;

  const float ballRadius = theFieldDimensions.ballRadius * hysteresisMult;
  const float robotRadius = 140.f * hysteresisMult; // TODO Constants

  const float minKickRadius = minKickWidth / 2 * hysteresisMult;

  const Vector2f ballToTarget = (targetPosition - ballPosition);
  const Vector2f ballToTargetNormalized = ballToTarget.normalized();
  const float ballToTargetDistance = ballToTarget.norm();
  const Geometry::Line ballToTargetLine = {ballPosition, ballToTargetNormalized};
  const Geometry::Line behindBallLine = {ballPosition, Vector2f(ballToTargetNormalized).rotateLeft()};

  for (auto& robot : theRobotMap.robots)
  {
    const Vector2f position = robot.pose.translation;

    const bool behindBall = Geometry::isPointLeftOfLine(position, behindBallLine);
    if (behindBall)
    {
      continue;
    }

    float radius = robotRadius + minKickRadius + ballRadius;

    if (ballToTargetDistance < Geometry::distance(ballPosition, position) - radius)
    {
      // Robot ist too far away to block
      continue;
    }

    Geometry::Circle robotReach = {position, radius};

    Vector2f intersection1;
    Vector2f intersection2;
    const int intersections = Geometry::getIntersectionOfLineAndCircle(ballToTargetLine, robotReach, intersection1, intersection2);
    if (intersections > 0)
    {
      CIRCLE("behavior:BallchaserProvider:KickManager:Blocked", robot.pose.translation.x(), robot.pose.translation.y(), radius, 20, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::orange);
      return true;
    }
  }
  LINE("behavior:BallchaserProvider:KickManager:Blocked", ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5, Drawings::solidPen, ColorRGBA::green);
  return false;
}

bool BlockUtils::isTargetControlledByOpponent(const Vector2f& ballPosition, const Vector2f& targetPosition, const float minFreeAroundTarget, const bool hysteresis, const RobotMap& theRobotMap)
{
  const float hysteresisMult = hysteresis ? 1.f : 1.1f; // TODO Constant

  const float degree0_minFreeAroundTarget = minFreeAroundTarget * hysteresisMult;
  const float degree70_minFreeAroundTarget = minFreeAroundTarget * 0.7f * hysteresisMult; // TODO Constant
  const float degree120_minFreeAroundTarget = minFreeAroundTarget * 0.4f * hysteresisMult; // TODO Constant

  Geometry::Circle degree0_saveTargetArea = {targetPosition, degree0_minFreeAroundTarget};
  Geometry::Circle degree70_saveTargetArea = {targetPosition, degree70_minFreeAroundTarget};
  Geometry::Circle degree120_saveTargetArea = {targetPosition, degree120_minFreeAroundTarget};

  const Vector2f ballToTarget = (targetPosition - ballPosition).normalized();
  const Geometry::Line behindBallLine = {ballPosition, Vector2f(ballToTarget).rotateLeft()};

  for (auto& robot : theRobotMap.robots)
  {
    if (robot.robotType == RobotEstimate::teammateRobot)
    {
      continue;
    }

    const Vector2f position = robot.pose.translation;

    const bool behindBall = Geometry::isPointLeftOfLine(position, behindBallLine);
    if (behindBall)
    {
      continue;
    }

    if (Geometry::isPointInCircle(degree0_saveTargetArea, position))
    {
      LINE("behavior:BallchaserProvider:KickManager:TargetFree", ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5, Drawings::solidPen, ColorRGBA::red);
      CIRCLE("behavior:BallchaserProvider:KickManager:TargetFree", position.x(), position.y(), degree0_minFreeAroundTarget, 20, Drawings::dashedPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
      return true;
    }

    /* TODO Use of RobotMap includes Pose of enemies
    const Angle robotToTargetAngle = fabs(PositionUtils::getRelativeTurnToTargetAngle(robot.pose, targetPosition));

    if (robotToTargetAngle > 120_deg)
    {
      if (Geometry::isPointInCircle(degree120_saveTargetArea, position))
      {
        LINE("behavior:BallchaserProvider:KickManager:TargetFree",
             ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5,
             Drawings::solidPen, ColorRGBA::yellow);
        CIRCLE("behavior:BallchaserProvider:KickManager:TargetFree",
               position.x(), position.y(), degree120_minFreeAroundTarget, 20,
               Drawings::dashedPen, ColorRGBA::yellow,
               Drawings::noBrush, ColorRGBA::yellow);
        return true;
      }
    }
    else if (robotToTargetAngle > 70_deg)
    {
      if (Geometry::isPointInCircle(degree70_saveTargetArea, position))
      {
        LINE("behavior:BallchaserProvider:KickManager:TargetFree",
             ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5,
             Drawings::solidPen, ColorRGBA::orange);
        CIRCLE("behavior:BallchaserProvider:KickManager:TargetFree",
               position.x(), position.y(), degree70_minFreeAroundTarget, 20,
               Drawings::dashedPen, ColorRGBA::orange,
               Drawings::noBrush, ColorRGBA::orange);
        return true;
      }
    }
    else
    {
      if (Geometry::isPointInCircle(degree0_saveTargetArea, position))
      {
        LINE("behavior:BallchaserProvider:KickManager:TargetFree",
             ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5,
             Drawings::solidPen, ColorRGBA::red);
        CIRCLE("behavior:BallchaserProvider:KickManager:TargetFree",
               position.x(), position.y(), degree0_minFreeAroundTarget, 20,
               Drawings::dashedPen, ColorRGBA::red,
               Drawings::noBrush, ColorRGBA::red);
        return true;
      }
    }
    */
  }
  LINE("behavior:BallchaserProvider:KickManager:TargetFree", ballPosition.x(), ballPosition.y(), targetPosition.x(), targetPosition.y(), 5, Drawings::solidPen, ColorRGBA::green);
  return false;
}

/**
* @brief brief Calculates for all opponent robots that are kind of close to opponent goal, if they block a goal kick.
*/
bool BlockUtils::isKickBlockedOld(
    const Vector2f& ballPosition, const Vector2f& targetPosition, const float minOpponentToBallDistance, const float minOpeningAngleForDirectKick, const float robotRadius, const RobotMap& theRobotMap)
{
  // Ball position on field with angle to opponent goal.
  Pose2f ballPosistionWCAngleToOppGoalCenter((targetPosition - ballPosition).angle(), ballPosition);

  // For each robot (only opponents) of the robot map:
  for (auto& robot : theRobotMap.robots)
  {
    if (robot.robotType != RobotEstimate::teammateRobot)
    {
      // The position of the opponent robot from the robot map.
      Vector2f opponentRobot = robot.pose.translation;

      Vector2f ballPositionRelativeToRobot = Transformation::fieldToRobot(ballPosistionWCAngleToOppGoalCenter, opponentRobot);
      float distOpponentToBall = ballPositionRelativeToRobot.norm(); // Distance between opponent robot and ball.
      Angle angleOpponentToBall = ballPositionRelativeToRobot.angle(); // Angle opponent robot to ball.

      // Check close opponent robots that are near the line from ball to kick targetPosition
      if (distOpponentToBall < minOpponentToBallDistance // Distance smaller than max distance to block goal, with hysteresis.
          //&& (opponentRobot - targetPosition).norm()
          //    > theFieldDimensions.yPosLeftGoal * 2 - (goalKickBlocked ? 250.f : 0.f) // Opponent is close to opponent goal, with hysteresis.
          && std::abs(angleOpponentToBall) < 90_deg) // Angle from opponent to ball is not more than 90 degrees.
      {
        float g = std::abs(distOpponentToBall * std::sin(angleOpponentToBall)); // Opposite side of triangle ball - vecToGoal - robot.
        float a = std::abs(distOpponentToBall * std::cos(angleOpponentToBall)); // Adjacent side of triangle ball - vecToGoal - robot.

        // The allowed angle of the corridor increases with the distance to the robot.
        // float maxCorridorAngle = minOpeningAngleForDirectKick + (distOpponentToBall / minOpponentToBallDistance) * minOpeningAngleForDirectKick;

        // Calculate the opposite side of the triangle ball - vecToGoal - corridor edge.
        float minDistToBallToGoalVector = a * std::tan(minOpeningAngleForDirectKick);
        if (g - robotRadius < minDistToBallToGoalVector)
        {
          return true;
        }
      }
    }
  }
  return false;
}