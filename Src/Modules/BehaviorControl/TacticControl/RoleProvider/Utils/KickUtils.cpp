#include "KickUtils.h"

#include <optional>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/Optimize.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "DangerUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BlockUtils.h>
#include "Representations/Configuration/FieldDimensions.h"
#include "MathUtils.h"

Pose2f KickUtils::getKickPose(
    const Vector2f& ballPosition, const Vector2f& targetPosition, const bool rotateLeft, const bool ballLeftFoot, const Angle optAngle, const float afterRotation_optDistanceToBallX, const float afterRotation_optDistanceToBallY)
{
  const Angle ballToTargetAngle = (targetPosition - ballPosition).angle();
  const Angle field_angleUnnormalized = ballToTargetAngle + (rotateLeft ? 1.f : -1.f) * optAngle;
  const Angle field_angle = Angle::normalize(field_angleUnnormalized);
  return getKickPose(field_angle, ballPosition, ballLeftFoot, afterRotation_optDistanceToBallX, afterRotation_optDistanceToBallY);
}

Pose2f KickUtils::getKickPose(const Angle& robotRotation, const Vector2f& ballPosition, const bool ballLeftFoot, const float afterRotation_optDistanceToBallX, const float afterRotation_optDistanceToBallY)
{
  Pose2f pose(robotRotation, ballPosition);
  pose = pose.translate(-afterRotation_optDistanceToBallX, (ballLeftFoot ? -1.f : +1.f) * afterRotation_optDistanceToBallY);
  return pose;
}

Angle KickUtils::getFastestReachableKickAngleBetweenTargets(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2)
{
  const Vector2f ballToTarget1 = target1 - ballPosition;
  const Vector2f ballToTarget2 = target2 - ballPosition;

  const Angle ballToTarget1Angle = ballToTarget1.angle();
  const Angle ballToTarget2Angle = ballToTarget2.angle();

  const Angle minAngle = ballToTarget1Angle <= ballToTarget2Angle ? ballToTarget1Angle : ballToTarget2Angle;
  const Angle maxAngle = ballToTarget1Angle > ballToTarget2Angle ? ballToTarget1Angle : ballToTarget2Angle;

  const Rangef kickAngleLimit(minAngle, maxAngle);

  const Angle playerToBallAngle = (ballPosition - playerPosition).angle();

  return kickAngleLimit.limit(playerToBallAngle);
}

float KickUtils::getKickTime(const Kick& kick, const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, const bool hysteresis, const bool leftFootClosestToBall)
{
  const Angle TURN_ANGLE_PER_SECOND = 15_deg; // TODO Constant
  const float WALK_MM_PER_SECOND = 150.f; // TODO Constant
  const float MAX_DISTANCE = 300.f;

  const Pose2f kickPose = kick.getKickPose(playerPose, ballPosition, targetPosition, leftFootClosestToBall);

  float time = kick.getTime(hysteresis);

  const float mult = MathUtils::clamp_f((Geometry::distance(playerPose.translation, kickPose.translation) - 200) / 200.f, 0.f, 1.f);

  const Angle optAroundBallAngle = (ballPosition - kickPose.translation).angle();
  const Angle currentAroundBallAngle = (ballPosition - playerPose.translation).angle();
  const float aroundBallAngleDiff = MathUtils::getAngleDiff(optAroundBallAngle, currentAroundBallAngle, hysteresis);
  time += mult * (aroundBallAngleDiff / TURN_ANGLE_PER_SECOND);

  const Angle optToBallAngle = kickPose.rotation;
  const Angle currentToBallAngle = playerPose.rotation;
  const float toBallAngleDiff = MathUtils::getAngleDiff(optToBallAngle, currentToBallAngle, hysteresis);
  time += (1 - mult) * (toBallAngleDiff / TURN_ANGLE_PER_SECOND);

  const float distance = MathUtils::clamp_f(Geometry::distance(playerPose.translation, kickPose.translation), 0.f, MAX_DISTANCE);
  const float walkTime = distance / WALK_MM_PER_SECOND;
  time += walkTime;

  return time;
}

bool KickUtils::fulfillsDistanceRequirements(const Kick& kick, const float targetDistance, const DistanceRequirement distanceRequirement, const bool hysteresis)
{
  const int iTargetDistance = (int)targetDistance;

  const int minDistance = (int)kick.getMinDistance(hysteresis, false);
  const int maxDistance = (int)kick.getMaxDistance(hysteresis, true);

  switch (distanceRequirement)
  {
  case onPoint:
    return minDistance <= iTargetDistance && iTargetDistance <= maxDistance;

  case mayFurther:
    return iTargetDistance <= maxDistance;
  case shouldFurther:
    return iTargetDistance <= (int)kick.getRealisticDistance(targetDistance, hysteresis, true);
  case mustFurther:
    return iTargetDistance <= (int)kick.getMinDistance(hysteresis, true);

  case mayShorter:
    return minDistance <= iTargetDistance;
  case shouldShorter:
    return (int)kick.getRealisticDistance(targetDistance, hysteresis, false) <= iTargetDistance;
  case mustShorter:
    return (int)kick.getMaxDistance(hysteresis, false) <= iTargetDistance;

  case mayShorterOrFurther:
    return true;

  default:
    throw std::logic_error("");
  }
}

bool KickUtils::isKickToOutside(const Kick* kick, const Vector2f& ballPosition, const Vector2f& direction, const bool hysteresis, const FieldDimensions& theFieldDimensions)
{
  float kickDistance;
  if (kick->isDistanceAdjustable())
  {
    kickDistance = kick->getMinDistance(hysteresis, false); // TODO + Inaccuracy ?
  }
  else
  {
    kickDistance = kick->getMaxDistance(hysteresis, false);
  }
  const Vector2f worstCaseTargetPosition = ballPosition + direction * kickDistance;
  return !theFieldDimensions.isInsideField(worstCaseTargetPosition);
}

float KickUtils::getMinKickToObstaclesDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  return std::min(getMinKickToGoalPostDistance(ballPosition, targetPosition, theFieldDimensions),
      std::min(getMinRobotToKickDistance(ballPosition, targetPosition, theRobotMap), getMinFieldBorderToKickDistance(ballPosition, targetPosition, theFieldDimensions)));
}

float KickUtils::getMinKickToGoalPostDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions)
{
  const float goalPostRadius = theFieldDimensions.goalPostRadius;

  const Vector2f ownLeftGoalPost = {theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal - goalPostRadius};
  const Vector2f ownRightGoalPost = {theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal + goalPostRadius};
  const Vector2f opponentLeftGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal - goalPostRadius};
  const Vector2f opponentRightGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal + goalPostRadius};

  std::vector<Vector2f> goalPostPositions = {};
  goalPostPositions.push_back(ownLeftGoalPost);
  goalPostPositions.push_back(ownRightGoalPost);
  goalPostPositions.push_back(opponentLeftGoalPost);
  goalPostPositions.push_back(opponentRightGoalPost);

  return getMinDistance(ballPosition, targetPosition, goalPostPositions);
}

float KickUtils::getMinRobotToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const RobotMap& theRobotMap)
{
  std::vector<Vector2f> robotPositions = {};
  for (auto& robot : theRobotMap.robots)
  {
    robotPositions.push_back(robot.pose.translation);
  }
  return getMinDistance(ballPosition, targetPosition, robotPositions);
}

float KickUtils::getMinDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const std::vector<Vector2f>& obstacles)
{
  // TODO Method is very inefficient

  const Vector2f kickDirection = (targetPosition - ballPosition).normalized();

  Geometry::Line kickLine = {ballPosition, kickDirection};
  Geometry::Line ballBorder = {ballPosition, Vector2f(kickDirection).rotateRight()};
  Geometry::Line targetBorder = {targetPosition, Vector2f(kickDirection).rotateLeft()};

  float minDistance = std::numeric_limits<float>::infinity();

  for (const Vector2f& obstacle : obstacles)
  {
    float distance = Geometry::distance(targetPosition, obstacle);
    distance = std::min(distance, Geometry::distance(ballPosition, obstacle));

    if (Geometry::isPointLeftOfLine(obstacle, ballBorder) && Geometry::isPointLeftOfLine(obstacle, targetBorder))
    {
      distance = std::min(distance, std::abs(Geometry::getDistanceToLine(kickLine, obstacle)));
    }

    if (distance < minDistance)
    {
      minDistance = distance;
    }
  }

  return minDistance;
}

float KickUtils::getMinFieldBorderToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions)
{
  const Vector2f oppLeftGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal};
  const Vector2f oppRightGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal};
  bool inOpponentGoal = Geometry::checkIntersectionOfLines(ballPosition, targetPosition, oppLeftGoalPost, oppRightGoalPost);
  if (inOpponentGoal)
  {
    return std::numeric_limits<float>::infinity();
  }

  const float yToSides = std::max(0.f, theFieldDimensions.yPosLeftSideline - std::abs(targetPosition.y()));
  const float xToGroundLines = std::max(0.f, theFieldDimensions.xPosOpponentGroundline - std::abs(targetPosition.x()));
  return std::min(yToSides, xToGroundLines);
}

std::tuple<Vector2f, Vector2f> KickUtils::getLeftAndRightTarget(const Vector2f& position, const Vector2f& target1, const Vector2f& target2)
{
  const Vector2f betweenTarget = target1 + (target2 - target1) / 2;
  const Geometry::Line betweenTargetLine = {position, (betweenTarget - position).normalized()};
  const bool target1Left = Geometry::isPointLeftOfLine(target1, betweenTargetLine);
  if (target1Left)
  {
    return {target1, target2};
  }
  else
  {
    return {target2, target1};
  }
}

/**
 * @return targets with a fixed angle to the ball. The distance between those targets varies with their distance to the ball
 */
std::vector<Vector2f> KickUtils::getTargetsWithEvenAngle(const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2)
{
  const Angle STEP_SIZE = 3_deg;
  const Angle MIN_STEP_SIZE = 1_deg;

  const Geometry::Line targetLine = {target1, (target2 - target1).normalized()};

  const Angle angle1 = (target1 - ballPosition).angle();
  const Angle angle2 = (target2 - ballPosition).angle();
  const Angle angle1Minus2 = angle1 - angle2;
  const Angle angleDiff = std::fabs(Angle(angle1Minus2).normalize());

  const Angle leftAngle = MathUtils::getLeftAngle(angle1, angle2);
  const int stepCount = (int)(angleDiff / STEP_SIZE);

  std::vector<Vector2f> targets = {};

  for (int stepNr = 0; stepNr < stepCount; ++stepNr)
  {
    const Angle targetAngle = leftAngle - STEP_SIZE * (float)stepNr;
    const Vector2f direction = MathUtils::angleToVector(targetAngle).normalized();
    Vector2f target = {};
    VERIFY(Geometry::getIntersectionOfLines(targetLine, {ballPosition, direction}, target));
    targets.push_back(target);
  }

  if (angleDiff - STEP_SIZE * (float)stepCount < MIN_STEP_SIZE) // If there is leftover target 2 will be missing
  {
    targets.push_back(target2);
  }

  return targets;
}

/**
 * @return targets with a fixed distance between each other. The angle to those targets varies with their distance to the ball
 */
std::vector<Vector2f> KickUtils::getTargetsWithEvenDistance(const Vector2f& target1, const Vector2f& target2, const int stepSize)
{
  const Vector2f target1to2 = target2 - target1;
  const Vector2f direction = target1to2.normalized();
  const int distance = (int)target1to2.norm();

  const int stepCount = (int)(distance / stepSize);

  std::vector<Vector2f> targets = {};

  for (int stepNr = 0; stepNr < stepCount; ++stepNr)
  {
    const Vector2f target = target1 + direction * stepSize * stepNr;
    targets.push_back(target);
  }

  if (distance % stepSize == 0) // If there is leftover target 2 will be missing
  {
    targets.push_back(target2);
  }

  return targets;
}

std::optional<Vector2f> KickUtils::getStraightAheadTarget(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2)
{
  const Angle angle1 = (target1 - ballPosition).angle();
  const Angle angle2 = (target2 - ballPosition).angle();
  const Angle angle = (ballPosition - playerPosition).angle();
  if (angle < angle1 && angle < angle2)
  {
    return std::nullopt;
  }
  if (angle > angle1 && angle > angle2)
  {
    return std::nullopt;
  }
  const Geometry::Line targetLine = {target1, (target2 - target1).normalized()};
  const Vector2f direction = MathUtils::angleToVector(angle).normalized();
  Vector2f target = {};
  const bool intersected = Geometry::getIntersectionOfLines(targetLine, {ballPosition, direction}, target);
  if (intersected)
  {
    return target;
  }
  OUTPUT_WARNING("StraightAheadTarget no intersection");
  return std::nullopt;
}

std::vector<Kick*> KickUtils::unpack(const std::vector<std::unique_ptr<Kick>>& kicks)
{
  std::vector<Kick*> unpackedKicks;
  for (const auto& kickUniquePointer : kicks)
  {
    Kick* kick = kickUniquePointer.get();
    unpackedKicks.push_back(kick);
  }
  return unpackedKicks;
}
