#include "KickManager.h"

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Tools/Math/Eigen.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include <stdexcept>
#include <optional>
#include <utility>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BlockUtils.h>
#include <Representations/MotionControl/WalkingEngineParams.h>
#include <Tools/Debugging/Annotation.h>

void KickManager::update(const Vector2f& ballPosition, const RobotPoseAfterPreview& theRobotPoseAfterPreview, const WalkingEngineParams& theWalkingEngineParams)
{
  // updateLeftFootClosestToBall

  const float footDecisionHysteresis = 50.f; // TODO Constant

  const Pose2f leftFoot = Pose2f(theRobotPoseAfterPreview).translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
  const Pose2f rightFoot = Pose2f(theRobotPoseAfterPreview).translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);

  const float leftDistance = (ballPosition - leftFoot.translation).norm();
  const float rightDistance = (ballPosition - rightFoot.translation).norm();

  if (leftFootClosestToBall && rightDistance < leftDistance - footDecisionHysteresis)
    leftFootClosestToBall = false;
  else if (!leftFootClosestToBall && leftDistance < rightDistance - footDecisionHysteresis)
    leftFootClosestToBall = true;

  // ball moved
  const Vector2f ballMoved = (ballPosition - currentBallPosition);
  if (ballMoved.norm() > 200.f) // TODO Constant
  {
    stop();
  }
}

bool KickManager::kickInRangeCenter(Ballchaser& ballchaser,
    const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const Vector2f& target1,
    const Vector2f& target2,
    const int stepSize,
    const DistanceRequirement distanceRequirement,
    const bool optimizeForTime,
    const float minKickWidth,
    const float minFreeAroundTarget,
    std::vector<Kick*> kicks,
    const FieldDimensions& theFieldDimensions,
    const RobotMap& theRobotMap)
{
  const Vector2f target1ToTarget2 = target2 - target1;
  const float target1ToTarget2Distance = target1ToTarget2.norm();
  const int optAhead = (int)(target1ToTarget2Distance / 2);
  return kickCloseToOptimalPosition(ballchaser,
      playerPose,
      ballPosition,
      Geometry::Line(target1, target1ToTarget2.normalized()),
      0,
      optAhead,
      (int)target1ToTarget2Distance,
      stepSize,
      distanceRequirement,
      optimizeForTime,
      minKickWidth,
      minFreeAroundTarget,
      std::move(kicks),
      theFieldDimensions,
      theRobotMap);
}

bool KickManager::kickCloseToOptimalPosition(Ballchaser& ballchaser,
    const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const Geometry::Line& line,
    const int minAhead,
    const int optimalAhead,
    const int maxAhead,
    const int stepSize,
    const DistanceRequirement distanceRequirement,
    const bool optimizeForTime,
    const float minKickWidth,
    const float minFreeAroundTarget,
    std::vector<Kick*> kicks,
    const FieldDimensions& theFieldDimensions,
    const RobotMap& theRobotMap)
{
  ASSERT(minAhead <= optimalAhead);
  ASSERT(optimalAhead <= maxAhead);
  ASSERT(stepSize > 0);
  ASSERT(minKickWidth >= 0);
  ASSERT(minFreeAroundTarget >= 0);

  KickUtils::drawKickRange(ballPosition, line, minAhead, maxAhead);

  Vector2f target;
  int stepSizeSum = 0;
  bool testedNewTarget = true;
  while (testedNewTarget)
  {
    testedNewTarget = false;
    for (int neg = -1; neg <= 1; neg = neg + 2)
    {
      int ahead = optimalAhead + neg * stepSizeSum;

      if (minAhead > ahead)
      {
        continue;
      }
      if (ahead > maxAhead)
      {
        continue;
      }
      testedNewTarget = true;

      target = line.base + ahead * line.direction;

      auto [kick, realisticTarget] = getBestKick(playerPose, ballPosition, target, distanceRequirement, optimizeForTime, minKickWidth, minFreeAroundTarget, kicks, theFieldDimensions, theRobotMap);

      if (kick != nullptr)
      {
        kickTo(ballchaser, playerPose, ballPosition, realisticTarget, *kick);
        return true;
      }

      stepSizeSum += stepSize;
    }
  }
  return false;
}

std::tuple<Kick*, Vector2f> KickManager::getBestKick(const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const Vector2f& targetPosition,
    const DistanceRequirement distanceRequirement,
    const bool optimizeForTime,
    const float minKickWidth,
    const float minFreeAroundTarget,
    std::vector<Kick*>& kicks,
    const FieldDimensions& theFieldDimensions,
    const RobotMap& theRobotMap)
{
  const float distance = Geometry::distance(ballPosition, targetPosition);

  float minTime = std::numeric_limits<float>::max();
  Kick* minTimeKick = nullptr;
  Vector2f minTimeRealisticTarget;

  float minInaccuracy = std::numeric_limits<float>::max();
  Kick* minInaccuracyKick = nullptr;
  Vector2f minInaccuracyRealisticTarget;

  for (Kick* kick : kicks)
  {
    const Vector2f realisticTargetPosition = kick->getRealisticTarget(ballPosition, targetPosition);

    const bool hysteresis = isCurrent(kick, realisticTargetPosition);

    if (!KickUtils::fulfillsDistanceRequirements(*kick, distance, distanceRequirement, hysteresis))
    {
      continue;
    }

    if (BlockUtils::isKickBlocked(ballPosition, realisticTargetPosition, minKickWidth, hysteresis, theFieldDimensions, theRobotMap))
    {
      continue;
    }

    if (minFreeAroundTarget > 0 && BlockUtils::isTargetControlledByOpponent(ballPosition, realisticTargetPosition, minFreeAroundTarget, hysteresis, theRobotMap))
    {
      continue;
    }

    const float kickTime = KickUtils::getKickTime(*kick, playerPose, ballPosition, targetPosition, hysteresis, leftFootClosestToBall);

    float kickInaccuracy = kick->getHorizontalInaccuracy();

    if (kickTime < minTime || (kickTime == minTime && (minTimeKick == nullptr || kickInaccuracy < minTimeKick->getHorizontalInaccuracy())))
    {
      minTime = kickTime;
      minTimeKick = kick;
      minTimeRealisticTarget = realisticTargetPosition;
    }

    if (kickInaccuracy < minInaccuracy || (kickInaccuracy == minInaccuracy && (minInaccuracyKick == nullptr || kickTime < minInaccuracyKick->getTime(hysteresis))))
    {
      minInaccuracy = kickInaccuracy;
      minInaccuracyKick = kick;
      minInaccuracyRealisticTarget = realisticTargetPosition;
    }
  }

  if (optimizeForTime)
  {
    return std::tuple<Kick*, Vector2f>{minTimeKick, minTimeRealisticTarget};
  }
  else
  {
    return std::tuple<Kick*, Vector2f>{minInaccuracyKick, minInaccuracyRealisticTarget};
  }
}

void KickManager::stop()
{
  ANNOTATION("KickManagerStop", "The KickManager stopped the current kick!");
  currentKick = nullptr;
}

void KickManager::kickTo(Ballchaser& ballchaser, const ExecutableKicks& executableKicks)
{
  ASSERT(executableKicks.hasBest());
  ExecutableKick best = executableKicks.getBest();
  kickTo(ballchaser, executableKicks.playerPose, executableKicks.ballPosition, best.target, *best.kick);
}

void KickManager::kickTo(Ballchaser& ballchaser, const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, Kick& kick)
{
  CIRCLE(DRAW_EXECUTABLE_KICK_TARGET_AREA, targetPosition.x(), targetPosition.y(), kick.getInaccuracy(), 12, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
  ballchaser.log_kickName = kick.getName();

  Pose2f useKickPose = currentKickPose;
  if (currentKick && isCurrent(&kick, targetPosition))
  {
    const Vector2f ballMoved = (ballPosition - currentBallPosition);
    useKickPose = {useKickPose.rotation, useKickPose.translation + ballMoved};
  }
  else
  {
    currentKick = &kick;
    currentKickPose = currentKick->getKickPose(playerPose, ballPosition, targetPosition, leftFootClosestToBall);
    currentBallPosition = ballPosition;
    currentKickTarget = targetPosition;
    currentKickStarted = false;
  }

  const Angle optAngle = (useKickPose.translation - ballPosition).angle();
  const Angle leftOptAngle = optAngle + 45_deg;
  const Angle rightOptAngle = optAngle - 45_deg;
  const Angle currentAngle = (playerPose.translation - ballPosition).angle();
  currentKickStarted = currentKickStarted || (rightOptAngle < currentAngle && currentAngle < leftOptAngle);

  const Vector2f leftOptPosition = ballPosition + MathUtils::angleToVector(leftOptAngle) * 1000.f;
  const Vector2f rightOptPosition = ballPosition + MathUtils::angleToVector(rightOptAngle) * 1000.f;
  const Vector2f currentPosition = ballPosition + MathUtils::angleToVector(currentAngle) * 1000.f;
  LINE(DRAW_KICK_MANAGER_ACTIVATE_RANGE_NAME, ballPosition.x(), ballPosition.y(), leftOptPosition.x(), leftOptPosition.y(), 10, Drawings::solidPen, ColorRGBA::cyan);
  LINE(DRAW_KICK_MANAGER_ACTIVATE_RANGE_NAME, ballPosition.x(), ballPosition.y(), rightOptPosition.x(), rightOptPosition.y(), 10, Drawings::solidPen, ColorRGBA::cyan);
  LINE(DRAW_KICK_MANAGER_ACTIVATE_RANGE_NAME, ballPosition.x(), ballPosition.y(), currentPosition.x(), currentPosition.y(), 10, Drawings::solidPen, ColorRGBA::green);

  currentKick->perform(ballchaser, useKickPose, currentKickTarget, currentKickStarted);
}

bool KickManager::isActive()
{
  return currentKick != nullptr;
}

bool KickManager::isCurrent(const Kick* kick, const Vector2f& kickTarget)
{
  if (kick == currentKick)
  {
    return Geometry::distance(kickTarget, currentKickTarget) < 180.f; // TODO CONST
  }
  else
  {
    return false;
  }
}
