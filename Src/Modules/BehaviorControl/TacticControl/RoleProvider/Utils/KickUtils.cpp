#include "KickUtils.h"

#include "MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include <optional>

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

std::vector<Kick*> KickUtils::unpack(const std::vector<std::unique_ptr<Kick>>& kicks1, const std::vector<std::unique_ptr<Kick>>& kicks2)
{
  std::vector<Kick*> unpackedKicks;
  for (const auto& kickUniquePointer : kicks1)
  {
    Kick* kick = kickUniquePointer.get();
    unpackedKicks.push_back(kick);
  }
  for (const auto& kickUniquePointer : kicks2)
  {
    Kick* kick = kickUniquePointer.get();
    unpackedKicks.push_back(kick);
  }
  return unpackedKicks;
}

/**
 * Use robot model's foot positions intersecting with best hypothesis
 */
bool KickUtils::isBallTouched(const Vector2f& relativeBallPosition, const RobotModel& theRobotModel, const FieldDimensions& theFieldDimensions)
{
  const Pose3f& kickFoot = (theRobotModel.soleLeft.translation.z() > theRobotModel.soleRight.translation.z()) ? theRobotModel.soleLeft : theRobotModel.soleRight;

  const float footCenterYToBallDistance = relativeBallPosition.y() - kickFoot.translation.y();

  const float footCenterXToBallDistance = relativeBallPosition.x() - kickFoot.translation.x();
  const float footFrontXToBallDistance = footCenterXToBallDistance - 104.f;

  /*
   * The values used for calculation are fluctuating heavily. To avoid false
   * positives and because false negatives can be tolerated the ballRadius is
   * reduced
  */
  const float saferBallRadius = theFieldDimensions.ballRadius - 20.f;

  const bool ballTouched = std::abs(footCenterYToBallDistance) < saferBallRadius && std::abs(footFrontXToBallDistance) < saferBallRadius;

  return ballTouched;
}

/**
 * Two ways to trigger a kick are handled here: kick engine or in walk kick
 */
bool KickUtils::isBallKicked(const MotionInfo& theMotionInfo)
{
  const bool customStepKickStartedAndCantGetInterrupted = theMotionInfo.motion == MotionRequest::Motion::walk && theMotionInfo.walkKicking;
  const bool kickEngineKickStartedAndCantGetInterrupted = theMotionInfo.motion == MotionRequest::kick;
  return customStepKickStartedAndCantGetInterrupted || kickEngineKickStartedAndCantGetInterrupted;
}
