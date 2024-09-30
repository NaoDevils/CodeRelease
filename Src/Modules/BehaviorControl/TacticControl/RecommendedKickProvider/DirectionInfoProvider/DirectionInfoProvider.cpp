#include "DirectionInfoProvider.h"

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"

DirectionInfoProvider::DirectionInfoProvider() = default;

void DirectionInfoProvider::update(DirectionInfo& directionInfo)
{
  DECLARE_DEBUG_DRAWING("behavior:DirectionInfoProvider:directionInfo", "drawingOnField");

  if (skips < SKIP_UPDATES)
  {
    skips += 1;
    return;
  }
  skips = 0;

  Vector2f ballPosition = theBallSymbols.ballPositionField;

  update(directionInfo, stepAngle, ballPosition, theFieldDimensions, theRobotMap);

  directionInfo.draw(ballPosition);
}

void DirectionInfoProvider::update(DirectionInfo& directionInfo, const Angle stepAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  if (directionInfo.angles.empty())
  {
    directionInfo.stepAngle = stepAngle;
    for (Angle angle = -180_deg; angle < 180_deg; angle = angle + stepAngle)
    {
      directionInfo.angles.push_back(angle);
    }
  }

  const std::vector<Geometry::Circle> obstacles = getObstacles(theFieldDimensions, theRobotMap);

  directionInfo.blockedDistances.clear();
  directionInfo.intoGoalKickOutsideDistances.clear();
  directionInfo.intoKickInOutsideDistances.clear();
  directionInfo.intoCornerKickOutsideDistances.clear();
  directionInfo.intoOpponentGoalDistances.clear();
  directionInfo.intoOwnGoalDistances.clear();
  for (const Angle angle : directionInfo.angles)
  {
    const Vector2f direction = MathUtils::angleToVector(angle); // todo save these for better performance

    const float blockedDistance = getBlockedDistance(angle, ballPosition, obstacles);
    directionInfo.blockedDistances.push_back(blockedDistance);

    const float noGoalNetIntoOpponentGoalDistance = FieldUtils::getDistanceToOpponentsGoal(ballPosition, direction, theFieldDimensions);
    if (noGoalNetIntoOpponentGoalDistance >= 0.f)
    {
      directionInfo.intoGoalKickOutsideDistances.push_back(-1.f);
      directionInfo.intoKickInOutsideDistances.push_back(-1.f);
      directionInfo.intoCornerKickOutsideDistances.push_back(-1.f);
      directionInfo.intoOpponentGoalDistances.push_back(noGoalNetIntoOpponentGoalDistance);
      directionInfo.intoOwnGoalDistances.push_back(-1.f);
      continue;
    }

    const float noGoalNetIntoOwnGoalDistance = FieldUtils::getDistanceToOwnGoal(ballPosition, direction, theFieldDimensions);
    if (noGoalNetIntoOwnGoalDistance >= 0.f)
    {
      directionInfo.intoGoalKickOutsideDistances.push_back(-1.f);
      directionInfo.intoKickInOutsideDistances.push_back(-1.f);
      directionInfo.intoCornerKickOutsideDistances.push_back(-1.f);
      directionInfo.intoOpponentGoalDistances.push_back(-1.f);
      directionInfo.intoOwnGoalDistances.push_back(noGoalNetIntoOwnGoalDistance);
      continue;
    }

    auto [goalKickDistance, kickInDistance, cornerKickDistance] = getTowardsBordersDistances(angle, ballPosition, theFieldDimensions);
    directionInfo.intoGoalKickOutsideDistances.push_back(goalKickDistance);
    directionInfo.intoKickInOutsideDistances.push_back(kickInDistance);
    directionInfo.intoCornerKickOutsideDistances.push_back(cornerKickDistance);
    directionInfo.intoOpponentGoalDistances.push_back(-1.f);
    directionInfo.intoOwnGoalDistances.push_back(-1.f);
  }
}

std::vector<Geometry::Circle> DirectionInfoProvider::getObstacles(const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
{
  std::vector<Geometry::Circle> obstacles = {};

  // Add robots as obstacles
  const float ROBOT_RADIUS = 100.f;
  for (auto& robot : theRobotMap.robots)
  {
    obstacles.emplace_back(robot.pose.translation, ROBOT_RADIUS);
  }

  // Add goal posts as obstacles
  const float goalPostRadius = theFieldDimensions.goalPostRadius;
  const Vector2f ownLeftGoalPost = {theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal};
  const Vector2f ownRightGoalPost = {theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal};
  const Vector2f opponentLeftGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal};
  const Vector2f opponentRightGoalPost = {theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal};
  obstacles.emplace_back(ownLeftGoalPost, goalPostRadius);
  obstacles.emplace_back(ownRightGoalPost, goalPostRadius);
  obstacles.emplace_back(opponentLeftGoalPost, goalPostRadius);
  obstacles.emplace_back(opponentRightGoalPost, goalPostRadius);

  return obstacles;
}

float DirectionInfoProvider::getBlockedDistance(const Angle angle, const Vector2f& ballPosition, const std::vector<Geometry::Circle>& obstacles)
{
  const Vector2f direction = MathUtils::angleToVector(angle);
  const Geometry::Line line = {ballPosition, direction};

  float distance = std::numeric_limits<float>::max();
  bool blocked = false;
  for (const Geometry::Circle& obstacle : obstacles)
  {
    Vector2f blockedTarget = {};
    Vector2f dummy = {};
    if (Geometry::getIntersectionOfLineAndCircle(line, obstacle, dummy, blockedTarget))
    {
      const float firstIntersectionAngle = (blockedTarget - ballPosition).angle();
      if (MathUtils::getLargerMinusSmallerAngleDiff(angle, firstIntersectionAngle) > 5_deg)
      {
        // Intersection on wrong side
        continue;
      }
      const float firstIntersectionDistance = Geometry::distance(ballPosition, blockedTarget);
      if (firstIntersectionDistance < distance)
      {
        distance = firstIntersectionDistance;
        blocked = true;
      }
    }
  }
  if (blocked)
  {
    return distance;
  }
  else
  {
    return -1.f;
  }
}

/**
 * @return viewing from own teams side goalKickDistance, kickInDistance, cornerKickDistance
 */
std::tuple<float, float, float> DirectionInfoProvider::getTowardsBordersDistances(const Angle directionAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
{
  auto [toFieldBorderDistance, fieldBorderIndex] = getToFieldBorderDistanceAndFieldBorderIndex(directionAngle, ballPosition, theFieldDimensions);
  if (fieldBorderIndex == -1)
  {
    return {-1.f, -1.f, -1.f};
  }
  if (fieldBorderIndex == 0)
  {
    return {toFieldBorderDistance, -1.f, -1.f};
  }
  if (fieldBorderIndex == 1)
  {
    return {-1.f, toFieldBorderDistance, -1.f};
  }
  if (fieldBorderIndex == 2)
  {
    return {-1.f, -1.f, toFieldBorderDistance};
  }
  if (fieldBorderIndex == 3)
  {
    return {-1.f, toFieldBorderDistance, -1.f};
  }
  throw std::exception();
}

/**
 * @return distance to the closest field border in direction of the directionAngle. If the ball is outside: The closest line to the ball are ignored when calculating the distance.
 * The ball may be outside because it is a predicted ball, a false recognition or the referee was too slow to remove it.
 */
std::tuple<float, int> DirectionInfoProvider::getToFieldBorderDistanceAndFieldBorderIndex(const Angle directionAngle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
{
  const float MAX_DISTANCE = 3 * theFieldDimensions.xPosOpponentGroundline;

  enum ReturnDistanceEnum
  {
    YES_BECAUSE_BALL_INSIDE, // transitions to NO_BECAUSE_BALL_OUTSIDE and YES_BECAUSE_BALL_OUTSIDE_BUT_THE_DIRECTION_POINTS_TOWARDS_THE_FIELD
    NO_BECAUSE_BALL_OUTSIDE, // transitions to YES_BECAUSE_BALL_OUTSIDE_BUT_THE_DIRECTION_POINTS_TOWARDS_THE_FIELD
    YES_BECAUSE_BALL_OUTSIDE_BUT_THE_DIRECTION_POINTS_TOWARDS_THE_FIELD // no transitions
  };

  ReturnDistanceEnum returnDistanceEnum = ReturnDistanceEnum::YES_BECAUSE_BALL_INSIDE;
  float minToFieldBorderDistance = std::numeric_limits<float>::max();
  int minToFieldBorderDistance_fieldBorderIndex = -1;
  int outside_fieldBorderIndex = -1;

  int fieldBorderIndex = 0;
  for (const auto& rawFieldLine : theFieldDimensions.fieldBorder.lines)
  {
    const Vector2f maxDistanceDirection = (rawFieldLine.to - rawFieldLine.from).normalized(MAX_DISTANCE);
    const Geometry::Line stretchedFieldLine = {Vector2f(rawFieldLine.from - maxDistanceDirection), 2 * maxDistanceDirection};

    if (Geometry::isPointLeftOfLine(ballPosition, stretchedFieldLine))
    {
      // ball is inside the field if only this line is considered

      const Vector2f direction = MathUtils::angleToVector(directionAngle);
      const Geometry::Line directionLine = {ballPosition, direction.normalized(MAX_DISTANCE)};

      float intersectionFactor;
      const bool intersected = Geometry::getIntersectionOfRaysFactor(directionLine, stretchedFieldLine, intersectionFactor);
      if (intersected && intersectionFactor > 0)
      {
        const float intersectionDistance = intersectionFactor * MAX_DISTANCE;
        if (minToFieldBorderDistance > intersectionDistance)
        {
          minToFieldBorderDistance = intersectionDistance;
          minToFieldBorderDistance_fieldBorderIndex = fieldBorderIndex;
        }
      }
    }
    else
    {
      // ball is outside the field if only this line is considered

      if (returnDistanceEnum == ReturnDistanceEnum::YES_BECAUSE_BALL_OUTSIDE_BUT_THE_DIRECTION_POINTS_TOWARDS_THE_FIELD)
      {
        // If the ball can be kicked in
        continue;
      }

      const Angle leftAngle = (rawFieldLine.from - ballPosition).angle();
      const Angle rightAngle = (rawFieldLine.to - ballPosition).angle();
      const bool towardsField = MathUtils::isBetweenAngles(directionAngle, leftAngle, rightAngle);
      if (towardsField)
      {
        returnDistanceEnum = ReturnDistanceEnum::YES_BECAUSE_BALL_OUTSIDE_BUT_THE_DIRECTION_POINTS_TOWARDS_THE_FIELD;
      }
      else
      {
        returnDistanceEnum = ReturnDistanceEnum::NO_BECAUSE_BALL_OUTSIDE;
        outside_fieldBorderIndex = fieldBorderIndex;
      }
    }
    fieldBorderIndex += 1;
  }
  if (returnDistanceEnum == ReturnDistanceEnum::NO_BECAUSE_BALL_OUTSIDE)
  {
    return {0.f, outside_fieldBorderIndex}; // technically there can be multiple borders the ball is outside over but in reality it does not matter
  }
  if (minToFieldBorderDistance_fieldBorderIndex == -1)
  {
    /*
     * A rare but reachable case.
     *
     * The ball can be outside and point towards the field but still have a minToFieldBorderDistance_fieldBorderIndex of -1.
     * This case occurs if the ball is VERY far outside.
     * The reason for this is that the intersection method uses finite lines.
     *
     * Handle this as if the ball was outside and the angle pointing outside.
     */
    return {-1.f, -1};
  }

  ASSERT(minToFieldBorderDistance_fieldBorderIndex != -1);
  return {minToFieldBorderDistance, minToFieldBorderDistance_fieldBorderIndex};
}

MAKE_MODULE(DirectionInfoProvider, modeling)
