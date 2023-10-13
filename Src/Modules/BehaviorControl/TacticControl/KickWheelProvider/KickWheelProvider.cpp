#include "KickWheelProvider.h"
#include "Tools/Math/Transformation.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h>

KickWheelProvider::KickWheelProvider() = default;

void KickWheelProvider::update(KickWheel& kickWheel)
{
  DECLARE_DEBUG_DRAWING("behavior:KickWheelProvider:kickWheel", "drawingOnField");

  if (kickWheel.angles.empty())
  {
    const Angle STEP_ANGLE = 1_deg;
    for (Angle angle = -180_deg; angle < 180_deg; angle = angle + STEP_ANGLE)
    {
      kickWheel.angles.push_back(angle);
    }
  }

  Vector2f ballPosition = theBallSymbols.ballPositionField;
  const std::vector<Geometry::Circle> obstacles = getObstacles();

  kickWheel.blockedDistances.clear();
  kickWheel.outsideDistances.clear();
  for (const Angle angle : kickWheel.angles)
  {
    const float blockedDistance = getBlockedDistance(angle, ballPosition, obstacles);
    kickWheel.blockedDistances.push_back(blockedDistance);
    const float outsideDistance = getOutsideDistance(angle, ballPosition, theFieldDimensions);
    kickWheel.outsideDistances.push_back(outsideDistance);
  }

  kickWheel.draw(ballPosition);
}

std::vector<Geometry::Circle> KickWheelProvider::getObstacles() const
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

float KickWheelProvider::getBlockedDistance(const Angle angle, const Vector2f& ballPosition, const std::vector<Geometry::Circle>& obstacles)
{
  const Vector2f direction = MathUtils::angleToVector(angle);
  const Geometry::Line line = {ballPosition, direction};

  float distance = std::numeric_limits<float>::max();
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
      }
    }
  }
  return distance;
}

/**
 * @param ballPosition has to be inside field
 */
float KickWheelProvider::getOutsideDistance(const Angle angle, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
{
  const Vector2f direction = MathUtils::angleToVector(angle);

  if (FieldUtils::isInDirectionOfOpponentsGoal(ballPosition, direction, theFieldDimensions))
  {
    return std::numeric_limits<float>::max();
  }

  const float MAX_DISTANCE = 3 * theFieldDimensions.xPosOpponentGroundline;
  const Geometry::Line directionLine = {ballPosition, direction.normalized(MAX_DISTANCE)};

  /*
   * 0 = in field,
   * -1 = outside at least one border and no border to kick in over,
   * 1 = outside at least one border and at least one border to kick in over
   * Can go from 0 to -1 or 1 and from -1 to 1 but never leave 1!
   */
  int ballOutsideFieldFlag = 0;
  float minIntersectionDistance = std::numeric_limits<float>::max();

  for (const auto& rawFieldLine : theFieldDimensions.fieldBorder.lines)
  {
    const Vector2f maxDistanceDirection = (rawFieldLine.to - rawFieldLine.from).normalized(MAX_DISTANCE);
    const Geometry::Line fieldLine = {Vector2f(rawFieldLine.from - maxDistanceDirection), 2 * maxDistanceDirection};

    if (Geometry::isPointLeftOfLine(ballPosition, fieldLine))
    {
      float intersectionFactor;
      const bool intersected = Geometry::getIntersectionOfRaysFactor(directionLine, fieldLine, intersectionFactor);
      if (intersected && intersectionFactor > 0)
      {
        const float intersectionDistance = intersectionFactor * MAX_DISTANCE;
        minIntersectionDistance = std::min(minIntersectionDistance, intersectionDistance);
      }
    }
    else
    {
      if (ballOutsideFieldFlag != 1)
      {
        const Angle leftAngle = (rawFieldLine.from - ballPosition).angle();
        const Angle rightAngle = (rawFieldLine.to - ballPosition).angle();
        const bool towardsField = MathUtils::isBetweenAngles(angle, leftAngle, rightAngle);
        if (towardsField)
        {
          ballOutsideFieldFlag = 1;
        }
        else
        {
          ballOutsideFieldFlag = -1;
        }
      }
    }
  }
  if (ballOutsideFieldFlag == -1)
  {
    return 0.f;
  }
  return minIntersectionDistance;
}

MAKE_MODULE(KickWheelProvider, modeling)
