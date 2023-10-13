#include "Tools/Math/Geometry.h"
#include "Filterer.h"

Filterer& Filterer::filterKickPoseBlockedByRobots(const RobotMap& theRobotMap)
{
  poseBlockedByRobotFilter = [&theRobotMap](SelectablePose& selectablePose)
  {
    const float ROBOT_RADIUS = 200.f; // TODO Constant
    const float adjustedRobotRadius = ROBOT_RADIUS * 5 / 8; // Should be small to avoid accidental not kicking
    const float minRobotDistance = 2 * adjustedRobotRadius;
    for (const auto& robot : theRobotMap.robots)
    {
      const float distance = Geometry::distance(robot.pose.translation, selectablePose.pose.translation);
      if (distance < minRobotDistance)
      {
        return true;
      }
    }
    return false;
  };
  return *this;
}

Filterer& Filterer::filterKickPoseBlockedByGoalPost(const FieldDimensions& theFieldDimensions)
{
  poseBlockedByRobotFilter = [&theFieldDimensions](SelectablePose& selectablePose)
  {
    const float robotRadius = 150.f;
    const float offset = theFieldDimensions.goalPostRadius + robotRadius;
    const float fieldMaxX = theFieldDimensions.xPosOpponentGroundline - robotRadius;
    const float leftGoalPost_left = theFieldDimensions.yPosLeftGoal + offset;
    const float leftGoalPost_right = theFieldDimensions.yPosLeftGoal - offset;
    const float rightGoalPost_left = theFieldDimensions.yPosRightGoal + offset;
    const float rightGoalPost_right = theFieldDimensions.yPosRightGoal - offset;

    const Vector2f kickPosition = selectablePose.pose.translation;
    const float x = kickPosition.x();
    if (-fieldMaxX < x && x < fieldMaxX)
    {
      return false; // Is in field in x direction
    }
    const float y = kickPosition.y();
    if (y > leftGoalPost_left)
    {
      return false;
    }
    if (leftGoalPost_right > y && y > rightGoalPost_left)
    {
      return false;
    }
    if (rightGoalPost_right > y)
    {
      return false;
    }
    return true;
  };
  return *this;
}
