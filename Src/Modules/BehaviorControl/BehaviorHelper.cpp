/**
* \file BehviorHelper.cpp
* Thomas Klute
*/

#include <optional>
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "BehaviorHelper.h"
#include <Representations/Modeling/RobotMap.h>
#include <Representations/Configuration/FieldDimensions.h>

/**
 * @brief returns the opponent robot that has the min distance to the own goal center - this should be the goal keeper
 * @param map the robot map
 * @param fieldDimensions the field dimensions
 * @return the keepers pose or if no robot is found within the threshold distance: a fallback pose of the center of the opponent goal
*/
Pose2f BehaviorHelper::getOpponentKeeperPose(const RobotMap& map, const FieldDimensions& fieldDimensions)
{
  //a goalies distance to the center of the goal may be at maximum 1.2 the distance between goal center and goal post
  float maxDistanceThreshold = fieldDimensions.yPosLeftGoal * 1.2f;
  float minDistance = maxDistanceThreshold;
  //fallback position, if we are unable to localize the opponent keeper
  Pose2f keeperPos = Pose2f(pi, fieldDimensions.xPosOpponentFieldBorder, 0.f);
  Vector2f opGoalCenter = Vector2f(fieldDimensions.xPosOpponentFieldBorder, 0.f);
  for (auto& robot : map.robots)
  {
    if (robot.robotType != RobotEstimate::teammateRobot)
    {
      float dist = (robot.pose.translation - opGoalCenter).norm();
      if (dist < minDistance)
      {
        minDistance = dist;
        keeperPos = robot.pose;
      }
    }
  }
  return keeperPos;
}
