#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Geometry.h"
//#include <algorithm>

class HelperFunctions
{
public:
  static bool isPositionOccupied(
      const RobotPose& myRobotPose, const RobotMap& robotMap, const TeammateData& teammateData, float x, float y, float dist, bool useTeamMateData, bool useTeamMatesOnly = false)
  {
    Vector2f pos(x, y);
    for (auto& robot : robotMap.robots)
    {
      if ((robot.pose.translation - pos).norm() < dist && (pos - myRobotPose.translation).norm() < 1500)
        return true;
    }
    if (useTeamMateData)
    {
      for (auto& mate : teammateData.teammates)
      {
        if (mate.status >= TeammateReceived::Status::ACTIVE)
        {
          const Pose2f& pose = mate.robotPose;
          if ((pose.translation - pos).norm() < dist)
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  static void addActivePlayerPositions(const TeammateData& teammateData, std::vector<Pose2f>& positions)
  {
    for (auto& mate : teammateData.teammates)
      if (mate.status >= TeammateReceived::Status::ACTIVE)
        positions.push_back(mate.robotPose);
  }

  /* calculates the distance in mm adding penalties for bad approaches (obstacles, from side, ...) */
  static float calcDistanceModified(const BehaviorConfiguration& theBehaviorConfiguration, const RobotMap& theRobotMap, const bool& isKeeper, const Pose2f& fromPose, const Pose2f& targetOnField)
  {
    // initial : euclidian distance
    const Vector2f targetRelative(Transformation::fieldToRobot(fromPose, targetOnField.translation));
    const float euclidianDistance = targetRelative.norm();
    float distanceModified = euclidianDistance;

    // adding penalty for wrong rotation towards target - USEFUL? thinking about situations near target
    float targetRotationRelative = toDegrees(std::abs(Angle::normalize(targetOnField.rotation - fromPose.rotation)));
    distanceModified += targetRotationRelative * theBehaviorConfiguration.targetDistanceRobotRotFactor;

    // add penalty for difference of target rotation (on field) to robot pose rotation
    float rotationRobotToTargetPosition = std::abs(toDegrees(Angle::normalize(targetOnField.rotation - (targetOnField.translation - fromPose.translation).angle())));
    float addedDistance = //(euclidianDistance/(2*fieldDimensions.xPosOpponentGroundline))*
        (std::pow(rotationRobotToTargetPosition, 1.5f) / theBehaviorConfiguration.targetDistRobotToTargetAngleFactor);
    distanceModified += addedDistance;

    // obstacles (checking if obstacles are near the line from robot to target)
    Geometry::Line lineToTarget;
    lineToTarget.base = fromPose.translation;
    lineToTarget.direction = targetRelative;

    // for each obstacle in the way the distance is increased
    for (auto& robot : theRobotMap.robots)
    {
      Vector2f obstacleRelPos = Transformation::fieldToRobot(fromPose, robot.pose.translation);
      float distToTargetLine = std::abs(Geometry::getDistanceToLine(lineToTarget, robot.pose.translation));

      if (distToTargetLine < 400 // obstacle has to be near the robot
          // obstacle has to be closer than the target
          && obstacleRelPos.norm() < targetRelative.norm()
          // obstacle has to be in front of and not behind or next to the robot to increase the distance
          && std::abs(Angle::normalize(targetRotationRelative - obstacleRelPos.angle())) < pi_2)
      {
        distanceModified += theBehaviorConfiguration.targetDistanceObstacleFactor / (1 + sqr(distToTargetLine / 100));
      }
    }
    // Keeper gets a penalty if other players are only slightly farther away from the ball,
    // he should not go to far outside the goal
    if (isKeeper)
      distanceModified += 200;

    return (float)distanceModified;
  }

  static bool arePositionsCloseToEachOther(const Vector2f& pos1, const Vector2f& pos2, const float xOffset, const float yOffset)
  {
    return static_cast<float>(fabs(pos1.x() - pos2.x())) < xOffset && static_cast<float>(fabs(pos1.y() - pos2.y())) < yOffset;
  }
};
