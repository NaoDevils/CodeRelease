#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/HeatMapFunctions.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RobotMap.h"

class PositionInfoUtils
{
public:
  static float getSidesHeat(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    const float x = point.x();
    const float y = point.y();

    const float oppXDistance = theFieldDimensions.xPosOpponentGroundline - x;
    const float ownXDistance = x - theFieldDimensions.xPosOwnGroundline;
    const float leftYDistance = theFieldDimensions.yPosLeftSideline - y;
    const float rightYDistance = y - theFieldDimensions.yPosRightSideline;
    const float minDistance = std::min(oppXDistance, std::min(ownXDistance, std::min(leftYDistance, rightYDistance)));

    return std::pow(1.f - minDistance / theFieldDimensions.xPosOpponentGroundline, 2.f);
  }

  static Angle getAngleRawDiff(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f opponentLeftGoalPost = FieldUtils::getOpponentGoalLeftPost(theFieldDimensions);
    const Vector2f opponentRightGoalPost = FieldUtils::getOpponentGoalRightPost(theFieldDimensions);
    const Angle toOpponentLeftGoalPostAngle = (opponentLeftGoalPost - point).angle();
    const Angle toOpponentRightGoalPostAngle = (opponentRightGoalPost - point).angle();
    return MathUtils::getAngleSmallestDiff(toOpponentLeftGoalPostAngle, toOpponentRightGoalPostAngle);
  }

  static float getGoalsHeat(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    // toOwnGoalDistanceHeat
    const float toOwnGoalMaxDistance = (1.f / 2.f) * theFieldDimensions.xPosOpponentGroundline;
    const float toOwnGoalDistance = Geometry::distance(point, FieldUtils::getOwnGoalCenter(theFieldDimensions));
    const float toOwnGoalDistanceHeat = 1.f - std::min(toOwnGoalDistance, toOwnGoalMaxDistance) / toOwnGoalMaxDistance;
    ASSERT(toOwnGoalDistanceHeat >= 0.f);
    ASSERT(toOwnGoalDistanceHeat <= 1.f);

    // opponentAngleToGoalHeat
    const Angle minAngle = 0_deg; //getAngleRawDiff(borderPoint, theFieldDimensions);
    const Angle maxAngle = 60_deg;
    const Angle toOpponentGoalAngleRawDiff = getAngleRawDiff(point, theFieldDimensions);
    ASSERT(toOpponentGoalAngleRawDiff >= 0_deg);
    const Angle toOpponentGoalAngleDiff = std::max(minAngle, std::min(maxAngle, toOpponentGoalAngleRawDiff));
    float toOpponentGoalAngleHeat = (toOpponentGoalAngleDiff - minAngle) / (maxAngle - minAngle);
    if (point.x() < 1000.f)
    {
      toOpponentGoalAngleHeat = 0.f;
    }
    ASSERT(toOpponentGoalAngleHeat >= 0.f);
    ASSERT(toOpponentGoalAngleHeat <= 1.f);

    const float MAX_DISTANCE = 2 * theFieldDimensions.xPosOpponentGroundline;
    const float toBorderDistance = theFieldDimensions.xPosOpponentGroundline - point.x();
    const float toBorderHeat = 1.f - std::max(0.f, std::min(1.f, toBorderDistance / MAX_DISTANCE));
    ASSERT(toBorderHeat >= 0.f);
    ASSERT(toBorderHeat <= 1.f);

    const float OWN_GOAL = 0.2f;
    const float OPPONENT_GOAL = 0.2f;
    const float OTHER = OWN_GOAL + OPPONENT_GOAL;
    const float ownGoalsHeat = toOwnGoalDistanceHeat;
    const float generalGoalsHeat = toBorderHeat;
    const float opponentGoalsHeat = toOpponentGoalAngleHeat;
    const float goalsHeat = (OWN_GOAL - OWN_GOAL * ownGoalsHeat) + (1.f - OTHER) * generalGoalsHeat + OPPONENT_GOAL * opponentGoalsHeat;

    ASSERT(goalsHeat >= 0.f);
    ASSERT(goalsHeat <= 1.f);
    return goalsHeat;
  }

  static std::array<std::vector<Pose2f>, 2> getTeammateAndOtherRobots(const RobotMap& theRobotMap)
  {
    std::array<std::vector<Pose2f>, 2> ret;
    for (const auto& robot : theRobotMap.robots)
    {
      std::vector<Pose2f>& robots = (robot.robotType == RobotEstimate::teammateRobot ? ret[0] : ret[1]);
      robots.emplace_back(robot.pose);
    }
    return ret;
  }

  static std::tuple<float, float> getRobotHeatForPosition(const Vector2f& cellPosition, const Pose2f& robotPose, const Vector2f& goalCenter, const FieldDimensions& theFieldDimensions)
  {
    std::vector<Pose2f> robotPoses = {};
    robotPoses.push_back(robotPose);
    return getRobotHeatForPosition(cellPosition, robotPoses, goalCenter, theFieldDimensions);
  }

  static std::tuple<float, float> getRobotHeatForPosition(const Vector2f& cellPosition, const std::vector<Pose2f>& robotPoses, const Vector2f& goalCenter, const FieldDimensions& theFieldDimensions)
  {
    if (robotPoses.empty())
    {
      return {0.5f, 0.5f};
    }

    const float MAX_AREA_HEAT_DISTANCE = FieldUtils::getMaxDistanceOnField(theFieldDimensions);
    const Angle MAX_GOAL_KICK_HEAT_ANGLE = 180_deg;
    const float MAX_GOAL_KICK_HEAT_DISTANCE = 4000.f;

    float minDistance = MAX_AREA_HEAT_DISTANCE;
    float maxToGoalHeat = 0.f;

    for (const Pose2f& robotPose : robotPoses)
    {
      float distance = Geometry::distance(robotPose.translation, cellPosition);

      // find minDistance
      if (distance < minDistance)
      {
        minDistance = distance;
      }

      // find minDistanceToGoalKick
      const Angle cellPositionToGoalAngle = (goalCenter - cellPosition).angle();
      const Angle robotPositionToCellPositionAngle = (cellPosition - robotPose.translation).angle();
      const Angle angleDiff = MathUtils::getAngleSmallestDiff(cellPositionToGoalAngle, robotPositionToCellPositionAngle);
      const float angleMultiplier = std::pow(std::max(0.f, 1.f - angleDiff / MAX_GOAL_KICK_HEAT_ANGLE), 1 / 2.f);
      const float distanceMultiplier = std::pow(std::max(0.f, 1.f - distance / MAX_GOAL_KICK_HEAT_DISTANCE), 1.55f);
      const float toGoalHeat = angleMultiplier * distanceMultiplier;
      if (toGoalHeat > maxToGoalHeat)
      {
        maxToGoalHeat = toGoalHeat;
      }
    }

    float maxHeat = 1 - minDistance / MAX_AREA_HEAT_DISTANCE;
    maxHeat = std::pow(MathUtils::clamp_f(maxHeat, 0, 1), 2.f);

    ASSERT(maxToGoalHeat >= 0.f);
    ASSERT(maxToGoalHeat <= 1.f);

    return {maxHeat, maxToGoalHeat};
  }

  static std::vector<std::tuple<Pose2f, float>> getCrowdedRobots(const Vector2f& ballPosition, const RobotMap& theRobotMap)
  {
    const float BALL_MIN_DISTANCE = 1500.f;
    const float OTHER_MIN_DISTANCE = 1500.f;

    std::vector<std::tuple<Pose2f, float>> crowdedRobots = {};
    for (const auto& robot : theRobotMap.robots)
    {
      if (robot.robotType == RobotEstimate::teammateRobot)
      {
        const float toBallDistance = Geometry::distance(robot.pose.translation, ballPosition);
        float ballCrowdedHeat = 1.f - toBallDistance / BALL_MIN_DISTANCE;
        if (toBallDistance < BALL_MIN_DISTANCE)
        {
          ballCrowdedHeat = 1.f - toBallDistance / BALL_MIN_DISTANCE;
        }

        float minToRobotDistance = OTHER_MIN_DISTANCE;
        for (const auto& otherRobot : theRobotMap.robots)
        {
          if (robot.pose == otherRobot.pose)
          {
            continue;
          }
          const float toRobotDistance = Geometry::distance(robot.pose.translation, otherRobot.pose.translation);
          if (toRobotDistance < minToRobotDistance)
          {
            minToRobotDistance = toRobotDistance;
          }
        }
        float robotCrowdedHeat = 0.f;
        if (minToRobotDistance < OTHER_MIN_DISTANCE)
        {
          robotCrowdedHeat = 1.f - minToRobotDistance / OTHER_MIN_DISTANCE;
        }

        const float heat = std::max(ballCrowdedHeat, robotCrowdedHeat);
        if (heat > 0.f)
        {
          crowdedRobots.emplace_back(robot.pose, heat);
        }
      }
      else // opponent or undecided robots
      {
        // Opponents always count as crowded robots. This is to avoid that a cluster of opponents (where they run over each other) has a higher score
        // than a single opponent (than has control over the ball)
        crowdedRobots.emplace_back(robot.pose, 1.f);
      }
    }
    return crowdedRobots;
  }

  static float getCrowdedHeat(const Vector2f& cellPosition, const std::vector<std::tuple<Pose2f, float>>& crowdedRobots)
  {
    const float DONT_PASS_TO_AREA = 1000.f;

    float highestAdjustedHeat = 0.f;
    for (const auto& [pose, heat] : crowdedRobots)
    {
      const float toRobotDistance = Geometry::distance(cellPosition, pose.translation);
      if (toRobotDistance < DONT_PASS_TO_AREA)
      {
        const float multiplier = 1.f - toRobotDistance / DONT_PASS_TO_AREA;
        const float adjustedHeat = multiplier * heat;
        highestAdjustedHeat = std::max(highestAdjustedHeat, adjustedHeat);
      }
    }

    ASSERT(highestAdjustedHeat >= 0.f);
    ASSERT(highestAdjustedHeat <= 1.f);
    return highestAdjustedHeat;
  }
};
