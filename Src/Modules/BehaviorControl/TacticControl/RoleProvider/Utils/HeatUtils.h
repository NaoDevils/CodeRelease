#pragma once

#include <Representations/Modeling/HeatMapCollection.h>
#include "Representations/Configuration/FieldDimensions.h"
#include "MathUtils.h"
#include "Representations/Modeling/RobotMap.h"
#include "PositionUtils.h"
#include "FieldUtils.h"

class HeatUtils
{
public:
  static float getSidesHeat(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    const float x = MathUtils::clamp_f(point.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOpponentGroundline);
    const float y = MathUtils::clamp_f(point.y(), theFieldDimensions.yPosRightSideline, theFieldDimensions.yPosLeftSideline);

    const float oppXDistance = theFieldDimensions.xPosOpponentGroundline - x;
    const float ownXDistance = x - theFieldDimensions.xPosOwnGroundline;
    const float leftYDistance = theFieldDimensions.yPosLeftSideline - y;
    const float rightYDistance = y - theFieldDimensions.yPosRightSideline;
    const float minDistance = std::min(oppXDistance, std::min(ownXDistance, std::min(leftYDistance, rightYDistance)));

    return 1 - minDistance / theFieldDimensions.xPosOpponentGroundline;
  }

  static float getOpponentsGoalHeat(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f opponentGoalCenter = FieldUtils::getOpponentGoalCenter(theFieldDimensions);
    const float opponentsGoalHeat = Geometry::distance(point, opponentGoalCenter);
    return 1 - (opponentsGoalHeat / (2 * theFieldDimensions.xPosOpponentGoal));
  }

  static std::tuple<float, float, float, float, float, float> getRobotHeat(const Vector2f& position, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap, const RobotPose& theRobotPose)
  {
    std::vector<Pose2f> allies = {};
    //allies.emplace_back(theRobotPose.translation);
    std::vector<Pose2f> opponents = {};
    for (auto& robot : theRobotMap.robots)
    {
      if (robot.robotType == RobotEstimate::teammateRobot)
        allies.emplace_back(robot.pose);
      else
        opponents.emplace_back(robot.pose);
    }

    auto [alliesHeat, alliesMaxHeat, alliesGoalKickHeat] = getHeat(allies, position, FieldUtils::getOpponentGoalCenter(theFieldDimensions), theFieldDimensions);
    auto [opponentsHeat, opponentsMaxHeat, opponentsGoalKickHeat] = getHeat(opponents, position, FieldUtils::getOwnGoalCenter(theFieldDimensions), theFieldDimensions);

    return {alliesHeat, alliesMaxHeat, alliesGoalKickHeat, opponentsHeat, opponentsMaxHeat, opponentsGoalKickHeat};
  }

  static std::tuple<float, float, float> getHeat(const std::vector<Pose2f>& robotPoses, const Vector2f& position, const Vector2f& goalCenter, const FieldDimensions& theFieldDimensions)
  {
    if (robotPoses.empty())
    {
      return {0.5f, 0.5f, 0.5f};
    }

    // TODO ADD if angle of robots known MAX_TURN_DISTANCE const float MAX_TURN_DISTANCE = 2500.f; // TODO Constant
    const float MAX_WALK_AROUND_BALL_DISTANCE = 2000.f; // TODO Constant

    const float maxDistance = FieldUtils::getMaxDistanceOnField(theFieldDimensions);
    const float maxDistanceToGoalKick = maxDistance + /*TODO ADD if angle of robots known MAX_TURN_DISTANCE +*/ MAX_WALK_AROUND_BALL_DISTANCE;

    float distanceSum = 0;
    float distanceMin = maxDistance;
    float distanceToGoalKickMin = maxDistanceToGoalKick;

    for (const Pose2f& robotPose : robotPoses)
    {
      float distance = Geometry::distance(robotPose.translation, position);

      distanceSum += distance;

      if (distance < distanceMin)
      {
        distanceMin = distance;
      }

      const float turnDistanceToGoalKick = getTurnAroundPositionDistance(robotPose.translation, position, goalCenter, MAX_WALK_AROUND_BALL_DISTANCE);
      const float canKickDistance = std::max(distance, 500.f);
      const float distanceToGoalKick = canKickDistance + turnDistanceToGoalKick;
      if (distanceToGoalKick < distanceToGoalKickMin)
      {
        distanceToGoalKickMin = distanceToGoalKick;
      }
    }

    const float distanceAverage = distanceSum / (float)robotPoses.size();
    float heat = 1 - distanceAverage / maxDistance;
    heat = std::pow(MathUtils::clamp_f(heat, 0, 1), 2.f); // TODO Constant

    float maxHeat = 1 - distanceMin / maxDistance;
    maxHeat = std::pow(MathUtils::clamp_f(maxHeat, 0, 1), 2.f); // TODO Constant

    float goalKickHeat = 1 - distanceToGoalKickMin / maxDistanceToGoalKick;
    goalKickHeat = std::pow(MathUtils::clamp_f(goalKickHeat, 0.f, 1.f), 2.f); // TODO Constant

    return {heat, maxHeat, goalKickHeat};
  }

  static float getTurnDistanceToBall(const Vector2f& playerPosition, const Vector2f& ballPosition, const float maxTurnToDistance)
  {
    const Angle turnToTargetAngleDiff = std::fabs(PositionUtils::getTurnRobotByAngle(playerPosition, ballPosition));
    return (turnToTargetAngleDiff / 180_deg) * maxTurnToDistance;
  }

  /**
   * @return The distance the player has to walk around the turnAroundPosition to face the turnToPosition
   */
  static float getTurnAroundPositionDistance(const Vector2f& playerPosition, const Vector2f& turnAroundPosition, const Vector2f& turnToPosition, const float maxWalkAroundDistance)
  {
    const Pose2f assumedPose = {(turnAroundPosition - playerPosition).angle(), turnAroundPosition};
    const Angle angleDiff = std::fabs(PositionUtils::getTurnRobotByAngle(assumedPose, turnToPosition));
    return (angleDiff / 180_deg) * maxWalkAroundDistance;
  }

  // DRAW ==============================================================================================================

  static void draw(const HeatMap& heatMap, const FieldDimensions& theFieldDimensions)
  {
    const float offsetX = HeatMap::getStepSizeX(theFieldDimensions) / 2;
    const float offsetY = HeatMap::getStepSizeY(theFieldDimensions) / 2;

    std::vector<Vector2f> drawFieldPositionVector;
    std::vector<float> heatVector;
    for (int i = 0; i < HeatMap::CELL_COUNT; i++)
    {
      const Vector2f fieldPosition = HeatMap::indexToField(i, theFieldDimensions);
      const Vector2f drawFieldPosition = {fieldPosition.x() - offsetX, fieldPosition.y() - offsetY};
      drawFieldPositionVector.push_back(drawFieldPosition);
      const float heat = heatMap.getHeat(i);
      heatVector.push_back(heat);
    }
    draw(drawFieldPositionVector, heatVector, theFieldDimensions);
  }

  static void draw(std::vector<Vector2f> positionVector, std::vector<float> heatVector, const FieldDimensions& theFieldDimensions)
  {
    //MathUtils::stretch(heatVector);

    const int stepSizeX = (int)HeatMap::getStepSizeX(theFieldDimensions);
    const int stepSizeY = (int)HeatMap::getStepSizeY(theFieldDimensions);
    for (int i = 0; i < HeatMap::CELL_COUNT; i++)
    {
      const Vector2f position = positionVector.at(i);
      const float heat = heatVector.at(i);

      const int alpha = 155;

      RECTANGLE2("module:HeatMapProvider:HeatMap",
          Vector2i((int)position.x(), (int)position.y()),
          stepSizeX,
          stepSizeY,
          0,
          10,
          Drawings::noBrush,
          ColorRGBA(static_cast<unsigned char>(255 - alpha), static_cast<unsigned char>(alpha), 0),
          Drawings::solidBrush,
          ColorRGBA((unsigned char)(255 * (1 - heat)), (unsigned char)(255 * heat), 0, alpha));
    }
  }
};
