#pragma once

#include "Representations/Modeling/HeatMapCollection.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Representations/Modeling/RobotMap.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"

class HeatMapUtils
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

  static float getGoalsHeat(const Vector2f& point, const FieldDimensions& theFieldDimensions)
  {
    const float EXPONENT = 0.5f;
    const float MAX_DISTANCE = theFieldDimensions.xPosOpponentFieldBorder * 0.9f;

    const bool onOpponentsSide = point.x() > 0;

    const Vector2f goalCenter = onOpponentsSide ? FieldUtils::getOpponentGoalCenter(theFieldDimensions) : FieldUtils::getOwnGoalCenter(theFieldDimensions);
    const float goalDistance = std::min(Geometry::distance(point, goalCenter), MAX_DISTANCE);

    const float fraction = std::pow(goalDistance / MAX_DISTANCE, EXPONENT);
    const float heat = onOpponentsSide ? (1 - fraction) : fraction;
    const float fieldSideAdjustedHeat = (onOpponentsSide ? 0.5f : 0.f) + 0.5f * heat;

    return fieldSideAdjustedHeat;
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

  static std::tuple<float, float> getRobotHeatForPosition(const Vector2f& cellPosition, const std::vector<Pose2f>& robotPoses, const Vector2f& goalCenter, const FieldDimensions& theFieldDimensions)
  {
    if (robotPoses.empty())
    {
      return {0.5f, 0.5f};
    }

    const float HALF_STEP_SIZE_X = HeatMap::getStepSizeX(theFieldDimensions) / 2.f;
    const float HALF_STEP_SIZE_Y = HeatMap::getStepSizeY(theFieldDimensions) / 2.f;
    const float MAX_DISTANCE = FieldUtils::getMaxDistanceOnField(theFieldDimensions);
    const Angle MAX_GOAL_KICK_HEAT_ANGLE = 180_deg;
    const float MAX_GOAL_KICK_HEAT_DISTANCE = 4000.f;

    float minDistance = MAX_DISTANCE;
    float maxGoalKickHeat = 0.f;

    for (const Pose2f& robotPose : robotPoses)
    {
      float distance = Geometry::distance(robotPose.translation, cellPosition);

      if (HeatMap::isInCell(robotPose.translation, cellPosition, HALF_STEP_SIZE_X, HALF_STEP_SIZE_Y)) // Because the position is not perfect and we want to be safe
      {
        return {1.0f, 1.0f};
      }

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
      const float goalKickHeat = angleMultiplier * distanceMultiplier;
      if (goalKickHeat > maxGoalKickHeat)
      {
        maxGoalKickHeat = goalKickHeat;
      }
    }

    float maxKickHeat = 1 - minDistance / MAX_DISTANCE;
    maxKickHeat = std::pow(MathUtils::clamp_f(maxKickHeat, 0, 1), 2.f);

    ASSERT(maxGoalKickHeat >= 0.f);
    ASSERT(maxGoalKickHeat <= 1.f);

    return {maxKickHeat, maxGoalKickHeat};
  }

  // DRAW ==============================================================================================================

  static void draw(const HeatMap& heatMap, const bool stretchColors, const FieldDimensions& theFieldDimensions)
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
    draw(drawFieldPositionVector, heatVector, stretchColors, theFieldDimensions);
  }

  static void draw(const std::vector<Vector2f>& positionVector, std::vector<float> heatVector, const bool stretchColors, const FieldDimensions& theFieldDimensions)
  {
    if (stretchColors)
    {
      MathUtils::stretch(heatVector);
    }

    const int stepSizeX = (int)HeatMap::getStepSizeX(theFieldDimensions);
    const int stepSizeY = (int)HeatMap::getStepSizeY(theFieldDimensions);
    for (int i = 0; i < HeatMap::CELL_COUNT; i++)
    {
      const Vector2f position = positionVector.at(i);
      const float heat = heatVector.at(i);

      const int alpha = 155;

      RECTANGLE2(DRAW_HEAT_MAP,
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
