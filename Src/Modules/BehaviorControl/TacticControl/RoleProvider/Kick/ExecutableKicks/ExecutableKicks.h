#pragma once

#include <utility>
#include <optional>
#include <vector>
#include <functional>
#include "Tools/Math/Geometry.h"
#include "ExecutableKick.h"

class ExecutableKicks
{

public:
  ExecutableKicks(Pose2f playerPose, Vector2f ballPosition, std::vector<ExecutableKick> executableKicks)
      : playerPose(std::move(playerPose)), ballPosition(std::move(ballPosition)), executableKicks(std::move(executableKicks))
  {
  }
  ExecutableKicks() {}
  virtual ~ExecutableKicks() = default;

  void insert(const ExecutableKicks& other);

  void insert(const std::vector<ExecutableKick>& other);

  ExecutableKicks filterBlocked(const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  ExecutableKicks filterOutside(const FieldDimensions& theFieldDimensions);

  ExecutableKicks filterTooFarBack(float minX);

  ExecutableKicks filterLeft(float maxY);

  ExecutableKicks filterRight(float minY);

  ExecutableKicks filterTooSharpForGoalKickAngles(const FieldDimensions& theFieldDimensions);

  ExecutableKicks reduceToBest(float timeFactor,
      float inaccuracyFactor,
      float widthFactor,
      float sidesHeatFactor,
      float opponentsGoalHeatFactor,
      float alliesHeatFactor,
      float alliesMaxHeatFactor,
      float alliesGoalKickHeatFactor,
      float opponentsHeatFactor,
      float opponentsMaxHeatFactor,
      float opponentsGoalKickHeatFactor,
      bool leftFootClosestToBall,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap);

  [[nodiscard]] bool hasBest() const;

  [[nodiscard]] ExecutableKick getBest() const;

  const Pose2f playerPose;
  const Vector2f ballPosition = Vector2f::Zero();

private:
  std::vector<ExecutableKick> executableKicks;

  ExecutableKicks filter(const std::function<bool(ExecutableKick)>& isToRemove);

  static void drawFree(const std::vector<Vector2f>& draw_targets, std::vector<float>& draw_scores);

  static void drawInGrid(const std::vector<Vector2f>& draw_targets, const std::vector<float>& draw_scores, const FieldDimensions& theFieldDimensions);

  static void drawKickMinObstacleWidth(const Vector2f& ballPosition, const Vector2f& targetPosition, float width);
};
