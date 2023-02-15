#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <algorithm>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/KickRange.h>

STREAMABLE(HeatMap,
  PROTECT(
public:
  static constexpr int CELL_COUNT_X = 30;
  static constexpr int CELL_COUNT_Y = 20;
  static constexpr int CELL_COUNT = CELL_COUNT_X * CELL_COUNT_Y;

  HeatMap()
  {
    for (int i = 0; i < CELL_COUNT; i++)
    {
      heatVector.push_back(0.f);
    }
  }

  void reset()
  {
    for (int i = 0; i < CELL_COUNT; i++)
    {
      heatVector[i] = 0.f;
    }
  }

  class Area
  {
  public:
    bool add(const int index, const float relativeDistance)
    {
      if (std::find(indexes.begin(), indexes.end(), index) != indexes.end())
      {
        return false;
      }
      indexes.push_back(index);
      relativeDistances.push_back(relativeDistance);
      relativeDistanceSum += relativeDistance;
      return true;
    }
    [[nodiscard]] size_t size() const
    {
      return indexes.size();
    }
    [[nodiscard]] int getIndex(const size_t i) const
    {
      return indexes.at(i);
    }
    [[nodiscard]] float getRelativeDistance(const size_t i) const
    {
      return relativeDistances.at(i);
    }
    [[nodiscard]] float getRelativeDistanceSum() const
    {
      return relativeDistanceSum;
    }
  private:
    std::vector<int> indexes = {};
    std::vector<float> relativeDistances = {};
    float relativeDistanceSum = 0;
  };

  [[nodiscard]] static Area getArea(const Vector2f& position, const float radius,
                                    const FieldDimensions& theFieldDimensions)
  {
    ASSERT(radius > 0);

    const float minX = position.x() - radius;
    const float maxX = position.x() + radius;
    const float minY = position.y() - radius;
    const float maxY = position.y() + radius;

    Area area = {};
    for (int cellIndex = 0; cellIndex < CELL_COUNT; ++cellIndex)
    {
      const Vector2f cellPosition = indexToField(cellIndex, theFieldDimensions);
      const float x = cellPosition.x();
      const float y = cellPosition.y();

      if (x < minX || x > maxX || y < minY || y > maxY) // Bounding box for efficiency
      {
        continue;
      }

      const float cellDistance = Geometry::distance(cellPosition, position);

      if (radius < cellDistance) // Not in circle
      {
        continue;
      }

      const float relativeDistance = 1 - cellDistance;

      area.add(cellIndex, relativeDistance);
    }
    return area;
  }

  [[nodiscard]] float getHeat(const Vector2f& position, const float radius, const FieldDimensions& theFieldDimensions) const
  {
    return getHeat(getArea(position, radius, theFieldDimensions), theFieldDimensions);
  }

  [[nodiscard]] float getHeat(const Area& area, const FieldDimensions& theFieldDimensions) const
  {
    const size_t size = area.size();

    if (size == 0)
    {
      return 0;
    }

    float relativeHeatSum = 0;
    for (size_t i = 0; i < size; i++)
    {
      const float cellHeat = getHeat(area.getIndex(i));
      const float relativeHeat = cellHeat * area.getRelativeDistance(i);
      relativeHeatSum += relativeHeat;
    }

    const float heat = relativeHeatSum / area.getRelativeDistanceSum();
    ASSERT(heat + 0.0001 > 0);
    ASSERT(heat - 0.0001 < 1);
    return MathUtils::clamp_f(heat, 0, 1);
  }

  [[nodiscard]] float getHeat(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const
  {
    const int index = fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions);
    return getHeat(index);
  }

  [[nodiscard]] inline float getHeat(const int index) const
  {
    return heatVector[index];
  }

  void updateHeat(const float takeNewPercent, float newHeat, const int index, const FieldDimensions& theFieldDimensions)
  {
    const float oldHeat = getHeat(index);
    const float diff = oldHeat / newHeat;
    if (diff > 0.99 || diff < 1.01)
    {
      newHeat = oldHeat * (1.f - takeNewPercent) + newHeat * takeNewPercent;
    }
    setHeat(newHeat, index, theFieldDimensions);
  }

  void setHeat(const float heat, const float worldX, const float worldY, const FieldDimensions& theFieldDimensions)
  {
    const int index = fieldToIndex(worldX, worldY, theFieldDimensions);
    setHeat(heat, index, theFieldDimensions);
  }

  inline void setHeat(const float heat, const int index, const FieldDimensions& theFieldDimensions)
  {
    heatVector[index] = heat;
  }

  static float getStepSizeX(const FieldDimensions& theFieldDimensions)
  {
    const float maxNonNegativeWorld = 2 * theFieldDimensions.xPosOpponentGroundline;
    return maxNonNegativeWorld / CELL_COUNT_X;
  }

  static float getStepSizeY(const FieldDimensions& theFieldDimensions)
  {
    const float maxNonNegativeWorld = 2 * theFieldDimensions.yPosLeftSideline;
    return maxNonNegativeWorld / CELL_COUNT_Y;
  }

  [[nodiscard]] static int fieldToIndex(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions)
  {
    return fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions);
  }

  [[nodiscard]] static int fieldToIndex(const float worldX, const float worldY, const FieldDimensions& theFieldDimensions)
  {
    const float nonNegativeWorldX = worldX + theFieldDimensions.xPosOpponentGroundline;
    const float nonNegativeWorldY = worldY + theFieldDimensions.yPosLeftSideline;
    const float maxNonNegativeWorldX = 2 * theFieldDimensions.xPosOpponentGroundline;
    const float maxNonNegativeWorldY = 2 * theFieldDimensions.yPosLeftSideline;
    const float distancePercentX = nonNegativeWorldX / maxNonNegativeWorldX;
    const float distancePercentY = nonNegativeWorldY / maxNonNegativeWorldY;
    const int indexX = (int) MathUtils::clamp_f(distancePercentX * CELL_COUNT_X, 0, CELL_COUNT_X - 1);
    const int indexY = (int) MathUtils::clamp_f(distancePercentY * CELL_COUNT_Y, 0, CELL_COUNT_Y - 1);
    return indexX * CELL_COUNT_Y + indexY;
  }

  [[nodiscard]] static Vector2f indexToField(const int index, const FieldDimensions& theFieldDimensions)
  {
    const float stepSizeX = getStepSizeX(theFieldDimensions);
    const float stepSizeY = getStepSizeY(theFieldDimensions);
    const int indexX = index / CELL_COUNT_Y;
    const int indexY = index % CELL_COUNT_Y;
    const float nonNegativeWorldX = stepSizeX * (float) indexX + stepSizeX / 2;
    const float nonNegativeWorldY = stepSizeY * (float) indexY + stepSizeY / 2;
    const float worldX = nonNegativeWorldX - theFieldDimensions.xPosOpponentGroundline;
    const float worldY = nonNegativeWorldY - theFieldDimensions.yPosLeftSideline;
    return {worldX, worldY};
  }

  [[nodiscard]] static std::vector<Vector2f> getTargets(const KickRange& kickRange,
                                                        const FieldDimensions& theFieldDimensions)
  {
    const Vector2f& ballPosition = kickRange.ballPosition;

    std::vector<Vector2f> points = {};

    for (size_t i = 1; i < kickRange.targets.size(); ++i)
    {
      const Vector2f &target1 = kickRange.targets.at(i - 1);
      const Vector2f &target2 = kickRange.targets.at(i);

      auto [leftTarget, rightTarget] = KickUtils::getLeftAndRightTarget(
          ballPosition, target1, target2);

      const Geometry::Line leftLine = {ballPosition, (leftTarget - ballPosition).normalized()};
      const Geometry::Line rightLine = {ballPosition, (rightTarget - ballPosition).normalized()};
      const Geometry::Line leftToRightTargetLine = {leftTarget, (rightTarget - leftTarget).normalized()};

      for (int index = 0; index < CELL_COUNT; ++index)
      {
        const Vector2f point = indexToField(index, theFieldDimensions);

        if (Geometry::isPointLeftOfLine(point, leftLine))
        {
          continue;
        }

        if (!Geometry::isPointLeftOfLine(point, rightLine))
        {
          continue;
        }

        points.push_back(point);
      }
    }

    return points;
  }
  ),
  (std::vector<float>)({}) heatVector
);
