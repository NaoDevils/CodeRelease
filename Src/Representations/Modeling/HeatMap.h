#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <algorithm>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/KickRange.h>

/**
 * There is no method to get heat in an area because it is more efficient to let the heat distribution on the field not have hard corners
 */
STREAMABLE(HeatMap,
  PROTECT(
public:
  static constexpr int CELL_COUNT_X = 45;
  static constexpr int CELL_COUNT_Y = 30;
  static constexpr int CELL_COUNT = CELL_COUNT_X * CELL_COUNT_Y;

  HeatMap() : heatVector(CELL_COUNT, 0.f)
  {}

  void reset()
  {
    heatVector.assign(CELL_COUNT, 0.f);
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

  static bool isInCell(const Vector2f& position, const Vector2f& cellPosition, const float cellHalfStepSizeX, const float cellHalfStepSizeY)
  {
    const float xDiff = std::abs(position.x() - cellPosition.x());
    const float yDiff = std::abs(position.y() - cellPosition.y());
    return xDiff < cellHalfStepSizeX + 1 && yDiff < cellHalfStepSizeY + 1;
  }

  [[nodiscard]] static int fieldToIndex(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions)
  {
    return fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions);
  }

   [[nodiscard]] static int indexesToIndex(const int indexX, const int indexY)
   {
     return indexX * CELL_COUNT_Y + indexY;
   }

   [[nodiscard]] static std::tuple<int, int> indexToIndexes(const int index)
   {
     const int indexX = index / CELL_COUNT_Y;
     const int indexY = index % CELL_COUNT_Y;
     return {indexX, indexY};
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
    return indexesToIndex(indexX, indexY);
  }

   [[nodiscard]] static Vector2f indexToField(const int index, const FieldDimensions& theFieldDimensions)
   {
     const float stepSizeX = getStepSizeX(theFieldDimensions);
     const float stepSizeY = getStepSizeY(theFieldDimensions);
     const auto [indexX, indexY] = indexToIndexes(index);
     const float nonNegativeWorldX = stepSizeX * (float) indexX + stepSizeX / 2;
     const float nonNegativeWorldY = stepSizeY * (float) indexY + stepSizeY / 2;
     const float worldX = nonNegativeWorldX - theFieldDimensions.xPosOpponentGroundline;
     const float worldY = nonNegativeWorldY - theFieldDimensions.yPosLeftSideline;
     return {worldX, worldY};
   }
  ),
  (std::vector<float>)({}) heatVector
);
