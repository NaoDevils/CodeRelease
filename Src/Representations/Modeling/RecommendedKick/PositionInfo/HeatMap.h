#pragma once

#include "Config.h"
#include "HeatMapFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Representations/Configuration/FieldDimensions.h"
#include <algorithm>

/**
 * There is no method to get heat in an area because it is more efficient to use a smooth distribution of heat on the field instead
 */
STREAMABLE(HeatMap,
  PROTECT(

public:

  HeatMap() : heatVector(Config::CELL_COUNT, 0.f)
  {}

  void reset()
  {
    heatVector.assign(Config::CELL_COUNT, 0.f);
  }

  [[nodiscard]] float getHeat(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const
  {
    const int index = HeatMapFunctions::fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions);
    return getHeat(index);
  }

  [[nodiscard]] inline float getHeat(const int index) const
  {
    return heatVector[index];
  }

  void updateHeat(const float takeNewPercent, float newHeat, const int index, const FieldDimensions& theFieldDimensions)
  {
    const float oldHeat = getHeat(index);
    setHeat(calculateUpdatedHeat(takeNewPercent, oldHeat, newHeat), index, theFieldDimensions);
  }

  static float calculateUpdatedHeat(const float takeNewPercent, const float oldHeat, const float newHeat)
  {
    ASSERT(takeNewPercent > 0.f && takeNewPercent <= 1.f);
    return oldHeat * (1.f - takeNewPercent) + newHeat * takeNewPercent;
  }

  inline void setHeat(const float heat, const int index, const FieldDimensions& theFieldDimensions)
  {
    ASSERT(heat >= 0.f && heat <= 1.f);
    heatVector[index] = heat;
  }

  void draw(const bool stretchColors, const FieldDimensions& theFieldDimensions) const
  {
    std::vector<Vector2f> fieldPositionVector;
    std::vector<float> fieldPositionHeatVector;
    for (int i = 0; i < Config::CELL_COUNT; i++)
    {
      const Vector2f fieldPosition = HeatMapFunctions::indexToField(i, theFieldDimensions);
      fieldPositionVector.push_back(fieldPosition);
      const float heat = getHeat(i);
      fieldPositionHeatVector.push_back(heat);
    }
    HeatMapFunctions::draw(fieldPositionVector, fieldPositionHeatVector, stretchColors, theFieldDimensions);
  }

  ),

  (std::vector<float>)({}) heatVector
);
