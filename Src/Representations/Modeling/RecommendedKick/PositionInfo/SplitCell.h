#pragma once

#include "../../../../Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "../../../../Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "../../../Configuration/FieldDimensions.h"
#include "SplitCellFunctions.h"
#include <algorithm>

STREAMABLE(SplitCell,
PROTECT(

public:

  SplitCell() : heatVector(4, 0.f)
  {}

  void reset()
  {
    heatVector.assign(4, 0.f);
  }

  [[nodiscard]] float getHeat(const Vector2f& worldPosition) const
  {
    const int splitCellIndex = SplitCellFunctions::getSplitCellIndex(worldPosition, center);
    return heatVector.at(splitCellIndex);
  }

  inline void setHeat(const Vector2f& newCenter, const FieldDimensions& theFieldDimensions, const std::function<float (const Vector2f&)>& positionToHeatFunction)
  {
    const std::vector<Vector2f> splitCellPositions = SplitCellFunctions::getPositions(newCenter, theFieldDimensions);
    for (int positionIndex = 0; positionIndex < 4; positionIndex++)
    {
      const float heat = positionToHeatFunction(splitCellPositions.at(positionIndex));
      heatVector.at(positionIndex) = heat;
    }
    centerIndex = HeatMapFunctions::fieldToIndex(newCenter, theFieldDimensions);
    center = newCenter;
  }

  void draw(const bool stretchColors, const FieldDimensions& theFieldDimensions) const
  {
    const std::vector<Vector2f> splitCellDimensions = SplitCellFunctions::getDimensions(center, theFieldDimensions);
    const std::vector<Vector2f> splitCellPositions = SplitCellFunctions::getPositions(center, theFieldDimensions);
    ASSERT(splitCellDimensions.size() == splitCellPositions.size());
    ASSERT(splitCellDimensions.size() == heatVector.size());

    for (int i = 0; i < (int) heatVector.size(); i++)
    {
      const Vector2f& splitCellDimension = splitCellDimensions.at(i);
      const Vector2f& splitCellPosition = splitCellPositions.at(i);

      const Vector2f drawFieldPosition = {splitCellPosition.x() - splitCellDimension.x() / 2, splitCellPosition.y() - splitCellDimension.y() / 2};

      const float heat = heatVector.at(i);

      RECTANGLE2(DRAW_HEAT_MAP,
          Vector2i((int)drawFieldPosition.x(), (int)drawFieldPosition.y()),
          splitCellDimension.x(),
          splitCellDimension.y(),
          0,
          10,
          Drawings::noBrush,
          ColorRGBA(),
          Drawings::solidBrush,
          MathUtils::valueToRGB(heat));
    }
  }

  [[nodiscard]] inline int getCenterIndex() const
  {
    return centerIndex;
  }

  ),

  // always have to match each other
  (int)(-1) centerIndex,
  (Vector2f)({0, 0}) center,

  (std::vector<float>)({}) heatVector
);
