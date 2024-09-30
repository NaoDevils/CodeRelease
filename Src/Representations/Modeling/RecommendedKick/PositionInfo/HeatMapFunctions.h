/**
 * Collection of static functions for the HeatMap class which is a STREAMABLE and therefore difficult to program in
 */

#pragma once

#include "Config.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>

class HeatMapFunctions
{
public:
  static float getStepSizeX(const FieldDimensions& theFieldDimensions)
  {
    const float maxNonNegativeWorld = 2 * theFieldDimensions.xPosOpponentGroundline;
    return maxNonNegativeWorld / Config::CELL_COUNT_X;
  }

  static float getStepSizeY(const FieldDimensions& theFieldDimensions)
  {
    const float maxNonNegativeWorld = 2 * theFieldDimensions.yPosLeftSideline;
    return maxNonNegativeWorld / Config::CELL_COUNT_Y;
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

  [[nodiscard]] static int indexesToIndex(const int indexX, const int indexY) { return indexX * Config::CELL_COUNT_Y + indexY; }

  [[nodiscard]] static std::tuple<int, int> indexToIndexes(const int index)
  {
    const int indexX = index / Config::CELL_COUNT_Y;
    const int indexY = index % Config::CELL_COUNT_Y;
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
    const int indexX = (int)MathUtils::clamp_f(distancePercentX * Config::CELL_COUNT_X, 0, Config::CELL_COUNT_X - 1);
    const int indexY = (int)MathUtils::clamp_f(distancePercentY * Config::CELL_COUNT_Y, 0, Config::CELL_COUNT_Y - 1);
    return indexesToIndex(indexX, indexY);
  }

  [[nodiscard]] static Vector2f indexToField(const int index, const FieldDimensions& theFieldDimensions)
  {
    const float stepSizeX = getStepSizeX(theFieldDimensions);
    const float stepSizeY = getStepSizeY(theFieldDimensions);
    const auto [indexX, indexY] = indexToIndexes(index);
    const float nonNegativeWorldX = stepSizeX * (float)indexX + stepSizeX / 2;
    const float nonNegativeWorldY = stepSizeY * (float)indexY + stepSizeY / 2;
    const float worldX = nonNegativeWorldX - theFieldDimensions.xPosOpponentGroundline;
    const float worldY = nonNegativeWorldY - theFieldDimensions.yPosLeftSideline;
    return {worldX, worldY};
  }

  static void drawArrows(const std::vector<float>& heatVector, const FieldDimensions& theFieldDimensions)
  {
    for (int mainX = 0; mainX < Config::CELL_COUNT_X; mainX++)
    {
      for (int mainY = 0; mainY < Config::CELL_COUNT_Y; mainY++)
      {
        const int mainIndex = HeatMapFunctions::indexesToIndex(mainX, mainY);
        const float mainHeat = heatVector.at(mainIndex);

        std::vector<int> bestCompareIndices = {};
        float bestCompareHeat = mainHeat;
        for (int addX = -1; addX <= 1; addX++)
        {
          for (int addY = -1; addY <= 1; addY++)
          {
            const int compareX = mainX + addX;
            if (compareX < 0 || compareX >= Config::CELL_COUNT_X)
            {
              continue;
            }
            const int compareY = mainY + addY;
            if (compareY < 0 || compareY >= Config::CELL_COUNT_Y)
            {
              continue;
            }
            const int compareIndex = HeatMapFunctions::indexesToIndex(compareX, compareY);
            const float compareHeat = heatVector.at(compareIndex);
            if (compareHeat == bestCompareHeat)
            {
              bestCompareIndices.emplace_back(compareIndex);
            }
            if (compareHeat > bestCompareHeat)
            {
              bestCompareIndices.clear();
              bestCompareIndices.emplace_back(compareIndex);
              bestCompareHeat = compareHeat;
            }
          }
        }

        if (bestCompareIndices.empty())
        {
          continue;
        }

        const Vector2f mainPosition = HeatMapFunctions::indexToField(mainIndex, theFieldDimensions);
        for (const int bestCompareIndex : bestCompareIndices)
        {
          const Vector2f bestComparePosition = HeatMapFunctions::indexToField(bestCompareIndex, theFieldDimensions);
          const Vector2f drawPosition = bestComparePosition - (2.f / 4.f) * (bestComparePosition - mainPosition);
          ARROW(DRAW_HEAT_MAP, mainPosition.x(), mainPosition.y(), drawPosition.x(), drawPosition.y(), 5, Drawings::solidPen, ColorRGBA::black);
        }
      }
    }
  }

  static void draw(const std::vector<Vector2f>& positionVector, std::vector<float> heatVector, const bool stretchColors, const FieldDimensions& theFieldDimensions)
  {
    ASSERT(positionVector.size() == heatVector.size());

    if (stretchColors)
    {
      MathUtils::stretch(heatVector);
    }

    const int stepSizeX = (int)getStepSizeX(theFieldDimensions);
    const int stepSizeY = (int)getStepSizeY(theFieldDimensions);
    const int offsetX = stepSizeX / 2;
    const int offsetY = stepSizeY / 2;
    for (int i = 0; i < (int)positionVector.size(); i++)
    {
      const Vector2f& position = positionVector.at(i);
      const Vector2f drawFieldPosition = {(int)position.x() - offsetX, (int)position.y() - offsetY};
      const float heat = heatVector.at(i);

      RECTANGLE2(DRAW_HEAT_MAP, Vector2i((int)drawFieldPosition.x(), (int)drawFieldPosition.y()), stepSizeX, stepSizeY, 0, 10, Drawings::noBrush, ColorRGBA(), Drawings::solidBrush, MathUtils::valueToRGB(heat));
    }
  }
};
