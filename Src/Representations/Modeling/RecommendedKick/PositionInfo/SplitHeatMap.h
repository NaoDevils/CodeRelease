#pragma once

#include "Config.h"
#include "HeatMap.h"
#include "HeatMapFunctions.h"
#include "SplitCell.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SplitHeatMap,
  [[nodiscard]] float getHeat(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const;
  [[nodiscard]] const SplitCell* getUseSplitCell(const int index) const;
  [[nodiscard]] float getHeat(const int index, const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const;
  void setCellHeat(const float heat, const int index, const FieldDimensions& theFieldDimensions);
  void setSplitCellHeat(const std::vector<Pose2f>& newCenters, const FieldDimensions& theFieldDimensions, const std::function<std::function<float (const Vector2f&)> (const Pose2f&)>& getPositionToHeatFunction);
  void draw(const bool stretchColors, const FieldDimensions& theFieldDimensions) const;
  ,
  (std::vector<SplitCell>) splitCells,
  (HeatMap) heatMap
);


inline float SplitHeatMap::getHeat(const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const
{
  const int index = HeatMapFunctions::fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions);
  return getHeat(index, worldPosition, theFieldDimensions);
}

inline const SplitCell* SplitHeatMap::getUseSplitCell(const int index) const
{
  const SplitCell* useSplitCell = nullptr;
  for (const SplitCell& splitCell : splitCells)
  {
    if (index == splitCell.getCenterIndex())
    {
      if (useSplitCell == nullptr)
      {
        useSplitCell = &splitCell;
      }
      else
      {
        // if there are multiple split cells don't use any of them
        useSplitCell = nullptr;
        break;
      }
    }
  }
  return useSplitCell;
}

inline float SplitHeatMap::getHeat(const int index, const Vector2f& worldPosition, const FieldDimensions& theFieldDimensions) const
{
  ASSERT(index == HeatMapFunctions::fieldToIndex(worldPosition.x(), worldPosition.y(), theFieldDimensions));

  const SplitCell* useSplitCell = getUseSplitCell(index);
  if (useSplitCell == nullptr)
  {
    return heatMap.getHeat(index);
  }
  return useSplitCell->getHeat(worldPosition);
}

inline void SplitHeatMap::setCellHeat(const float heat, const int index, const FieldDimensions& theFieldDimensions)
{
  heatMap.setHeat(heat, index, theFieldDimensions);
}

inline void SplitHeatMap::setSplitCellHeat(
    const std::vector<Pose2f>& newCenters, const FieldDimensions& theFieldDimensions, const std::function<std::function<float(const Vector2f&)>(const Pose2f&)>& getPositionToHeatFunction)
{
  splitCells.resize(newCenters.size());
  for (int i = 0; i < (int)newCenters.size(); i++)
  {
    const auto positionToHeatFunction = getPositionToHeatFunction(newCenters.at(i));
    splitCells.at(i).setHeat(newCenters.at(i).translation, theFieldDimensions, positionToHeatFunction);
  }
}

inline void SplitHeatMap::draw(const bool stretchColors, const FieldDimensions& theFieldDimensions) const
{
  std::vector<int> splitCellCenterIndexes = {};
  for (const auto& splitCell : splitCells)
  {
    splitCellCenterIndexes.push_back(splitCell.getCenterIndex());
  }

  std::vector<const SplitCell*> drawSplitCells = {};
  drawSplitCells.reserve(splitCells.size());

  std::vector<Vector2f> fieldPositionVector;
  std::vector<float> heatVector;
  for (int i = 0; i < Config::CELL_COUNT; i++)
  {
    if (std::find(splitCellCenterIndexes.begin(), splitCellCenterIndexes.end(), i) != splitCellCenterIndexes.end()) // contains
    {
      const SplitCell* drawSplitCell = getUseSplitCell(i);
      if (drawSplitCell != nullptr)
      {
        drawSplitCells.push_back(drawSplitCell);
      }
    }
    const Vector2f fieldPosition = HeatMapFunctions::indexToField(i, theFieldDimensions);
    fieldPositionVector.push_back(fieldPosition);
    const float heat = heatMap.getHeat(i);
    heatVector.push_back(heat);
  }
  HeatMapFunctions::draw(fieldPositionVector, heatVector, stretchColors, theFieldDimensions);

  for (const auto& splitCell : drawSplitCells)
  {
    splitCell->draw(stretchColors, theFieldDimensions);
  }
}
