/**
 * Collection of static functions for the SplitCell class which is a STREAMABLE and therefore difficult to program in
 */

#pragma once

#include "Config.h"
#include "HeatMap.h"
#include "HeatMapFunctions.h"
#include "../../../../Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "../../../../Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "../../../Configuration/FieldDimensions.h"
#include <algorithm>

class SplitCellFunctions
{
public:
  static std::vector<Vector2f> getDimensions(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    const auto [cell_front_x, cell_back_x, cell_left_y, cell_right_y] = getBorders(ballPosition, theFieldDimensions);

    const float ball_x = ballPosition.x();
    const float ball_y = ballPosition.y();

    const float front = cell_front_x - ball_x;
    const float back = ball_x - cell_back_x;
    const float left = cell_left_y - ball_y;
    const float right = ball_y - cell_right_y;

    const Vector2f front_left_dimensions = {front, left};
    const Vector2f front_right_dimensions = {front, right};
    const Vector2f back_left_dimensions = {back, left};
    const Vector2f back_right_dimensions = {back, right};

    std::vector<Vector2f> splitCellDimensions = {};
    splitCellDimensions.push_back(front_left_dimensions);
    splitCellDimensions.push_back(front_right_dimensions);
    splitCellDimensions.push_back(back_left_dimensions);
    splitCellDimensions.push_back(back_right_dimensions);
    return splitCellDimensions;
  }

  static std::tuple<float, float, float, float> getBorders(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    const float stepSizeX = HeatMapFunctions::getStepSizeX(theFieldDimensions);
    const float stepSizeY = HeatMapFunctions::getStepSizeY(theFieldDimensions);
    const float halfStepSizeX = stepSizeX / 2;
    const float halfStepSizeY = stepSizeY / 2;
    const int cellIndex = HeatMapFunctions::fieldToIndex(ballPosition, theFieldDimensions);
    const Vector2f cellPosition = HeatMapFunctions::indexToField(cellIndex, theFieldDimensions);
    const float cell_center_x = cellPosition.x();
    const float cell_center_y = cellPosition.y();
    const float cell_front_x = cell_center_x + halfStepSizeX;
    const float cell_back_x = cell_center_x - halfStepSizeX;
    const float cell_left_y = cell_center_y + halfStepSizeY;
    const float cell_right_y = cell_center_y - halfStepSizeY;
    return {cell_front_x, cell_back_x, cell_left_y, cell_right_y};
  }

  static std::vector<Vector2f> getPositions(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    const auto [cell_front_x, cell_back_x, cell_left_y, cell_right_y] = getBorders(ballPosition, theFieldDimensions);

    const float ball_x = ballPosition.x();
    const float ball_y = ballPosition.y();

    const float front = ball_x + (cell_front_x - ball_x) / 2;
    const float back = ball_x - (ball_x - cell_back_x) / 2;
    const float left = ball_y + (cell_left_y - ball_y) / 2;
    const float right = ball_y - (ball_y - cell_right_y) / 2;

    std::vector<Vector2f> splitCellPositions = {};
    splitCellPositions.emplace_back(front, left);
    splitCellPositions.emplace_back(front, right);
    splitCellPositions.emplace_back(back, left);
    splitCellPositions.emplace_back(back, right);
    return splitCellPositions;
  }

  static int getSplitCellIndex(const Vector2f& worldPosition, const Vector2f& ballPosition)
  {
    const float ball_x = ballPosition.x();
    const float ball_y = ballPosition.y();
    const float world_x = worldPosition.x();
    const float world_y = worldPosition.y();

    if (world_x > ball_x)
    {
      if (world_y > ball_y)
      {
        return 0; // front left
      }
      else
      {
        return 1; // front right
      }
    }
    else
    {
      if (world_y > ball_y)
      {
        return 2; // back left
      }
      else
      {
        return 3; // back right
      }
    }
  }
};
