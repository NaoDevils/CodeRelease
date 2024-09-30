#include "SelectFunctions.h"

std::vector<SelectableDirection> SelectFunctions::createSelectableDirections(const Vector2f& ballPosition, const Filterer& filterer, const DirectionInfo& theDirectionInfo)
{
  std::vector<SelectableDirection> selectableDirections = {};
  const int max_i = (int)theDirectionInfo.blockedDistances.size();
  for (int i = 0; i < max_i; i++)
  {
    std::optional<SelectableDirection> selectableDirectionOptional = createSelectableDirection(i, ballPosition, filterer, theDirectionInfo);
    if (selectableDirectionOptional.has_value())
    {
      selectableDirections.push_back(selectableDirectionOptional.value());
    }
  }
  return selectableDirections;
}

std::optional<SelectableDirection> SelectFunctions::createSelectableDirection(const int index, const Vector2f& ballPosition, const Filterer& filterer, const DirectionInfo& theDirectionInfo)
{
  SelectableDirection selectableDirection = {ballPosition,
      theDirectionInfo.angles.at(index),
      theDirectionInfo.blockedDistances.at(index),
      theDirectionInfo.intoGoalKickOutsideDistances.at(index),
      theDirectionInfo.intoKickInOutsideDistances.at(index),
      theDirectionInfo.intoCornerKickOutsideDistances.at(index),
      theDirectionInfo.intoOpponentGoalDistances.at(index),
      theDirectionInfo.intoOwnGoalDistances.at(index)};

  bool remove = false;
  for (const auto& filter : filterer.getEfficientlyOrderedSelectableDirectionFilters())
  {
    if ((filter)(selectableDirection))
    {
      remove = true;
      break;
    }
  }
  if (remove)
  {
    return {};
  }

  return selectableDirection;
}