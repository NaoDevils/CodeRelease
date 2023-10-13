#include "SelectFunctions.h"

std::vector<SelectableDirection> SelectFunctions::getSelectableDirections(const Vector2f& ballPosition, const Filterer& filterer, const KickWheel& theKickWheel)
{
  std::vector<SelectableDirection> selectableDirections = {};
  const int max_i = (int)theKickWheel.blockedDistances.size();
  for (int i = 0; i < max_i; i++)
  {
    float distance = std::numeric_limits<float>::max();

    bool distanceBlocked = false;
    bool distanceOutside = false;

    const float blockedDistance = theKickWheel.blockedDistances.at(i);
    if (blockedDistance < distance)
    {
      if (filterer.isFilterBlocked())
      {
        continue;
      }
      distance = blockedDistance;
      distanceBlocked = true;
    }

    const float outsideDistance = theKickWheel.outsideDistances.at(i);
    if (outsideDistance < distance)
    {
      distanceBlocked = false;
      if (filterer.isFilterOutside())
      {
        distance = outsideDistance;
      }
      distanceOutside = true;
    }

    SelectableDirection selectableDirection = {ballPosition, theKickWheel.angles.at(i), distance, distanceBlocked, distanceOutside};

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
      continue;
    }

    selectableDirections.push_back(selectableDirection);
  };
  return selectableDirections;
}