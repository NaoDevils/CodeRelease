#include "SelectFunctions.h"

std::optional<SelectableKick> SelectFunctions::createSelectableKick(Kick* kick, const Filterer& filterer)
{
  SelectableKick selectableKick = SelectableKick(kick);
  for (const auto& filter : filterer.getEfficientlyOrderedSelectableKickFilters())
  {
    if (filter(selectableKick))
    {
      return {};
    }
  }
  return selectableKick;
}

std::vector<SelectableShot> SelectFunctions::createSelectableShots(std::vector<SelectableTarget>& selectableTargets)
{
  std::vector<SelectableShot> selectableShots = {};
  selectableShots.reserve(selectableTargets.size());

  SelectableShot* firstSelectableShot = nullptr;
  SelectableShot* lastSelectableShot = nullptr;
  for (SelectableTarget& selectableTarget : selectableTargets)
  {
    selectableShots.emplace_back(selectableTarget);
    SelectableShot* selectableShot = &selectableShots.back();
    if (firstSelectableShot == nullptr)
    {
      firstSelectableShot = selectableShot;
    }
    else
    {
      lastSelectableShot->rightSelectableShot = selectableShot;
      selectableShot->leftSelectableShot = lastSelectableShot;
    }
    lastSelectableShot = selectableShot;
  }
  lastSelectableShot->rightSelectableShot = firstSelectableShot;
  firstSelectableShot->leftSelectableShot = lastSelectableShot;
  return selectableShots;
}

std::vector<float> SelectFunctions::createMinToMaxTargetedDistances(const SelectableKick& selectableKick)
{
  std::vector<float> targetedDistances = {};
  if (selectableKick.kick->isDistanceAdjustable())
  {
    const int DISTANCE_STEPS_COUNT = 5;

    const float minDistance = selectableKick.kick->minDistance;
    const float maxDistance = selectableKick.kick->maxDistance;

    const float distanceStepSize = (maxDistance - minDistance) / DISTANCE_STEPS_COUNT;

    for (int step = 0; step < DISTANCE_STEPS_COUNT; step++)
    {
      targetedDistances.push_back(minDistance + (float)step * distanceStepSize);
    }
  }
  else
  {
    targetedDistances.push_back(selectableKick.kick->minDistance);
  }
  return targetedDistances;
}

SelectableTarget SelectFunctions::createAndFilterSelectableTarget(
    const float targetedDistance, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer)
{
  SelectableTarget selectableTarget = {selectableKick, selectableDirection, targetedDistance};
  for (const auto& filter : filterer.getEfficientlyOrderedSelectableTargetFilters())
  {
    if ((filter)(selectableTarget))
    {
      selectableTarget.setFiltered(true);
      break;
    }
  }
  return selectableTarget;
}