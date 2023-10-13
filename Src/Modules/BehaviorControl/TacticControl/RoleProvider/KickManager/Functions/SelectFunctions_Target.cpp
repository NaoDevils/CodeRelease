#include "SelectFunctions.h"

void SelectFunctions::addSelectableTargets(std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, Kick* kick, const Filterer& filterer)
{
  if (kick->isDistanceAdjustable())
  {
    addSelectableTargetsFromAdjustableKick(selectableTargets, selectableDirection, kick, filterer);
  }
  else
  {
    addSelectableTargetsFromStaticKick(selectableTargets, selectableDirection, kick, filterer);
  }
}

void SelectFunctions::addSelectableTargetsFromAdjustableKick(std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, Kick* kick, const Filterer& filterer)
{
  const int DISTANCE_STEPS_COUNT = 5;

  float maxDistance;
  if (selectableDirection.distanceOutside)
  {
    const float MIN_DISTANCE_TO_BORDER = 200.f;
    maxDistance = selectableDirection.distance - MIN_DISTANCE_TO_BORDER;
  }
  else
  {
    maxDistance = std::min(selectableDirection.distance, kick->getMaxDistance(false, true));
  }
  const float distanceStepSize = maxDistance / DISTANCE_STEPS_COUNT;
  for (float targetDistance = kick->getMinDistance(false, false); targetDistance < maxDistance; targetDistance = targetDistance + distanceStepSize)
  {
    addSelectableTargetsForDistance(selectableTargets, targetDistance, selectableDirection, kick, filterer);
  }
}

void SelectFunctions::addSelectableTargetsFromStaticKick(
    std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer)
{
  float targetDistance;
  if (selectableDirection.distanceOutside)
  {
    const float minDistance = selectableKick.kick->getMinDistance(false, false);
    const float maxDistance = selectableKick.kick->getMaxDistance(false, false);
    const float probablyNotTooFarDistance = maxDistance + (maxDistance - minDistance) / 2; // TODO Hysteresis
    if (selectableDirection.distance < probablyNotTooFarDistance)
    {
      return;
    }
    targetDistance = selectableKick.kick->getRealisticDistance();
  }
  else
  {
    if (!selectableDirection.distanceBlocked && selectableDirection.distance < selectableKick.kick->getMinDistance(false, false)) // TODO Hysteresis
    {
      return;
    }
    targetDistance = std::min(selectableDirection.distance, selectableKick.kick->getRealisticDistance());
  }

  addSelectableTargetsForDistance(selectableTargets, targetDistance, selectableDirection, selectableKick, filterer);
}

void SelectFunctions::addSelectableTargetsForDistance(
    std::vector<SelectableTarget>& selectableTargets, const float targetDistance, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer)
{
  Vector2f targetPosition = selectableDirection.ballPosition + targetDistance * selectableDirection.direction;
  SelectableTarget selectableTarget = {selectableKick, selectableDirection.ballPosition, targetPosition};
  bool remove = false;
  for (const auto& filter : filterer.getEfficientlyOrderedSelectableTargetFilters())
  {
    if ((filter)(selectableTarget))
    {
      remove = true;
      break;
    }
  }
  if (remove)
  {
    return;
  }
  selectableTargets.push_back(selectableTarget);
}