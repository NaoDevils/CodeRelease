#include "Filterer.h"

[[nodiscard]] std::vector<std::function<bool(SelectableKick&)>> Filterer::getEfficientlyOrderedSelectableKickFilters() const
{
  std::vector<std::function<bool(SelectableKick&)>> efficientlyOrderedFilters = {};
  return efficientlyOrderedFilters;
}

[[nodiscard]] std::vector<std::function<bool(SelectableDirection&)>> Filterer::getEfficientlyOrderedSelectableDirectionFilters() const
{
  std::vector<std::function<bool(SelectableDirection&)>> efficientlyOrderedFilters = {};

  if (betweenAnglesFilter)
  {
    efficientlyOrderedFilters.push_back(betweenAnglesFilter);
  }
  if (outsideKickRangeDirectionFilter)
  {
    efficientlyOrderedFilters.push_back(outsideKickRangeDirectionFilter);
  }
  if (toOwnGoalFilter)
  {
    efficientlyOrderedFilters.push_back(toOwnGoalFilter);
  }

  return efficientlyOrderedFilters;
}

[[nodiscard]] std::vector<std::function<bool(SelectableTarget&)>> Filterer::getEfficientlyOrderedSelectableTargetFilters() const
{
  std::vector<std::function<bool(SelectableTarget&)>> efficientlyOrderedFilters = {};

  if (tooFarBackFilter)
  {
    efficientlyOrderedFilters.push_back(tooFarBackFilter);
  }
  if (leftFilter)
  {
    efficientlyOrderedFilters.push_back(leftFilter);
  }
  if (rightFilter)
  {
    efficientlyOrderedFilters.push_back(rightFilter);
  }
  if (outsideKickRangeTargetFilter)
  {
    efficientlyOrderedFilters.push_back(outsideKickRangeTargetFilter);
  }
  if (tooHighRotationToKickFilter)
  {
    efficientlyOrderedFilters.push_back(tooHighRotationToKickFilter);
  }
  if (leftOfLineFilter)
  {
    efficientlyOrderedFilters.push_back(leftOfLineFilter);
  }
  if (closeToFieldLineFilter)
  {
    efficientlyOrderedFilters.push_back(closeToFieldLineFilter);
  }
  if (inFrontOfOwnGoalFilter)
  {
    efficientlyOrderedFilters.push_back(inFrontOfOwnGoalFilter);
  }

  return efficientlyOrderedFilters;
}

[[nodiscard]] std::vector<std::function<bool(SelectablePose&)>> Filterer::getEfficientlyOrderedSelectablePoseFilters() const
{
  std::vector<std::function<bool(SelectablePose&)>> efficientlyOrderedFilters = {};
  if (poseBlockedByRobotFilter)
  {
    efficientlyOrderedFilters.push_back(poseBlockedByRobotFilter);
  }
  if (poseBlockedByGoalPostFilter)
  {
    efficientlyOrderedFilters.push_back(poseBlockedByGoalPostFilter);
  }
  return efficientlyOrderedFilters;
}
