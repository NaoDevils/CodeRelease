#include "SelectFunctions.h"

std::optional<SelectablePose> SelectFunctions::getBestSelectablePose(
    const Pose2f& playerPose, const SelectableTarget& selectableTarget, const Filterer& filterer, const Factors& factors, const TacticSymbols& theTacticSymbols)
{
  const Pose2f pose1 = selectableTarget.selectableKick.kick->getKickPose(selectableTarget.ballPosition, selectableTarget.target, true);
  const Pose2f pose2 = selectableTarget.selectableKick.kick->getKickPose(selectableTarget.ballPosition, selectableTarget.target, false);
  SelectablePose selectablePose1 = {playerPose, selectableTarget, pose1, true};
  SelectablePose selectablePose2 = {playerPose, selectableTarget, pose2, false};
  bool pose1Valid = true;
  bool pose2Valid = true;
  for (const auto& filter : filterer.getEfficientlyOrderedSelectablePoseFilters())
  {
    if (pose1Valid && filter(selectablePose1))
    {
      pose1Valid = false;
    }
    if (pose2Valid && filter(selectablePose2))
    {
      pose2Valid = false;
    }
  }

  if (pose1Valid && pose2Valid)
  {
    selectablePose1.score = ScoreFunctions::scorePose(selectablePose1, factors.dontRuntIntoFactor, factors.blockDefensiveCone, factors.blockOpponentFactor, factors.timeFactor, theTacticSymbols);
    selectablePose2.score = ScoreFunctions::scorePose(selectablePose2, factors.dontRuntIntoFactor, factors.blockDefensiveCone, factors.blockOpponentFactor, factors.timeFactor, theTacticSymbols);
    if (selectablePose1.score > selectablePose2.score)
    {
      return selectablePose1;
    }
    else
    {
      return selectablePose2;
    }
  }
  if (pose1Valid)
  {
    selectablePose1.score = ScoreFunctions::scorePose(selectablePose1, factors.dontRuntIntoFactor, factors.blockDefensiveCone, factors.blockOpponentFactor, factors.timeFactor, theTacticSymbols);
    return selectablePose1;
  }
  if (pose2Valid)
  {
    selectablePose2.score = ScoreFunctions::scorePose(selectablePose2, factors.dontRuntIntoFactor, factors.blockDefensiveCone, factors.blockOpponentFactor, factors.timeFactor, theTacticSymbols);
    return selectablePose2;
  }
  return {};
}