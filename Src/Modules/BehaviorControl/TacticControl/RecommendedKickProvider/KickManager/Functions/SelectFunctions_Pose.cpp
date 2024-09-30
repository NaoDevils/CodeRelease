#include "SelectFunctions.h"

std::vector<SelectablePose> SelectFunctions::createSelectablePoses(const SelectableShot& selectableShot, const Filterer& filterer, const ShotParameters& factors, const TacticSymbols& theTacticSymbols)
{
  const Pose2f pose1 = selectableShot.selectableTarget.selectableKick.kick->getKickPose(
      selectableShot.selectableTarget.selectableDirection.ballPosition, selectableShot.selectableTarget.selectableDirection.angle, true);
  const Pose2f pose2 = selectableShot.selectableTarget.selectableKick.kick->getKickPose(
      selectableShot.selectableTarget.selectableDirection.ballPosition, selectableShot.selectableTarget.selectableDirection.angle, false);
  SelectablePose selectablePose1 = {selectableShot, pose1, true};
  SelectablePose selectablePose2 = {selectableShot, pose2, false};
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

  std::vector<SelectablePose> selectablePoses = {};
  if (pose1Valid && pose2Valid)
  {
    selectablePoses.push_back(selectablePose1);
    selectablePoses.push_back(selectablePose2);
  }
  else if (pose1Valid)
  {
    selectablePoses.push_back(selectablePose1);
  }
  else if (pose2Valid)
  {
    selectablePoses.push_back(selectablePose2);
  }
  return selectablePoses;
}