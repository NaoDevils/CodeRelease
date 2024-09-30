#pragma once

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Ranges/KickRange.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableDirection.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableKick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectablePose.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableTarget.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class Filterer
{

public:
  [[nodiscard]] std::vector<std::function<bool(SelectableKick&)>> getEfficientlyOrderedSelectableKickFilters() const;
  [[nodiscard]] std::vector<std::function<bool(SelectableDirection&)>> getEfficientlyOrderedSelectableDirectionFilters() const;
  [[nodiscard]] std::vector<std::function<bool(SelectableTarget&)>> getEfficientlyOrderedSelectableTargetFilters() const;
  [[nodiscard]] std::vector<std::function<bool(SelectablePose&)>> getEfficientlyOrderedSelectablePoseFilters() const;

  [[nodiscard]] bool isFilterBlocked() const { return blockedFilter; }

  [[nodiscard]] bool isFilterOutside() const { return outsideFilter; }

  Filterer& filterBlocked()
  {
    blockedFilter = true;
    return *this;
  }

  Filterer& filterOutside()
  {
    outsideFilter = true;
    return *this;
  }

  Filterer& filterOutsideKickRange(const KickRange& kickRange);
  Filterer& filterBetweenAngles(const Vector2f& ballPosition, Angle leftAngle, Angle rightAngle);
  Filterer& filterToOwnGoal(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions);

  Filterer& filterTooHighRotationToKick(Angle currentRotation, Angle tooHighRotation);
  Filterer& filterInFrontOfOwnGoal(const FieldDimensions& selectableTarget);
  Filterer& filterTooFarBack(float minX);
  Filterer& filterLeft(float maxY);
  Filterer& filterRight(float minY);
  Filterer& filterLeftOfLine(const Vector2f& linePoint1, const Vector2f& linePoint2);
  Filterer& filterCloseToFieldLine(const FieldDimensions& theFieldDimensions);

  Filterer& filterKickPoseBlockedByRobots(const RobotMap& theRobotMap);
  Filterer& filterKickPoseBlockedByGoalPost(const FieldDimensions& theFieldDimensions);

private:
  bool blockedFilter = false;
  bool outsideFilter = false;

  std::function<bool(SelectableDirection&)> outsideKickRangeDirectionFilter;
  std::function<bool(SelectableDirection&)> betweenAnglesFilter;
  std::function<bool(SelectableDirection&)> toOwnGoalFilter;

  std::function<bool(SelectableTarget&)> outsideKickRangeTargetFilter;
  std::function<bool(SelectableTarget&)> tooHighRotationToKickFilter;
  std::function<bool(SelectableTarget&)> inFrontOfOwnGoalFilter;
  std::function<bool(SelectableTarget&)> tooFarBackFilter;
  std::function<bool(SelectableTarget&)> leftFilter;
  std::function<bool(SelectableTarget&)> rightFilter;
  std::function<bool(SelectableTarget&)> leftOfLineFilter;
  std::function<bool(SelectableTarget&)> closeToFieldLineFilter;

  std::function<bool(SelectablePose&)> poseBlockedByRobotFilter;
  std::function<bool(SelectablePose&)> poseBlockedByGoalPostFilter;
};
