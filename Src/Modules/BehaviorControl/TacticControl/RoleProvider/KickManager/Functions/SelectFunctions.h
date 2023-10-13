#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/DrawFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/ScoreFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/CurrentKick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/KickPlan.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/SelectablePose.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/SelectableTarget.h"
#include "Representations/Modeling/KickWheel.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class SelectFunctions
{

public:
  static std::optional<KickPlan> createAndFilterAndSelect(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const std::vector<Kick*>& kicks,
      const std::optional<CurrentKick>& currentKick,
      const Filterer& filterer,
      const Factors& factors,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const KickWheel& theKickWheel,
      const RobotMap& theRobotMap,
      const TacticSymbols& theTacticSymbols);

  static std::optional<KickPlan> filterAndSelect(std::vector<KickPlan>& kickPlans,
      const Filterer& filterer,
      const Factors& factors,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap,
      const TacticSymbols& theTacticSymbols,
      const KickWheel& theKickWheel);

private:
  static std::vector<SelectableDirection> getSelectableDirections(const Vector2f& ballPosition, const Filterer& filterer, const KickWheel& theKickWheel);

  static void addSelectableTargets(std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, Kick* kick, const Filterer& filterer);
  static void addSelectableTargetsFromStaticKick(
      std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer);
  static void addSelectableTargetsFromAdjustableKick(std::vector<SelectableTarget>& selectableTargets, const SelectableDirection& selectableDirection, Kick* kick, const Filterer& filterer);
  static void addSelectableTargetsForDistance(
      std::vector<SelectableTarget>& selectableTargets, const float targetDistance, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer);

  static std::optional<SelectablePose> getBestSelectablePose(
      const Pose2f& playerPose, const SelectableTarget& selectableTarget, const Filterer& filterer, const Factors& factors, const TacticSymbols& theTacticSymbols);
};
