#pragma once

#include "DrawFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/CurrentKick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/ShotParameters.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Filterer/Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/ExecutableShot.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectablePose.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableTarget.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "ScoreFunctions.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class SelectFunctions
{

public:
  static std::optional<ExecutableShot> createAndFilterAndSelect(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const std::vector<Kick*>& kicks,
      const std::optional<CurrentKick>& currentKick,
      const Filterer& filterer,
      const ShotParameters& factors,
      const DirectionInfo& theDirectionInfo,
      const FieldDimensions& theFieldDimensions,
      const PositionInfo& thePositionInfo,
      const TacticSymbols& theTacticSymbols);

private:
  static std::vector<SelectableDirection> createSelectableDirections(const Vector2f& ballPosition, const Filterer& filterer, const DirectionInfo& theDirectionInfo);
  static std::optional<SelectableDirection> createSelectableDirection(int index, const Vector2f& ballPosition, const Filterer& filterer, const DirectionInfo& theDirectionInfo);

  static std::optional<SelectableKick> createSelectableKick(Kick* kick, const Filterer& filterer);

  static std::vector<float> createMinToMaxTargetedDistances(const SelectableKick& selectableKick);
  static SelectableTarget createAndFilterSelectableTarget(float targetedDistance, const SelectableDirection& selectableDirection, const SelectableKick& selectableKick, const Filterer& filterer);

  static std::vector<SelectableShot> createSelectableShots(std::vector<SelectableTarget>& selectableTargets);

  static std::vector<SelectablePose> createSelectablePoses(const SelectableShot& selectableShot, const Filterer& filterer, const ShotParameters& factors, const TacticSymbols& theTacticSymbols);

  static std::optional<ExecutableShot> createExecutableShotFromCurrentKick(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const std::vector<SelectableShot>& selectableShots,
      const CurrentKick& currentKick,
      const Filterer& filterer,
      const ShotParameters& factors,
      const DirectionInfo& theDirectionInfo,
      const FieldDimensions& theFieldDimensions,
      const PositionInfo& thePositionInfo,
      const TacticSymbols& theTacticSymbols);
};
