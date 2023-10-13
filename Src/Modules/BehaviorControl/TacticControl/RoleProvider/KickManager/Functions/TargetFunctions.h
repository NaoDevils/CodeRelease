#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickWithLeftUtils.h"
#include "SelectFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Math/Eigen.h"
#include <stdexcept>
#include <utility>

class TargetFunctions
{

public:
  static std::optional<KickPlan> getKickPlanForTarget(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const Vector2f& targetPosition,
      const DistanceRequirement distanceRequirement,
      const std::vector<Kick*>& kicks,
      const std::optional<CurrentKick>& currentKick,
      const Filterer& filterer,
      const Factors& factors,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap,
      const TacticSymbols& theTacticSymbols,
      const KickWheel& theKickWheel)
  {
    const float distance = Geometry::distance(ballPosition, targetPosition);

    std::vector<KickPlan> kickPlans = {};

    for (Kick* kick : kicks)
    {
      if (!KickUtils::fulfillsDistanceRequirements(*kick, distance, distanceRequirement, false))
      {
        continue;
      }

      SelectableKick selectableKick = {kick};

      Vector2f kickTargetPosition = kick->getRealisticTarget(ballPosition, targetPosition);
      SelectableTarget selectableTarget = {selectableKick, ballPosition, targetPosition};

      Pose2f kickPose1 = kick->getKickPose(ballPosition, kickTargetPosition, true);
      Pose2f kickPose2 = kick->getKickPose(ballPosition, kickTargetPosition, false);
      SelectablePose selectablePose1 = {playerPose, selectableTarget, kickPose1, true};
      SelectablePose selectablePose2 = {playerPose, selectableTarget, kickPose2, false};
      kickPlans.emplace_back(selectablePose1, false);
      kickPlans.emplace_back(selectablePose2, false);
    }

    if (currentKick.has_value())
    {
      const float oldToNewTargetDistance = Geometry::distance(targetPosition, currentKick->target);
      const bool isSameTarget = oldToNewTargetDistance > 300.f;
      if (isSameTarget && KickUtils::fulfillsDistanceRequirements(*currentKick->kick, distance, distanceRequirement, true))
      {
        kickPlans.emplace_back(currentKick.value().toKickPlan(playerPose, ballPosition));
      }
    }

    return SelectFunctions::filterAndSelect(kickPlans, filterer, factors, theFieldDimensions, theHeatMapCollection, theRobotMap, theTacticSymbols, theKickWheel);
  }
};