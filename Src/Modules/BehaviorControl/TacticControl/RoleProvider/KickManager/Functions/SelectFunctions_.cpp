#include "SelectFunctions.h"

#include <Representations/Modeling/HeatMapCollection.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/SelectableKick.h>

std::optional<KickPlan> SelectFunctions::createAndFilterAndSelect(const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const std::vector<Kick*>& kicks,
    const std::optional<CurrentKick>& currentKick,
    const Filterer& filterer,
    const Factors& factors,
    const FieldDimensions& theFieldDimensions,
    const HeatMapCollection& theHeatMapCollection,
    const KickWheel& theKickWheel,
    const RobotMap& theRobotMap,
    const TacticSymbols& theTacticSymbols)
{
  std::vector<SelectableDirection> selectableDirections = getSelectableDirections(ballPosition, filterer, theKickWheel);
  if (selectableDirections.empty())
  {
    return {};
  }

  const std::vector<std::function<bool(SelectableKick&)>> selectableKickFilters = filterer.getEfficientlyOrderedSelectableKickFilters();

  std::vector<Vector2f> draw_targets = {};
  std::vector<float> draw_scores = {};
  std::optional<KickPlan> bestKickPlan = {};

  for (Kick* kick : kicks)
  {
    SelectableKick selectableKick = {kick};
    bool valid = true;
    for (const auto& filter : selectableKickFilters)
    {
      if (filter(selectableKick))
      {
        valid = false;
        break;
      }
    }
    if (!valid)
    {
      continue;
    }

    std::vector<SelectableTarget> selectableTargets = {};
    for (const SelectableDirection& selectableDirection : selectableDirections)
    {
      addSelectableTargets(selectableTargets, selectableDirection, kick, filterer);
    }
    if (selectableTargets.empty())
    {
      continue;
    }

    std::vector<SelectablePose> selectablePoses = {};
    for (SelectableTarget& selectableTarget : selectableTargets)
    {
      auto selectablePoseOptional = getBestSelectablePose(playerPose, selectableTarget, filterer, factors, theTacticSymbols);
      if (selectablePoseOptional.has_value())
      {
        selectablePoses.push_back(selectablePoseOptional.value());
      }
    }

    for (SelectablePose& selectablePose : selectablePoses)
    {
      KickPlan kickPlan = {selectablePose, false};
      ScoreFunctions::scoreKickPlan(kickPlan, factors, theFieldDimensions, theHeatMapCollection, theRobotMap, theTacticSymbols);

      draw_targets.push_back(kickPlan.selectablePose.selectableTarget.target);
      draw_scores.push_back(kickPlan.score);

      if (!bestKickPlan.has_value() || kickPlan.score > bestKickPlan->score)
      {
        bestKickPlan = kickPlan;
      }
    }
  }

  DrawFunctions::drawFree(draw_targets, draw_scores);

  if (currentKick.has_value())
  {
    std::vector<KickPlan> kickPlans = {};
    kickPlans.emplace_back(currentKick.value().toKickPlan(playerPose, ballPosition));
    auto currentKickPlanOptional = filterAndSelect(kickPlans, filterer, factors, theFieldDimensions, theHeatMapCollection, theRobotMap, theTacticSymbols, theKickWheel);
    if (currentKickPlanOptional.has_value())
    {
      if (bestKickPlan.has_value())
      {
        auto currentKickPlan = currentKickPlanOptional.value();
        const float distanceToKick = Geometry::distance(playerPose.translation, ballPosition);
        currentKickPlan.score = ScoreFunctions::applyHysteresis(currentKickPlan.score, distanceToKick);
        if (currentKickPlan.score > bestKickPlan->score)
        {
          return currentKickPlanOptional;
        }
      }
      else
      {
        return currentKickPlanOptional;
      }
    }
  }

  return bestKickPlan;
}

std::optional<KickPlan> SelectFunctions::filterAndSelect(std::vector<KickPlan>& kickPlans,
    const Filterer& filterer,
    const Factors& factors,
    const FieldDimensions& theFieldDimensions,
    const HeatMapCollection& theHeatMapCollection,
    const RobotMap& theRobotMap,
    const TacticSymbols& theTacticSymbols,
    const KickWheel& theKickWheel)
{
  const std::vector<std::function<bool(SelectableKick&)>> selectableKickFilters = filterer.getEfficientlyOrderedSelectableKickFilters();
  const std::vector<std::function<bool(SelectableDirection&)>> selectableDirectionFilters = filterer.getEfficientlyOrderedSelectableDirectionFilters();
  const std::vector<std::function<bool(SelectableTarget&)>> selectableTargetFilters = filterer.getEfficientlyOrderedSelectableTargetFilters();
  const std::vector<std::function<bool(SelectablePose&)>> selectablePoseFilters = filterer.getEfficientlyOrderedSelectablePoseFilters();

  std::optional<KickPlan> bestKickPlan = {};
  for (auto& kickPlan : kickPlans)
  {
    bool valid = true;
    for (const auto& filter : selectableKickFilters)
    {
      if (filter(kickPlan.selectablePose.selectableTarget.selectableKick))
      {
        valid = false;
        break;
      }
    }
    if (!valid)
    {
      continue;
    }
    const Angle angle = (kickPlan.selectablePose.selectableTarget.target - kickPlan.selectablePose.selectableTarget.ballPosition).angle();
    const DistanceInfo distanceInfo = theKickWheel.getDistance(angle, kickPlan.hysteresis);
    const float requiredDistance = Geometry::distance(kickPlan.selectablePose.selectableTarget.ballPosition, kickPlan.selectablePose.selectableTarget.target);
    if (distanceInfo.distance < requiredDistance)
    {
      continue;
    }
    for (const auto& filter : selectableDirectionFilters)
    {
      SelectableDirection selectableDirection = {kickPlan.selectablePose.selectableTarget.ballPosition, angle, distanceInfo.distance, distanceInfo.distanceBlocked, distanceInfo.distanceOutside};
      if (filter(selectableDirection))
      {
        valid = false;
        break;
      }
    }
    if (!valid)
    {
      continue;
    }
    for (const auto& filter : selectableTargetFilters)
    {
      if (filter(kickPlan.selectablePose.selectableTarget))
      {
        valid = false;
        break;
      }
    }
    if (!valid)
    {
      continue;
    }
    for (const auto& filter : selectablePoseFilters)
    {
      if (filter(kickPlan.selectablePose))
      {
        valid = false;
        break;
      }
    }
    if (!valid)
    {
      continue;
    }
    ScoreFunctions::scoreKickPlan(kickPlan, factors, theFieldDimensions, theHeatMapCollection, theRobotMap, theTacticSymbols);
    if (!bestKickPlan.has_value() || kickPlan.score > bestKickPlan.value().score)
    {
      bestKickPlan = kickPlan;
    }
  }
  return bestKickPlan;
}