#include <Modules/BehaviorControl/TacticControl/TacticProvider.h>
#include "SelectFunctions.h"

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/DirectionInfoProvider/DirectionInfoProvider.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/ExecutableShotSelector.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SelectableKick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/PositionInfoProvider/PositionInfoUtils.h"
#include "ProbabilityFunctions.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"

std::optional<ExecutableShot> SelectFunctions::createAndFilterAndSelect(const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const std::vector<Kick*>& kicks,
    const std::optional<CurrentKick>& currentKick,
    const Filterer& filterer,
    const ShotParameters& factors,
    const DirectionInfo& theDirectionInfo,
    const FieldDimensions& theFieldDimensions,
    const PositionInfo& thePositionInfo,
    const TacticSymbols& theTacticSymbols)
{
  std::vector<SelectableDirection> selectableDirections = createSelectableDirections(ballPosition, filterer, theDirectionInfo);
  if (selectableDirections.empty())
  {
    return {};
  }

  const float noSuccessScore = ScoreFunctions::getNoSuccessScore(factors.targetParameters);
  ExecutableShotSelector selector = {};
  std::vector<Vector2f> draw_targets = {};
  std::vector<float> draw_values = {};

  for (Kick* kick : kicks)
  {
    std::optional<SelectableKick> selectableKick = createSelectableKick(kick, filterer);
    if (!selectableKick.has_value())
    {
      continue;
    }

    const float kickScore = ScoreFunctions::scoreKick(kick, factors.kickParameters);
    selectableKick->setScore(kickScore);

    const ScoreFunctions::ProbabilityCone probabilityCone = {kick, theDirectionInfo.stepAngle};

    std::vector<float> targetedDistances = createMinToMaxTargetedDistances(selectableKick.value());
    std::map<int, SelectableTarget> blockedSelectableTargets = {};

    for (const float targetedDistance : targetedDistances)
    {
      std::vector<SelectableTarget> selectableTargets = {};
      selectableTargets.reserve(selectableDirections.size());
      for (int i = 0; i < (int)selectableDirections.size(); i++)
      {
        if (blockedSelectableTargets.contains(i))
        {
          // reuse first selectable target where the distance is blocked
          selectableTargets.push_back(blockedSelectableTargets.at(i));
        }
        else
        {
          const SelectableDirection& selectableDirection = selectableDirections.at(i);
          SelectableTarget selectableTarget = createAndFilterSelectableTarget(targetedDistance, selectableDirection, selectableKick.value(), filterer);
          if (selectableTarget.distanceInfo.isIntoBlocked)
          {
            blockedSelectableTargets.emplace(i, selectableTarget);
          }
          selectableTargets.push_back(selectableTarget);
        }
      }
      if (selectableTargets.empty())
      {
        continue;
      }

      const float ballGoalsHeat = PositionInfoUtils::getGoalsHeat(ballPosition, theFieldDimensions);
      for (SelectableTarget& selectableTarget : selectableTargets)
      {
        selectableTarget.setScore(ScoreFunctions::scoreTarget(ballGoalsHeat, selectableTarget, factors.targetParameters, theFieldDimensions, thePositionInfo));
      }

      std::vector<SelectableShot> selectableShots = createSelectableShots(selectableTargets);
      ASSERT(!selectableShots.empty());

      if (currentKick.has_value() && kick == currentKick->kick)
      {
        auto executableShotOptional = SelectFunctions::createExecutableShotFromCurrentKick(
            playerPose, ballPosition, selectableShots, currentKick.value(), filterer, factors, theDirectionInfo, theFieldDimensions, thePositionInfo, theTacticSymbols);
        if (executableShotOptional.has_value())
        {
          selector.add(executableShotOptional.value());
        }
      }

      for (SelectableShot& selectableShot : selectableShots)
      {
        if (selectableShot.isFiltered())
        {
          continue;
        }

        const float shotScore = ScoreFunctions::scoreSelectableShot(selectableShot, probabilityCone);
        selectableShot.setScore(shotScore);

        auto selectablePoses = createSelectablePoses(selectableShot, filterer, factors, theTacticSymbols);

        float draw_value = std::numeric_limits<float>::lowest();

        for (SelectablePose& selectablePose : selectablePoses)
        {
          const float poseTime = PathUtils::getPathTime(playerPose, selectablePose.pose, ballPosition);
          selectablePose.setPoseTime(poseTime);
          const float poseScore = ScoreFunctions::scorePose(selectablePose, factors.poseParameters, theTacticSymbols);
          selectablePose.setScore(poseScore);

          ExecutableShot executableShot = {selectablePose, false};
          const float successProbability = ProbabilityFunctions::getTimeProbability(executableShot.selectablePose.getTotalTime(), theTacticSymbols);
          executableShot.setSuccessProbability(successProbability);
          const float executableShotScore = ScoreFunctions::scoreExecutableShot(executableShot, successProbability, noSuccessScore);
          executableShot.setScore(executableShotScore);
          selector.add(executableShot);

          draw_value = std::max(draw_value, executableShot.getScore());
        }

        if (!selectablePoses.empty())
        {
          draw_targets.push_back(selectableShot.selectableTarget.target);
          draw_values.push_back(draw_value);
        }
      }
    }
  }

  DrawFunctions::drawFree(draw_targets, draw_values);

  return selector.get();
}

std::optional<ExecutableShot> SelectFunctions::createExecutableShotFromCurrentKick(const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const std::vector<SelectableShot>& selectableShots,
    const CurrentKick& currentKick,
    const Filterer& filterer,
    const ShotParameters& factors,
    const DirectionInfo& theDirectionInfo,
    const FieldDimensions& theFieldDimensions,
    const PositionInfo& thePositionInfo,
    const TacticSymbols& theTacticSymbols)
{
  const int index = theDirectionInfo.getIndex((currentKick.target - ballPosition).angle());
  auto selectableDirectionOptional = createSelectableDirection(index, ballPosition, filterer, theDirectionInfo);
  if (!selectableDirectionOptional.has_value())
  {
    return {};
  }
  const SelectableKick selectableKick = SelectableKick(currentKick.kick);

  const float currentKickDistance = Geometry::distance(ballPosition, currentKick.target);
  SelectableTarget selectableTarget = createAndFilterSelectableTarget(currentKickDistance, selectableDirectionOptional.value(), selectableKick, filterer);
  if (selectableTarget.isFiltered())
  {
    return {};
  }

  const float ballGoalsHeat = PositionInfoUtils::getGoalsHeat(ballPosition, theFieldDimensions);
  selectableTarget.setScore(ScoreFunctions::scoreTarget(ballGoalsHeat, selectableTarget, factors.targetParameters, theFieldDimensions, thePositionInfo));

  auto currentSelectableShot = SelectableShot(selectableTarget);
  const Angle currentAngle = currentSelectableShot.selectableTarget.selectableDirection.angle;
  for (const SelectableShot& selectableShot : selectableShots)
  {
    const Angle angle = selectableShot.selectableTarget.selectableDirection.angle;
    if (angle > currentAngle)
    {
      currentSelectableShot.leftSelectableShot = selectableShot.leftSelectableShot;
      currentSelectableShot.rightSelectableShot = &selectableShot;
    }
  }

  const Pose2f pose = selectableTarget.selectableKick.kick->getKickPose(selectableTarget.selectableDirection.ballPosition, selectableTarget.selectableDirection.angle, currentKick.kickWithLeft);
  SelectablePose selectablePose = {currentSelectableShot, pose, currentKick.kickWithLeft};
  for (const auto& filter : filterer.getEfficientlyOrderedSelectablePoseFilters())
  {
    if (filter(selectablePose))
    {
      return {};
    }
  }

  ExecutableShot es = ExecutableShot(selectablePose, true);

  const float kickScore = ScoreFunctions::scoreKick(currentKick.kick, factors.kickParameters);
  es.selectablePose.selectableShot.selectableTarget.selectableKick.setScore(kickScore);

  const ScoreFunctions::ProbabilityCone probabilityCone = {selectableKick.kick, theDirectionInfo.stepAngle};
  const float shotScore = ScoreFunctions::scoreSelectableShot(currentSelectableShot, probabilityCone);
  es.selectablePose.selectableShot.setScore(shotScore);

  const float poseTime = PathUtils::getPathTime(playerPose, selectablePose.pose, ballPosition);
  es.selectablePose.setPoseTime(poseTime);
  const float poseScore = ScoreFunctions::scorePose(es.selectablePose, factors.poseParameters, theTacticSymbols);
  es.selectablePose.setScore(poseScore);

  const float successProbability = ProbabilityFunctions::getTimeProbability(es.selectablePose.getTotalTime(), theTacticSymbols);
  es.setSuccessProbability(successProbability);
  const float executableShotScore = ScoreFunctions::scoreExecutableShot(es, successProbability, ScoreFunctions::getNoSuccessScore(factors.targetParameters));
  es.setScore(executableShotScore);
  return es;
}