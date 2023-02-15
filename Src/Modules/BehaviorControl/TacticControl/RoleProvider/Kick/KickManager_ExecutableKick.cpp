#include "KickManager.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h>
#include <Representations/Modeling/HeatMapCollection.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h>

#include <utility>
#include "Tools/Math/Geometry.h"
#include "Representations/Modeling/RobotMap.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/ExecutableKicks/ExecutableKick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/KickRange.h"

ExecutableKicks KickManager::getExecutableKicks(
    const Pose2f& playerPose, const Vector2f& ballPosition, std::vector<Kick*> kicks, const FieldDimensions& theFieldDimensions, const HeatMapCollection& theHeatMapCollection, const RobotMap& theRobotMap)
{
  std::vector<Vector2f> targets = {};
  targets.emplace_back(ballPosition.x(), ballPosition.y() + 500.f);
  targets.emplace_back(ballPosition.x() + 500.f, ballPosition.y());
  targets.emplace_back(ballPosition.x(), ballPosition.y() - 500.f);
  targets.emplace_back(ballPosition.x() - 500.f, ballPosition.y());
  targets.emplace_back(ballPosition.x(), ballPosition.y() + 500.f);
  return getExecutableKicks(playerPose, {ballPosition, targets, DistanceRequirement::mayShorterOrFurther}, 0, kicks, theFieldDimensions, theHeatMapCollection, theRobotMap);
}

ExecutableKicks KickManager::getExecutableKicks(
    const Pose2f& playerPose, const KickRange& kickRange, const int stepSizeOrZeroForAngle, std::vector<Kick*> kicks, const FieldDimensions& theFieldDimensions, const HeatMapCollection& theHeatMapCollection, const RobotMap& theRobotMap)
{
  const size_t targetCount = kickRange.targets.size();

  ASSERT(targetCount >= 2);

  ExecutableKicks executableKicks = {playerPose, kickRange.ballPosition, {}};

  for (size_t i = 1; i < targetCount; i++)
  {
    const Vector2f& target1 = kickRange.targets.at(i - 1);
    const Vector2f& target2 = kickRange.targets.at(i);
    const DistanceRequirement& distanceRequirement = kickRange.distanceRequirements.at(i - 1);

    std::vector<Vector2f> targets = stepSizeOrZeroForAngle <= 0
        ? KickUtils::getTargetsWithEvenAngle(kickRange.ballPosition, target1, target2)
        : KickUtils::getTargetsWithEvenDistance(target1, target2, stepSizeOrZeroForAngle);
    auto straightAheadTargetOptional = KickUtils::getStraightAheadTarget(playerPose.translation, kickRange.ballPosition, target1, target2);
    if (straightAheadTargetOptional.has_value())
    {
      targets.push_back(straightAheadTargetOptional.value());
    }

    for (const Vector2f& target : targets)
    {
      std::vector<ExecutableKick> executableKicksForTarget =
          getExecutableKicks(playerPose, kickRange.ballPosition, target, distanceRequirement, kicks, theFieldDimensions, theHeatMapCollection, theRobotMap);

      executableKicks.insert(executableKicksForTarget);
    }

    KickUtils::drawKickInfos(kickRange.ballPosition, target1, target2);
  }

  return executableKicks;
}

std::vector<ExecutableKick> KickManager::getExecutableKicks(const Pose2f& playerPose,
    const Vector2f& ballPosition,
    const Vector2f& targetPosition,
    const DistanceRequirement distanceRequirement,
    std::vector<Kick*> kicks,
    const FieldDimensions& theFieldDimensions,
    const HeatMapCollection& theHeatMapCollection,
    const RobotMap& theRobotMap)
{
  const float distance = Geometry::distance(ballPosition, targetPosition);

  std::vector<ExecutableKick> executableKicks = {};

  for (Kick* kick : kicks)
  {
    // TODO If adjustableKicks are added, use another loop here

    Vector2f kickTargetPosition = kick->getRealisticTarget(ballPosition, targetPosition);

    const bool hysteresis = isCurrent(kick, kickTargetPosition);

    if (!KickUtils::fulfillsDistanceRequirements(*kick, distance, distanceRequirement, hysteresis))
    {
      continue;
    }

    const ExecutableKick executableKick = {kick, kickTargetPosition, hysteresis};

    executableKicks.push_back(executableKick);
  }
  return executableKicks;
}
