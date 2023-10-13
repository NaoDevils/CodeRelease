#include "KickToCenterTestObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/TargetFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>


KickToCenterTestObjective::KickToCenterTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName) : Objective("TestObjective", role, logger)
{
  std::vector<std::string> kickNames = {};
  kickNames.emplace_back(kickName);
  kicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), kickNames);
}

bool KickToCenterTestObjective::perform(Ballchaser& ballchaser)
{
  const Filterer filterer = {};
  const Factors factors = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f};
  const Vector2f targetPosition = Vector2f(0, 0);
  const std::optional<KickPlan> selectableKickOptional = TargetFunctions::getKickPlanForTarget(role->theRobotPoseAfterPreview,
      role->ballPosition,
      targetPosition,
      DistanceRequirement::mayShorterOrFurther,
      KickUtils::unpack(kicks),
      kickManager.getCurrentKick(role->ballPosition),
      filterer,
      factors,
      role->theFieldDimensions,
      role->theHeatMapCollection,
      role->theRobotMap,
      role->theTacticSymbols,
      role->theKickWheel);
  if (selectableKickOptional.has_value())
  {
    kickManager.kickTo(ballchaser, selectableKickOptional.value(), role->theFrameInfo);
    return true;
  }
  logger.addSuccessReason("Cant Kick!");
  const Vector2f defensivePosition = BallchaserUtils::getWaitPosition(role->ballPosition, role->theFieldDimensions);
  PositionUtils::setPosition(ballchaser, defensivePosition);
  return true;
}

bool KickToCenterTestObjective::leaveCondition() const
{
  return false;
}

void KickToCenterTestObjective::postprocess()
{
  Objective::postprocess();
}
