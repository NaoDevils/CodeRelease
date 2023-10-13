#include "KickBallTestObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>

KickBallTestObjective::KickBallTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName) : Objective("TestObjective", role, logger)
{
  std::vector<std::string> kickNames = {};
  kickNames.emplace_back(kickName);
  kicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), kickNames);
}

bool KickBallTestObjective::perform(Ballchaser& ballchaser)
{
  const Filterer filterer = Filterer();
  const Factors factors = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f};
  const auto currentKick = kickManager.getCurrentKick(role->ballPosition);
  const auto kickPlanOptional = SelectFunctions::createAndFilterAndSelect(
      role->theRobotPoseAfterPreview, role->ballPosition, KickUtils::unpack(kicks), currentKick, filterer, factors, role->theFieldDimensions, role->theHeatMapCollection, role->theKickWheel, role->theRobotMap, role->theTacticSymbols);
  if (kickPlanOptional.has_value())
  {
    kickManager.kickTo(ballchaser, kickPlanOptional.value(), role->theFrameInfo);
  }
  else
  {
    OUTPUT_WARNING("Kick not possible!");
  }
  return true;
}
