#include "MoveObjective.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/ExecutableKicks/ExecutableKicks.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/FastLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SlowLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHack.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackVeryLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Rotate45Kick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuter45.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuterFoot.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysterUtils.h"

MoveObjective::MoveObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("MoveObjective", role, logger)
{
  //kickManager.add(std::make_unique<FastLongKick>());
  //kicks.push_back(std::make_unique<SlowLongKick>());
  kicks.push_back(std::make_unique<KickHack>());
  kicks.push_back(std::make_unique<KickHackLong>());
  kicks.push_back(std::make_unique<KickHackVeryLong>());
  kicks.push_back(std::make_unique<Rotate45Kick>());
  kicks.push_back(std::make_unique<SideKickOuter45>());
  //kicks.push_back(std::make_unique<SideKickOuterFoot>());
}

bool MoveObjective::perform(Ballchaser& ballchaser)
{
  update();

  float timeFactor;
  switch (role->danger)
  {
  case HIGH:
    OUTPUT_ERROR("Should not get reached! Start 1v1 instead!");
  case MEDIUM:
    timeFactor = -1.f;
    break;
  case LOW:
    timeFactor = -0.5f;
    break;
  case NONE:
    timeFactor = -0.3f;
    break;
  case IMPOSSIBLE:
    timeFactor = 0.f;
    break;
  default:
    throw std::logic_error("");
  }

  ExecutableKicks executableKicks =
      kickManager
          .getExecutableKicks(role->theRobotPoseAfterPreview, role->ballPosition, KickUtils::unpack(kicks), role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap)
          .filterTooFarBack(std::min(role->ballPosition.x() + 500.f, role->theFieldDimensions.xPosOpponentPenaltyArea))
          .filterOutside(role->theFieldDimensions)
          .filterBlocked(role->theFieldDimensions, role->theRobotMap)
          .reduceToBest(timeFactor, 0.f, 0.5f, -0.1f, 1.f, 0.f, 0.f, 1.f, 0.f, -0.2f, -1.f, role->kickWithLeft, role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap);

  bool suc = executableKicks.hasBest();
  if (suc)
  {
    kickManager.kickTo(ballchaser, executableKicks);
  }
  logger.addReason(suc, "KickBestWithFactors");
  return suc;
}

void MoveObjective::update()
{
  kickManager.update(role->ballPosition, role->theRobotPoseAfterPreview, role->theWalkingEngineParams);
}
