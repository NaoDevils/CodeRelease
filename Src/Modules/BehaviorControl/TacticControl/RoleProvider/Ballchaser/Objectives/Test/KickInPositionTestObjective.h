#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/Danger.h>

class BallchaserProvider;
struct Ballchaser;
class KickInPositionTestObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  KickInPositionTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName);
  bool perform(Ballchaser& ballchaser) override;

private:
  KickManager kickManager = {};
  std::vector<std::unique_ptr<Kick>> kicks = {};
  bool kicked = false;
};
