#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Rotate45Kick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHack.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/KickManager.h>

class BallchaserProvider;
struct Ballchaser;
class TestObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  TestObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool perform(Ballchaser& ballchaser) override;
  bool leaveCondition() const override;

private:
  KickManager kickManager = {};
  std::vector<std::unique_ptr<Kick>> kicks = {};
};
