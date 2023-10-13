#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h>

class BallchaserProvider;
struct Ballchaser;
class KickToCenterTestObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  KickToCenterTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName);
  bool perform(Ballchaser& ballchaser) override;
  bool leaveCondition() const override;
  void postprocess() override;

private:
  KickManager kickManager = {};
  std::vector<std::unique_ptr<Kick>> kicks = {};
};
