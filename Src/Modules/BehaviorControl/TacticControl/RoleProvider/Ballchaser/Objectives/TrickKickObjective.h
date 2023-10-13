#pragma once

#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"

class BallchaserProvider;
struct Ballchaser;
class TickKickObjective : public Objective<BallchaserProvider, Ballchaser>
{
public:
  TickKickObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool enterCondition() override;
  bool perform(Ballchaser& ballchaser) override;
  [[nodiscard]] bool leaveCondition() const override;
  void postprocess() override;

private:
  KickManager kickManager;
  std::vector<std::unique_ptr<Kick>> kicks = {};

  bool isBlockingImportantKickForOpponent();
  bool performTrickKick(Ballchaser& ballchaser);
};
