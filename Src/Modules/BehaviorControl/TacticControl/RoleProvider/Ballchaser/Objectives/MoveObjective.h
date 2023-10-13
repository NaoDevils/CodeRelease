#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/Danger.h>

class BallchaserProvider;
struct Ballchaser;
class MoveObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  MoveObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool perform(Ballchaser& ballchaser) override;
  [[nodiscard]] bool leaveCondition() const override;
  void postprocess() override;

private:
  std::vector<Kick*> getKicks(Danger danger);

  static float getWidthFactor(Danger danger);
  static float getTimeFactor(Danger danger);

  KickManager kickManager;
  std::vector<std::unique_ptr<Kick>> noDangerKicks = {};
  std::vector<std::unique_ptr<Kick>> dangerKicks = {};
};
