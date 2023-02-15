#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"

class BallchaserProvider;
struct Ballchaser;
class NoDeadlockObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  NoDeadlockObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool perform(Ballchaser& ballchaser) override;
};
