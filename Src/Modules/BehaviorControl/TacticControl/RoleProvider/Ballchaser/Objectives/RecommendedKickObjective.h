#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/Objective.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/CurrentKickManager.h"

class BallchaserProvider;
struct Ballchaser;
class RecommendedKickObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  RecommendedKickObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool perform(Ballchaser& ballchaser) override;
  [[nodiscard]] bool leaveCondition() const override;
  void postprocess() override;

private:
};
