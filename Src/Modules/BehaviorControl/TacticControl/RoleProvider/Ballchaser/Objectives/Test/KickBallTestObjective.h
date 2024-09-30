#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/ObjectivesManager.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/Objective.h>

class BallchaserProvider;
struct Ballchaser;
class KickBallTestObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  KickBallTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName);
  bool perform(Ballchaser& ballchaser) override;

private:
  CurrentKickManager currentKickManager = {};
  std::vector<std::unique_ptr<Kick>> kicks = {};

  std::optional<Vector2f> targetDirectionVectorOptional = {};
};
