#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/RoleSymbols/FrontWing.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(FrontWingProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(TacticSymbols),
  REQUIRES(RobotMap),
  USES(RoleSymbols),
  PROVIDES(FrontWing)
);

class FrontWingProvider : public FrontWingProviderBase, public RoleProvider<FrontWing>
{

public:
  void update(FrontWing& role) override;

private:
  void stateReady_kickOff_own(FrontWing& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(FrontWing& role, const Vector2f& ballPosition) override;
  float goalKick_own(FrontWing& role, bool left) override;
  float goalKick_opponent(FrontWing& role, bool left) override;
  float pushingFreeKick_own(FrontWing& role) override;
  float pushingFreeKick_opponent(FrontWing& role) override;
  float cornerKick_own(FrontWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(FrontWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(FrontWing& role, bool left) override;
  float kickIn_opponent(FrontWing& role, bool left) override;
  float stateReady_penaltyKick_own(FrontWing& role) override;
  float stateReady_penaltyKick_opponent(FrontWing& role) override;
  void regularPlay(FrontWing& role) override;
};
