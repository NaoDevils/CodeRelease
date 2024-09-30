#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/LeftWing.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(LeftWingProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(TacticSymbols),
  REQUIRES(RobotMap),
  USES(RoleSymbols),
  PROVIDES(LeftWing)
);

class LeftWingProvider : public LeftWingProviderBase, public RoleProvider<LeftWing>
{

public:
  void update(LeftWing& role) override;

private:
  void stateReady_kickOff_own(LeftWing& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(LeftWing& role, const Vector2f& ballPosition) override;
  float goalKick_own(LeftWing& role, bool left) override;
  float goalKick_opponent(LeftWing& role, bool left) override;
  float pushingFreeKick_own(LeftWing& role) override;
  float pushingFreeKick_opponent(LeftWing& role) override;
  float cornerKick_own(LeftWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(LeftWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(LeftWing& role, bool left) override;
  float kickIn_opponent(LeftWing& role, bool left) override;
  float stateReady_penaltyKick_own(LeftWing& role) override;
  float stateReady_penaltyKick_opponent(LeftWing& role) override;
  void regularPlay(LeftWing& role) override;

  void updateBallIsLeft(const Vector2f& ballPosition);
  bool ballIsLeft = false;
};
