#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/RightWing.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(RightWingProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(TacticSymbols),
  REQUIRES(RobotMap),
  USES(RoleSymbols),
  PROVIDES(RightWing)
);

class RightWingProvider : public RightWingProviderBase, public RoleProvider<RightWing>
{

public:
  void update(RightWing& role) override;

private:
  void stateReady_kickOff_own(RightWing& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(RightWing& role, const Vector2f& ballPosition) override;
  float goalKick_own(RightWing& role, bool left) override;
  float goalKick_opponent(RightWing& role, bool left) override;
  float pushingFreeKick_own(RightWing& role) override;
  float pushingFreeKick_opponent(RightWing& role) override;
  float cornerKick_own(RightWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(RightWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(RightWing& role, bool left) override;
  float kickIn_opponent(RightWing& role, bool left) override;
  float stateReady_penaltyKick_own(RightWing& role) override;
  float stateReady_penaltyKick_opponent(RightWing& role) override;
  void regularPlay(RightWing& role) override;

  void updateBallIsLeft(const Vector2f& ballPosition);
  bool ballIsLeft = false;
};
