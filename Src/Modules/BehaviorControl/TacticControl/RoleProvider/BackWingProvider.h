#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BackWing.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(BackWingProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(TacticSymbols),
  USES(RoleSymbols),
  PROVIDES(BackWing)
);

class BackWingProvider : public BackWingProviderBase, public RoleProvider<BackWing>
{

public:
  void update(BackWing& role) override;

private:
  void stateReady_kickOff_own(BackWing& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(BackWing& role, const Vector2f& ballPosition) override;
  float goalKick_own(BackWing& role, bool left) override;
  float goalKick_opponent(BackWing& role, bool left) override;
  float pushingFreeKick_own(BackWing& role) override;
  float pushingFreeKick_opponent(BackWing& role) override;
  float cornerKick_own(BackWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(BackWing& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(BackWing& role, bool left) override;
  float kickIn_opponent(BackWing& role, bool left) override;
  float stateReady_penaltyKick_own(BackWing& role) override;
  float stateReady_penaltyKick_opponent(BackWing& role) override;
  void regularPlay(BackWing& role) override;

  void updateBallIsLeft(const Vector2f& ballPosition);
  bool ballIsLeft = false;
};
