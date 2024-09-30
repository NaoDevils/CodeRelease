#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(BackupBallchaserProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotPose),
  REQUIRES(TacticSymbols),
  PROVIDES(BackupBallchaser)
);

class BackupBallchaserProvider : public BackupBallchaserProviderBase, public RoleProvider<BackupBallchaser>
{

public:
  void update(BackupBallchaser& role) override;

private:
  void stateReady_kickOff_own(BackupBallchaser& role, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(BackupBallchaser& role, const Vector2f& ballPosition) override;
  float goalKick_own(BackupBallchaser& role, bool left) override;
  float goalKick_opponent(BackupBallchaser& role, bool left) override;
  float pushingFreeKick_own(BackupBallchaser& role) override;
  float pushingFreeKick_opponent(BackupBallchaser& role) override;
  float cornerKick_own(BackupBallchaser& role, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(BackupBallchaser& role, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(BackupBallchaser& role, bool left) override;
  float kickIn_opponent(BackupBallchaser& role, bool left) override;
  float stateReady_penaltyKick_own(BackupBallchaser& role) override;
  float stateReady_penaltyKick_opponent(BackupBallchaser& role) override;
  void regularPlay(BackupBallchaser& role) override;

  void updateBallIsLeft(const Vector2f& ballPosition);
  bool ballIsLeft = false;
};
