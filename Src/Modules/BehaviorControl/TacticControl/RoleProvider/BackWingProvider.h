#pragma once

#include <optional>
#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/RoleSymbols/BackWing.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/BallchaserKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Settings.h"
#include "RoleProvider.h"

MODULE(BackWingProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(GoalSymbols),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSelection),
  REQUIRES(Receiver),
  REQUIRES(TeammateData),
  REQUIRES(TacticSymbols),
  REQUIRES(WalkingEngineParams),
  USES(PositioningSymbols),
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
};
