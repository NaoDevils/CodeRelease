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
#include "Representations/BehaviorControl/RoleSymbols/LeftWing.h"
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

MODULE(LeftWingProvider,
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
};
