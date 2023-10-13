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
#include "Representations/BehaviorControl/RoleSymbols/FrontWing.h"
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

MODULE(FrontWingProvider,
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
