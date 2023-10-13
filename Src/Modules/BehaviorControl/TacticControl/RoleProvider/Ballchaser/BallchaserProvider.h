/**
* @file BallchaserProvider.h
*
* Declaration of class BallchaserProvider.
* Provides optimal kick position and kick for current situation if robot is the ball chaser.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
* Kick target is in world coordinates here (gets translated to relative in CABSL).
*/

#pragma once

#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/GoalObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickBallTestObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/MoveObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickToCenterTestObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/Danger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/ObjectivesManager.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/HeatMapCollection.h"
#include "Representations/Modeling/KickWheel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Tools/Settings.h"

MODULE(BallchaserProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(GoalSymbols),
  REQUIRES(HeatMapCollection),
  REQUIRES(KickWheel),
  REQUIRES(MotionInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(Receiver),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSelection),
  REQUIRES(TacticSymbols),
  REQUIRES(TeammateData),
  REQUIRES(WalkingEngineParams),

  USES(PositioningSymbols),

  PROVIDES(Ballchaser),

  LOADS_PARAMETERS(,
  (bool)(true) previewArrival, /**< If true, the robot will use preview for decision if robot has arrived (faster kick trigger). */

  (std::vector<std::string>) moveObjectiveNoDangerKickNames,
  (std::vector<std::string>) moveObjectiveDangerKickNames,
  (std::vector<std::string>) oneVsOneObjectiveQuickKickNames,
  (std::vector<std::string>) oneVsOneObjectiveTrickKickNames,
  (std::vector<std::string>) goalObjectiveNoDangerKickNames,
  (std::vector<std::string>) goalObjectiveDangerKickNames,
  (std::vector<std::string>) kickOffKickNames,
  (std::vector<std::string>) kickInKickNames,
  (std::vector<std::string>) cornerKickNames,
  (std::vector<std::string>) goalKickNames,
  (std::vector<std::string>) penaltyKickNames,
  (std::vector<std::string>) pushingFreeKickNames,

  (Angle)(30_deg) oneVsOneObjective_quickKickConeSize,

  (float)(1000.f) oppSetPlayBallDistance
));

class BallchaserProvider : public BallchaserProviderBase, public RoleProvider<Ballchaser>
{

public:
  BallchaserProvider();
  void init();

  Vector2f ballPosition = Vector2f::Zero();
  bool ballOnLeftSide = false;
  Danger danger = Danger::NONE;
  int startedSetPlayWaitingRemainingTime = 0;

private:
  void update(Ballchaser& ballchaser) override;

  void stateReady_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition) override;
  void stateReady_kickOff_opponent(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition) override;
  void statePlaying_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition) override;
  float goalKick_own(Ballchaser& positioningSymbols, bool left) override;
  float goalKick_opponent(Ballchaser& positioningSymbols, bool left) override;
  float pushingFreeKick_own(Ballchaser& positioningSymbols) override;
  float pushingFreeKick_opponent(Ballchaser& positioningSymbols) override;
  float cornerKick_own(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(Ballchaser& positioningSymbols, bool left) override;
  float kickIn_opponent(Ballchaser& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(Ballchaser& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(Ballchaser& positioningSymbols) override;
  float statePlaying_penaltyKick_own(Ballchaser& positioningSymbols) override;
  void regularPlay(Ballchaser& positioningSymbols) override;

  void actInOwnSetPlay(Ballchaser& ballchaser, const Vector2f& waitPosition, float minX, float goalsHeatFactor, const std::vector<Kick*>& kicks);
  bool waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition);

  void actInOpponentsSetPlay(Ballchaser& ballchaser, bool left, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose);
  void defendOwnGoal(Ballchaser& ballchaser, bool leftSide);

  void updateVariables(Ballchaser& ballchaser);
  [[nodiscard]] bool isDanger(float dangerDistance, bool hysteresis) const;

  ObjectivesManager<BallchaserProvider, Ballchaser> regularPlayObjectivesManager;
  KickManager setPlayKickManager;
  std::vector<std::unique_ptr<Kick>> kickOffKicks;
  std::vector<std::unique_ptr<Kick>> kickInKicks;
  std::vector<std::unique_ptr<Kick>> cornerKicks;
  std::vector<std::unique_ptr<Kick>> goalKicks;
  std::vector<std::unique_ptr<Kick>> penaltyKicks;
  std::vector<std::unique_ptr<Kick>> pushingFreeKicks;
  BehaviorLogger logger;

  bool testBehavior = false;
};
