/**
* @file BallchaserProvider.h
*
* Declaration of class BallchaserProvider.
* Provides optimal kick position and kick for current situation if robot is the ball chaser.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
* Kick target is in world coordinates here (gets translated to relative in CABSL).
*/

#pragma once

#include <Representations/BehaviorControl/RoleSymbols/BallchaserHeadPOIList.h>
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/CurrentKickManager.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/ExecuteRecommendationObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/ObjectivesManager.h"
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
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RecommendedKick/RecommendedKick.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Tools/Settings.h"

MODULE(BallchaserProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(DirectionInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(GoalSymbols),
  REQUIRES(MotionInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PositionInfo),
  REQUIRES(Receiver),
  REQUIRES(RecommendedKick),
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
  PROVIDES(BallchaserHeadPOIList),

  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
  (bool)(true) previewArrival, /**< If true, the robot will use preview for decision if robot has arrived (faster kick trigger). */

  (std::vector<std::string>) penaltyKickNames,

  (float)(1000.f) oppSetPlayBallDistance
));

class BallchaserProvider : public BallchaserProviderBase, public RoleProvider<Ballchaser>
{

public:
  BallchaserProvider();
  void init();

  int startedSetPlayWaitingRemainingTime = 0;

private:
  void update(Ballchaser& ballchaser) override;
  void update(BallchaserHeadPOIList& ballchaserHeadPoiList) override;
  void execute(tf::Subflow& subflow) override;

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

  void actInOwnSetPlay(Ballchaser& ballchaser, const Vector2f& waitPosition);
  bool waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition);

  void actInOpponentsSetPlay(Ballchaser& ballchaser, bool left, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose);
  void defendOwnGoal(Ballchaser& ballchaser, bool leftSide);

  ObjectivesManager<BallchaserProvider, Ballchaser> regularPlayObjectivesManager;
  CurrentKickManager setPlayCurrentKickManager;
  std::vector<std::unique_ptr<Kick>> penaltyKicks;
  BehaviorLogger logger;

  bool testBehavior = false;

  Ballchaser localBallchaser;
  BallchaserHeadPOIList localBallchaserHeadPoiList;
  void fillHeadPoiList(BallchaserHeadPOIList& ballchaserHeadPoiList, const std::optional<Vector2f>& targetOptional);
};
