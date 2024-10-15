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
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/RecommendedKickObjective.h"
#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/KickManager.h>
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
#include "Representations/BehaviorControl/RoleSymbols/RemoteControl.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
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
  USES(RemoteControl),
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
  USES(PositioningAndKickSymbols),

  PROVIDES(Ballchaser),
  PROVIDES(BallchaserHeadPOIList),

  HAS_PREEXECUTION,

  LOADS_PARAMETERS(,
  (bool)(true) previewArrival, /**< If true, the robot will use preview for decision if robot has arrived (faster kick trigger). */

  (std::vector<std::string>) penaltyKickNames,

  (float)(1000.f) oppSetPlayBallDistance
));

class BallchaserProvider : public BallchaserProviderBase
{

public:
  BallchaserProvider();
  void init();

private:
  void update(Ballchaser& ballchaser) override;
  void update(BallchaserHeadPOIList& ballchaserHeadPoiList) override;
  void execute(tf::Subflow& subflow) override;
  static void declareDebugDrawings();
  void readTestBehaviorCommandsAndUpdate();

  void kickOff_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const bool readyElseSet);
  void kickOff_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const bool readyElseSet);
  void goalKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void pushingFreeKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void cornerKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void cornerKick_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void kickIn_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void kickIn_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void penaltyKick_own_ready(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void penaltyKick_opponent_ready(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void penaltyKick_own_playing(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);
  void regularPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList);

  void actInOwnSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const Vector2f& waitPosition);
  bool waitInOwnSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const Vector2f& waitingPosition);

  void actInOpponentsSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose);
  void defendOwnGoal(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, bool recommendShot);

  ObjectivesManager<BallchaserProvider, Ballchaser> regularPlayObjectivesManager;
  CurrentKickManager setPlayCurrentKickManager;
  std::vector<std::unique_ptr<Kick>> penaltyKicks;
  BehaviorLogger logger;
  bool testMode = false;
  KickManager kickManager = {};

  bool testBehavior = false;

  Ballchaser localBallchaser;
  BallchaserHeadPOIList localBallchaserHeadPoiList;
  int startedSetPlayWaitingRemainingTime = 0;
};
