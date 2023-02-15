/**
* @file BallchaserProvider.h
*
* Declaration of class BallchaserProvider.
* Provides optimal kick position and kick for current situation if robot is the ball chaser.
* Uses only walking engine kicks and dribbling.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
* Kick target is in world coordinates here (gets translated to relative in CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Ballchaser/TestObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Ballchaser/GoalObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Ballchaser/MoveObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Ballchaser/OneVsOneObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/ObjectivesManager.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallUtils.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h>
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/HeatMapCollection.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "RoleProvider.h"
#include "Tools/Settings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"
#include <Modules/BehaviorControl/BehaviorHelper.h> // TODO Remove Redundancy with Utils
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/Danger.h>

MODULE(BallchaserProvider,
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
  REQUIRES(LocalRobotMap),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSelection),
  REQUIRES(Receiver),
  REQUIRES(TeammateData),
  REQUIRES(TacticSymbols),
  REQUIRES(WalkingEngineParams),
  REQUIRES(HeatMapCollection),

  USES(PositioningSymbols),

  PROVIDES(Ballchaser),

  LOADS_PARAMETERS(,

    (float)(50.f) footDecisionHysteresis, /**< Min distance difference to change kick foot */
    (bool)(true) previewArrival, /**< If true, the robot will use preview for decision if robot has arrived (faster kick trigger). */
    (bool)(true) usePredictedBallPosition,
    (bool)(true) playAgainstDribbleTeam,

    (float)(300.f) penaltyKickOpponentPositionX,
		(float)(800.f) penaltyKickOpponentPositionY,

    (float)(1000.f) oppSetPlayBallDistance,

    (float)(1200.f) moveObjective_closeToMoveLineDistance,
    (float)(500.f) moveObjective_sidesToBallOffset,
    (float)(-20.f) moveObjective_oppGoalLineXOffset,
    (float)(-50.f) moveObjective_oppPenaltyAreaXOffset
  )
);

class BallchaserProvider : public BallchaserProviderBase, public RoleProvider<Ballchaser>
{

public:
  BallchaserProvider();

  Vector2f ballPosition = Vector2f::Zero();
  bool ballOnLeftSide = false;
  bool kickWithLeft = false;
  Danger danger = Danger::NONE;
  int timeArrivedInSetPlayWaiting = 0;

private:
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

  bool waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition);

  void freeKick(Ballchaser& ballchaser, const Vector2f& waitPosition, const float minX, const float opponentGoalHeatFactor);

  void update(Ballchaser& ballchaser) override;
  void updateVariables();
  void updateKickWithLeft();
  void updateBallPosition(const Vector2f* useBallPosition);
  void updateDanger(Ballchaser& ballchaser);

  ObjectivesManager<BallchaserProvider, Ballchaser> regularPlayObjectivesManager;
  KickManager setPlayKickManager;
  std::vector<std::unique_ptr<Kick>> kickOffKicks;
  std::vector<std::unique_ptr<Kick>> kickInKicks;
  std::vector<std::unique_ptr<Kick>> cornerKicks;
  std::vector<std::unique_ptr<Kick>> goalKick_Kicks;
  std::vector<std::unique_ptr<Kick>> penaltyKicks;
  BehaviorLogger logger;
};
