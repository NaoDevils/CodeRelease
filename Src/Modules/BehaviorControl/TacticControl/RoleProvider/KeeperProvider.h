/**
* @file KeeperProvider.h
*
* Declaration of class KeeperProvider.
* Provides positions for the keeper in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include <algorithm>
#include <float.h>

#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ObjectivesManager/ObjectivesManager.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h>
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/RoleSymbols/Keeper.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"
#include "Tools/Settings.h"
#include "Modules/BehaviorControl/CABSL/Libraries/HelperFunctions.h"
#include "RoleProvider.h"
MODULE(KeeperProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TeammateData),
  REQUIRES(RobotMap),
  REQUIRES(MotionInfo),
  REQUIRES(BallChaserDecision),
  REQUIRES(GameSymbols),
  REQUIRES(GoalSymbols),
  PROVIDES(Keeper),
  LOADS_PARAMETERS(,
    (float)(250.f) minBallDistanceForBlock,
    (float)(2000.f) maxBallDistanceForBlock,
    (float)(300.f) yReachableWithBlock,
    (float)(700.f) yReachableWithDive,
    (float)(200.f) distanceFromGroundLine,
    (float)(300.f) safeDistanceToGoalPost,
    (float)(3500.f) moveWithBallDistance,
    (float)(100.f) moveWithBallSpeed,
    (int)(8000) timeBetweenBlockMotions,
    (float)(-1000) borderForOwnHalfBehavior,
    (float)(3000) timeUntilSearchBehavior,
    (float)(6000) timeForSearchBehavior,
    (float)(10000) timeForSymmetrySearchBehavior,
    (float)(3000) timeForMovingSearchBehavior,
    (float)(3000) timeForSupporterSearchBehavior,
    (float)(600.f) maxDiveDistance,
    (float)(200.f) minBallMovementForPenaltyDive
  )
);


/**
* @class KeeperProvider
* Symbols for new role behavior 2019
*/

class KeeperProvider : public KeeperProviderBase, public RoleProvider<Keeper>
{

public:
  /** Constructor */
  KeeperProvider() {}

  /** Updates some of the symbols */
  void update(Keeper& positioningSymbols) override;

  /** Calculates the position the robot should target in ready state. */
  void getReadyPosition(Keeper& keeper);

private:
  void updateDecisionVariables(Keeper& keeper);
  void searchForBall(Keeper& keeper);
  void handleSetPlays(Keeper& keeper);
  void updateKeeper(Keeper& keeper);
  void calcOptPlayingPosition(Keeper& keeper);
  void updateBallchaserKeeper(Keeper& keeper);

  enum class KeeperState
  {
    wait,
    chaseBall,
    searchForBall,
    setPlay
  };

  KeeperState keeperState = KeeperState::wait;
  bool isSupported = false;
  bool supporterSeenBall = false;
  Vector2f ballPositionSupporter = Vector2f::Zero();
  //unsigned lastBlockTimeStamp = 0;
  bool kickLeft = false;
  unsigned lastInPrepareDive = 0;

private:
  void stateReady_kickOff_own(Keeper& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(Keeper& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(Keeper& positioningSymbols, bool left) override;
  float goalKick_opponent(Keeper& positioningSymbols, bool left) override;
  float pushingFreeKick_own(Keeper& positioningSymbols) override;
  float pushingFreeKick_opponent(Keeper& positioningSymbols) override;
  float cornerKick_own(Keeper& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(Keeper& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(Keeper& positioningSymbols, bool left) override;
  float kickIn_opponent(Keeper& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(Keeper& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(Keeper& positioningSymbols) override;
  void regularPlay(Keeper& positioningSymbols) override;
};
