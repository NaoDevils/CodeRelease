/**
* @file ReplacementKeeperProvider.h
*
* Declaration of class ReplacementKeeperProvider.
* Provides positions for the ReplacementKeeper in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/ReplacementKeeper.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(ReplacementKeeperProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(BallChaserDecision),
  REQUIRES(RobotInfo),
  USES(RoleSymbols), // This is not a perfect solution, but we need to know if we are allowed to go to the ball for the chase ball decision
  PROVIDES(ReplacementKeeper),
  LOADS_PARAMETERS(,
    (float)(250.f) minBallDistanceForBlock,
    (float)(1200.f) maxBallDistanceForBlock,
    (float)(300.f) yReachableWithBlock,
    (float)(200.f) distanceFromGroundLine,
    (float)(300.f) safeDistanceToGoalPost,
    (float)(3500.f) moveWithBallDistance,
    (float)(100.f) moveWithBallSpeed,
    (int)(8000) timeBetweenBlockMotions
  )
);


/**
* @class ReplacementKeeperProvider
* Symbols for new role behavior 2019
*/

class ReplacementKeeperProvider : public ReplacementKeeperProviderBase
{
public:
  /** Constructor */
  ReplacementKeeperProvider() {}

private:
  /** Updates some of the symbols */
  void update(ReplacementKeeper& positioningSymbols);

  void calcOptPosition(ReplacementKeeper& positioningSymbols);

  // members
  unsigned lastBlockTimeStamp = 0;
};
