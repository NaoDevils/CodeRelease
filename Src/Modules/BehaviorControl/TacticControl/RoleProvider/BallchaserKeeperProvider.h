/**
* @file BallchaserKeeperProvider.h
*
* Declaration of class BallchaserKeeperProvider.
* Provides positions for the BallchaserKeeper in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/RoleSymbols/BallchaserKeeper.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(BallchaserKeeperProvider,
  REQUIRES(BallModel),
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GoalSymbols),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TacticSymbols),
  PROVIDES(BallchaserKeeper)
);


/**
* @class BallchaserKeeperProvider
* Symbols for new role behavior 2019
*/

class BallchaserKeeperProvider : public BallchaserKeeperProviderBase
{
public:
  /** Constructor */
  BallchaserKeeperProvider() {}

  /** Updates some of the symbols */
  void update(BallchaserKeeper& positioningSymbols);

private:
};
