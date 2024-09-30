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
#include "Representations/Configuration/FieldDimensions.h"
MODULE(BallchaserKeeperProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(FieldDimensions),
  REQUIRES(GoalSymbols),
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
