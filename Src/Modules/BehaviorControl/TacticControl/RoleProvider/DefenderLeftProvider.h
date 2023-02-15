/**
* @file DefenderLeftProvider.h
*
* Declaration of class DefenderLeftProvider.
* Provides positions for the DefenderLeft in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderLeft.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(DefenderLeftProvider,
  REQUIRES(BehaviorConfiguration),
  USES(Ballchaser),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  PROVIDES(DefenderLeft)
);


/**
* @class DefenderLeftProvider
* Symbols for new role behavior 2019
*/

class DefenderLeftProvider : public DefenderLeftProviderBase
{
public:
  /** Constructor */
  DefenderLeftProvider() {}

  /** Updates some of the symbols */
  void update(DefenderLeft& positioningSymbols);

private:
  /** Calculation of the set play position. */
  bool calculateSetPlayPosition(DefenderLeft& positioningSymbols);
  void getStandardPosition(DefenderLeft& positioningSymbols);
  void getDefensivePosition(DefenderLeft& positioningSymbols);
  void avoidBallchaserConflict(DefenderLeft& positioningSymbols);
};
