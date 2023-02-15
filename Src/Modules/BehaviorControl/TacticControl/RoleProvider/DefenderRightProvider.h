/**
* @file DefenderRightProvider.h
*
* Declaration of class DefenderRightProvider.
* Provides positions for the DefenderRight in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderRight.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(DefenderRightProvider,
  REQUIRES(BehaviorConfiguration),
  USES(Ballchaser),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameSymbols),
  REQUIRES(GameInfo),
  PROVIDES(DefenderRight)
);


/**
* @class DefenderRightProvider
* Symbols for new role behavior 2019
*/

class DefenderRightProvider : public DefenderRightProviderBase
{
public:
  /** Constructor */
  DefenderRightProvider() {}

  /** Updates some of the symbols */
  void update(DefenderRight& positioningSymbols);

private:
  /** Calculation of the set play position. */
  bool calculateSetPlayPosition(DefenderRight& positioningSymbols);
  void getStandardPosition(DefenderRight& positioningSymbols);
  void getDefensivePosition(DefenderRight& positioningSymbols);
  void avoidBallchaserConflict(DefenderRight& positioningSymbols);
};
