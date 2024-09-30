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
#include "Representations/Modeling/RobotPose.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "RoleProvider.h"

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

class DefenderLeftProvider : public DefenderLeftProviderBase, public RoleProvider<DefenderLeft>
{
public:
  /** Constructor */
  DefenderLeftProvider() {}

private:
  /** Updates some of the symbols */
  void update(DefenderLeft& positioningSymbols) override;

  void stateReady_kickOff_own(DefenderLeft& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(DefenderLeft& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(DefenderLeft& positioningSymbols, bool left) override;
  float goalKick_opponent(DefenderLeft& positioningSymbols, bool left) override;
  float pushingFreeKick_own(DefenderLeft& positioningSymbols) override;
  float pushingFreeKick_opponent(DefenderLeft& positioningSymbols) override;
  float cornerKick_own(DefenderLeft& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(DefenderLeft& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(DefenderLeft& positioningSymbols, bool left) override;
  float kickIn_opponent(DefenderLeft& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(DefenderLeft& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(DefenderLeft& positioningSymbols) override;
  void regularPlay(DefenderLeft& positioningSymbols) override;

  /** Calculation of the set play position. */
  bool calculateSetPlayPosition(DefenderLeft& positioningSymbols);
  void getStandardPosition(DefenderLeft& positioningSymbols);
  void getDefensivePosition(DefenderLeft& positioningSymbols);
  void avoidBallchaserConflict(DefenderLeft& positioningSymbols);

  void setPlayingOrSetThresholds(DefenderLeft& positioningSymbols);
};
