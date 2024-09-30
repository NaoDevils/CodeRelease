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
#include "Representations/Modeling/RobotPose.h"
#include "Utils/PositionUtils.h"
#include "Utils/ThresholdUtils.h"
#include "RoleProvider.h"

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

class DefenderRightProvider : public DefenderRightProviderBase, public RoleProvider<DefenderRight>
{
public:
  /** Constructor */
  DefenderRightProvider() {}

private:
  /** Updates some of the symbols */
  void update(DefenderRight& positioningSymbols) override;

  void stateReady_kickOff_own(DefenderRight& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(DefenderRight& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(DefenderRight& positioningSymbols, bool left) override;
  float goalKick_opponent(DefenderRight& positioningSymbols, bool left) override;
  float pushingFreeKick_own(DefenderRight& positioningSymbols) override;
  float pushingFreeKick_opponent(DefenderRight& positioningSymbols) override;
  float cornerKick_own(DefenderRight& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(DefenderRight& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(DefenderRight& positioningSymbols, bool left) override;
  float kickIn_opponent(DefenderRight& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(DefenderRight& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(DefenderRight& positioningSymbols) override;
  void regularPlay(DefenderRight& positioningSymbols) override;

  /** Calculation of the set play position. */
  bool calculateSetPlayPosition(DefenderRight& positioningSymbols);
  void getStandardPosition(DefenderRight& positioningSymbols);
  void getDefensivePosition(DefenderRight& positioningSymbols);
  void avoidBallchaserConflict(DefenderRight& positioningSymbols);

  void setPlayingOrSetThresholds(DefenderRight& positioningSymbols);
};
