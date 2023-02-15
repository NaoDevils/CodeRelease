/**
* @file DefenderSingleProvider.h
*
* Declaration of class DefenderSingleProvider.
* Provides positions for the DefenderSingle in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/BallchaserKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderSingle.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Utils/AvoidUtils.h"
#include "Utils/PositionUtils.h"
#include "Utils/TeamUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/FrontUtils.h"
#include "RoleProvider.h"
#include <optional>

MODULE(DefenderSingleProvider,
  REQUIRES(Ballchaser), // for avoidance
  REQUIRES(BallchaserKeeper), // for avoidance
  REQUIRES(BallChaserDecision), // for avoidance
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(RoleSelection),
  REQUIRES(TacticSymbols),
  REQUIRES(TeammateData),
  PROVIDES(DefenderSingle),
  LOADS_PARAMETERS(,
      (float)(2400.f) setPlayOppCornerKickDistanceX,
      (float)(1000.f) setPlayOppCornerKickDistanceY,
      (float)(2000.f) kickInSupportDistanceY,
      (float)(1300.f) xDistanceToBallchaser,

      (float)(-2000.f) defensiveBehaviourX,
      (float)(-150.f) defensiveBehaviourHyperesisX
    )
);


/**
* @class DefenderSingleProvider
* Symbols for new role behavior 2019
*/

class DefenderSingleProvider : public DefenderSingleProviderBase, public RoleProvider<DefenderSingle>
{
public:
  /** Constructor */
  DefenderSingleProvider() {}

private:
  /** Updates some of the symbols */
  void update(DefenderSingle& positioningSymbols) override;

  void stateReady_kickOff_own(DefenderSingle& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(DefenderSingle& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(DefenderSingle& positioningSymbols, bool left) override;
  float goalKick_opponent(DefenderSingle& positioningSymbols, bool left) override;
  float pushingFreeKick_own(DefenderSingle& positioningSymbols) override;
  float pushingFreeKick_opponent(DefenderSingle& positioningSymbols) override;
  float cornerKick_own(DefenderSingle& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(DefenderSingle& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(DefenderSingle& positioningSymbols, bool left) override;
  float kickIn_opponent(DefenderSingle& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(DefenderSingle& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(DefenderSingle& positioningSymbols) override;
  void regularPlay(DefenderSingle& positioningSymbols) override;

  Vector2f coverShortAngleToGoal(bool usePredictedBallLocation, DefenderSingle& positioningSymbols);
  void guardPassOpportunities(DefenderSingle& positioningSymbols);
  Vector2f calculateSetPlayPosition(DefenderSingle& positioningSymbols);

  void setPlayingOrSetThresholds(DefenderSingle& positioningSymbols);
  void setNotPlayingOrSetThresholds(DefenderSingle& positioningSymbols);
  void handleGeneralSetPlay(DefenderSingle& positioningSymbolds);

  // member variables
  void updateBallIsLeft(const Vector2f& ballPosition);
  bool ballIsLeft = false; /**< Used to decide where the short angle of the goal is. */
  bool guardPass = false;
};
