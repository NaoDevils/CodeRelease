/**
* \file CenterProvider2022.h
* The file declares a class that containts data about the desired position of the center on the field.
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
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Utils/AvoidUtils.h"
#include "Utils/PositionUtils.h"
#include "Utils/TeamUtils.h"
#include "Utils/ThresholdUtils.h"
#include "Utils/FieldUtils.h"
#include "Utils/FrontUtils.h"
#include "RoleProvider.h"
#include <optional>

MODULE(CenterProvider2022,
    REQUIRES(Ballchaser), // for avoidance
    REQUIRES(BallchaserKeeper), // for avoidance
    REQUIRES(BallChaserDecision), // for avoidance
    REQUIRES(BallSymbols),
    REQUIRES(BehaviorConfiguration),
    REQUIRES(FieldDimensions),
    REQUIRES(GameInfo),
    REQUIRES(GameSymbols),
    REQUIRES(TacticSymbols),
    REQUIRES(TeammateData),
    REQUIRES(RoleSelection),
    PROVIDES(Center),
    REQUIRES(RobotPose),
    REQUIRES(RobotMap),
    LOADS_PARAMETERS(,
      (float)(2400.f) setPlayOppCornerKickDistanceX,
      (float)(1000.f) setPlayOppCornerKickDistanceY,
      (float)(2000.f) kickInSupportDistanceY,
      (float)(900.f) xDistanceToBallchaser,
      (float)(300.f) yDistanceToBallchaser,
      (float)(400.f) yDistanceToKeeper,

      (float)(-2000.f) defensiveBehaviourX,
      (float)(-150.f) defensiveBehaviourHyperesisX
    )
  );


/**
* @class CenterProvider2022
* Symbols for new role behavior 2019
*/

class CenterProvider2022 : public CenterProvider2022Base, public RoleProvider<Center>
{

public:
  /** Constructor */
  CenterProvider2022() {}

private:
  void update(Center& positioningSymbols) override;

  void stateReady_kickOff_own(Center& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(Center& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(Center& positioningSymbols, bool left) override;
  float goalKick_opponent(Center& positioningSymbols, bool left) override;
  float pushingFreeKick_own(Center& positioningSymbols) override;
  float pushingFreeKick_opponent(Center& positioningSymbols) override;
  float cornerKick_own(Center& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(Center& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(Center& positioningSymbols, bool left) override;
  float kickIn_opponent(Center& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(Center& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(Center& positioningSymbols) override;
  void regularPlay(Center& positioningSymbols) override;

  void setPassivePosition(Center& positioningSymbols, const Vector2f& ballPositionField);
  void setAgressivePlayingPosition(Center& positioningSymbols, const Vector2f& ballPositionField);
  void setDefensivePlayingPosition(Center& positioningSymbols, const Vector2f& ballPositionField);
  void updateBallIsLeft(const Vector2f& ballPosition);

  // member variables
  bool ballIsLeft = false;
};
