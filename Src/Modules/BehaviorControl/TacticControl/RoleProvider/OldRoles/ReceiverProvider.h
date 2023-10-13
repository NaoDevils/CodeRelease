/**
* @file ReceiverProvider.h
*
* Declaration of class ReceiverProvider.
* Provides positions for the Receiver in PLAY and READY.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PositionUtils.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/HeatMapCollection.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"

MODULE(ReceiverProvider,
  REQUIRES(BallSymbols),
  REQUIRES(RobotMap),
  REQUIRES(HeatMapCollection),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TacticSymbols),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TeammateData),
  REQUIRES(BallChaserDecision),
  PROVIDES(Receiver),
  LOADS_PARAMETERS(,
    (float)(200.f) minXPosition,
    (float)(2000.f) ySupportDistanceToBallchaser,
    (float)(1500.f) xSupportDistanceToBallchaser,
    (float)(800.f) evadeDistanceToBallchaser,
    (float)(1350.f) wantedYSpaceBorderToBC,
    (bool)(true) useHeatMap,
    (float)(1.f) sidesHeatFactor,
    (float)(1.f) opponentsHeatFactor
  )
);


/**
* @class ReceiverProvider
* Symbols for new role behavior 2019
*/

class ReceiverProvider : public ReceiverProviderBase, public RoleProvider<Receiver>
{

public:
  /** Constructor */
  ReceiverProvider() {}

  Pose2f ballchaserPose = Pose2f();

  bool playDefensive = false;

  //The 3 receiver states: 1) Evade 2) Support right 3) Support Left
  bool supportLeft = false;
  bool evadeBallchaser = false;

private:
  /** Updates some of the symbols */
  void update(Receiver& positioningSymbols) override;

  void stateReady_kickOff_own(Receiver& positioningSymbols, const Vector2f& ballPosition) override;
  void stateReady_kickOff_opponent(Receiver& positioningSymbols, const Vector2f& ballPosition) override;
  float goalKick_own(Receiver& positioningSymbols, bool left) override;
  float goalKick_opponent(Receiver& positioningSymbols, bool left) override;
  float pushingFreeKick_own(Receiver& positioningSymbols) override;
  float pushingFreeKick_opponent(Receiver& positioningSymbols) override;
  float cornerKick_own(Receiver& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float cornerKick_opponent(Receiver& positioningSymbols, const Vector2f& cornerKickPosition, bool left) override;
  float kickIn_own(Receiver& positioningSymbols, bool left) override;
  float kickIn_opponent(Receiver& positioningSymbols, bool left) override;
  float stateReady_penaltyKick_own(Receiver& positioningSymbols) override;
  float stateReady_penaltyKick_opponent(Receiver& positioningSymbols) override;
  void regularPlay(Receiver& positioningSymbols) override;

  /** Update ballchaser data, i.e. pose and intended action. */
  void updateBallchaserData();

  /** Support ballchaser, in ReceiverState supportLeft or supportRight. */
  void calcSupportBallchaserPosition(Receiver& positioningSymbols);

  /** Calculates the position the robot should target in ready state. */
  void getReadyPosition(Receiver& positioningSymbols, bool ownKickOff);

  /** Ballchaser evasion, between supportLeft and supportRight.*/
  void calcEvadeBallchaserPosition(Receiver& positioningSymbols);

  /** Everything to do with set plays. */
  void handleSetPlay(Receiver& receiver);

  void checkHeat(Vector2f positionToTry, Receiver& positioningSymbols);


  // newest ball chaser data, keeps old data if ballchaser is not active
  BehaviorData::SoccerState ballchaserAction = BehaviorData::controlBall;
  Vector2f ballchaserKickTarget = Vector2f(2000.f, 0.f);
};
