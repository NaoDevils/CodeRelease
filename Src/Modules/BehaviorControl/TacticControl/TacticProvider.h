/**
* @file TacticProvider.h
*
* Declaration of class TacticProvider.
* A collection of small tactical decisions and information gathering.
*
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Settings.h"

MODULE(TacticProvider,
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BallChaserDecision),
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(DangerMap),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(GoalSymbols),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(RemoteBallModel),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(TeammateData),
  PROVIDES(TacticSymbols)
);

/**
* @class TacticProvider
* Collects information about the current tactical situation.s
*/
class TacticProvider : public TacticProviderBase
{
public:
  /** Constructor */
  TacticProvider()
  {
    lastKickoffWasOwn = false;
    lastOwnScore = 0;
    lastOpponentScore = 0;
  }

private:
  /** Updates some of the symbols */
  void update(TacticSymbols& tacticSymbols);

  /** vvv--- methods for tactical decisions ---vvv */
  void calcNumberOfActiveFieldPlayers(TacticSymbols& tacticSymbols);
  bool decideDefensiveBehavior();
  void getBallDirection();
  void getBallSide();
  bool decideKickoffDirection(TacticSymbols& tacticSymbols);

  enum class BallSide
  {
    front,
    center,
    back
  };

  enum class BallDirection
  {
    towardsEnemySide,
    towardsOwnSide
  };

  // vvv--- Attributes used to keep track of calculation results over several frames. ---vvv
  BallSide currentSide = BallSide::center;
  BallDirection currentDirection = BallDirection::towardsEnemySide;
  bool directionChanged = false;
  bool defensiveBehavior = false;

  bool lastKickoffWasOwn;
  int lastOwnScore;
  int lastOpponentScore;
};
