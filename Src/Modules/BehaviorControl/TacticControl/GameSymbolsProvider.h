/** 
* @file GameSymbolsProvider.h
*
* Declaration of class GameSymbolsProvider.
*
* @author Judith Müller
*/

#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Module.h"

MODULE(GameSymbolsProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPoseAfterPreview),
  USES(RoleSymbols),
  REQUIRES(TeammateData),
  PROVIDES(GameSymbols)
);

/**
* @author Max Risler
* @author Judith Müller
*/
class GameSymbolsProvider : public GameSymbolsProviderBase
{
public:
  /*
  * Constructor.
  */
  GameSymbolsProvider() {}

  bool startup = true;
  unsigned char lastGameState = STATE_INITIAL;
  unsigned timeWhenChangedToPlayingState = 0;
  unsigned timeWhenGameStateChanged = 0;
  unsigned lastPenalizedTime = 0;
  int16_t lastSecsRemaining = 600;
  unsigned char lastSetPlay = 0;
  unsigned timeWhenSetPlayStarted = 0;
  unsigned timeWhenSetPlayFinished = 0;
  int timeOfLastBallout = -1;
  // The value of theGameInfo.kickingTeam of the last frame used to check if the kicking team changed
  int lastKickingTeam = -1;

  void update(GameSymbols& gameSymbols);
  bool calcAllowedInOwnGoalArea();

private:
  int counter = 0;
  Vector2f ballPositionField = Vector2f::Zero();

  unsigned char lastOwnScore = 0;
  unsigned timeSinceScoreChange = 0;
  bool ownTeamScoredGoal = false;
  Vector2f ballAtStart = Vector2f::Zero();
  float ballStartCounter = 0.f;

  void decideGameSituation(GameSymbols& theGameSymbols);
  int getSetPlay();
};
