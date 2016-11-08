/**
 * @file WhistleHandlerDortmund.cpp
 */

#include "WhistleHandlerDortmund.h"
#include "Tools/Settings.h"

MAKE_MODULE(WhistleHandlerDortmund, motionInfrastructure)

void WhistleHandlerDortmund::update(GameInfo& gameInfo)
{
  bool whistleDetected = false;
  if (gameInfo.state == STATE_PLAYING && theRawGameInfo.state == STATE_SET)
    whistleDetected = true;
  gameInfo = theRawGameInfo;
  // no whistle detection in penalty shootout or drop in (Rules 2016)
  if (gameInfo.gameType == GAME_DROPIN ||
    Global::getSettings().gameMode == Settings::penaltyShootout ||
    Global::getSettings().gameMode == Settings::dropIn)
    return;
  if (whistleDetected)
    gameInfo.state = STATE_PLAYING;
  if (gameInfo.state != lastGameState && gameInfo.state == STATE_SET)
  {
    for (unsigned int i = 0; i < MAX_NUM_PLAYERS; i++)
      whistleTimestamps[i] = 0;
    timeOfLastSetState = theFrameInfo.time;
  }
    
  
  if (gameInfo.state != STATE_SET)
    overrideGameState = false;
  else if (!overrideGameState && gameInfo.secondaryState == STATE2_NORMAL)
    overrideGameState = checkWhistles();

  if(overrideGameState)
  {
    gameInfo.state = STATE_PLAYING;
  }
  lastGameState = gameInfo.state;

}

bool WhistleHandlerDortmund::checkWhistles()
{
  int numberWhistleDetected = theWhistleDortmund.detected ? 1 : 0;

  for (auto &mate : theTeammateData.teammates)
  {
    if (mate.whistle.detected)
      whistleTimestamps[mate.number] = theFrameInfo.time;
    if (mate.whistleCausedPlay)
      return true;
  }

  for (unsigned int i = 0; i < MAX_NUM_PLAYERS; i++)
    if (theFrameInfo.getTimeSince(whistleTimestamps[i]) < timeWindow)
      numberWhistleDetected++;

  return numberWhistleDetected * 100 / (theTeammateData.numberOfActiveTeammates + 1) > percentOfTeamAgrees;
}
