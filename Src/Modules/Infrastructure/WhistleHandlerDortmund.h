/**
 * @file WhistleHandlerDortmund.h
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Module/Module.h"

MODULE(WhistleHandlerDortmund,
{,
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeammateData),
  REQUIRES(TeamBallModel),
  REQUIRES(WhistleDortmund),
  PROVIDES(GameInfo),
  DEFINES_PARAMETERS(
{,
    (int) (1000) timeWindow,
    (int) (49) percentOfTeamAgrees,
  }),
});

class WhistleHandlerDortmund : public WhistleHandlerDortmundBase
{
private:
  std::vector<unsigned> penaltyTimes;
  unsigned whistleTimestamps[MAX_NUM_PLAYERS];

  unsigned timeOfLastSetState = 0;
  unsigned lastGameState = STATE_INITIAL;
  bool overrideGameState = false;

  void update(GameInfo& gameInfo);
  bool checkWhistles();
};
