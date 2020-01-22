/**
 * @file WhistleHandlerDortmund.h
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/WhistleDortmund.h"

MODULE(WhistleHandlerDortmund,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  USES(TeammateData),
  USES(RemoteBallModel),
  USES(BallModel),
  USES(RobotPose),
  REQUIRES(WhistleDortmund),
  PROVIDES(GameInfo),
  LOADS_PARAMETERS(
  {,
    (int) (1000) timeWindow,
    (int) (49) percentOfTeamAgrees,
    (bool) (true) useBallPosition,
    (float) (600.f) maxBallToMiddleDistance,
  }),
});

class WhistleHandlerDortmund : public WhistleHandlerDortmundBase
{
private:
  std::vector<unsigned> penaltyTimes;
  unsigned whistleTimestamps[MAX_NUM_PLAYERS+1];

  unsigned timeOfLastSetState = 0;
  unsigned lastGameState = STATE_INITIAL;
  bool overrideGameState = false;

  void update(GameInfo& gameInfo);
  bool checkWhistles();
  bool checkBall();
  bool checkForIllegalMotionPenalty();
};
