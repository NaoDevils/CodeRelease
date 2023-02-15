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
  LOADS_PARAMETERS(,
    (int) (1000) timeWindow,
    (bool) (true) useBallPosition,
    (bool) (true) useBallForPlayingReadyTransition,
    (float) (600.f) maxBallToMiddleDistance,
    (int) (20000) playToReadyTimeout, // after this time, local decision will be reverted
    (int) (20000) setToPlayTimeout, // after this time, local decision will be reverted
    (int) (3000) maxTimediffWhistleToGoal
  )
);

class WhistleHandlerDortmund : public WhistleHandlerDortmundBase
{
private:
  std::vector<unsigned> penaltyTimes;
  unsigned whistleTimestamps[MAX_NUM_PLAYERS + 1];

  unsigned timeOfLastSetState = 0;
  unsigned lastGameState = STATE_INITIAL;
  unsigned timeStampPlayToReady = 0;
  unsigned timeStampSetToPlay = 0;

  void update(GameInfo& gameInfo);
  bool checkBall(); // returns true if the ball moved from the center location
  bool checkForGoal(); // returns true if the ball was detected near one of the goal lines
  bool checkForIllegalMotionPenalty();
};
