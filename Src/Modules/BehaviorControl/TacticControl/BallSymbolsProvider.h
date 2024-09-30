/** 
* @file BallSymbolsProvider.h
*
* Declaration of class BallSymbolsProvider.
*
* @author Ingmar Schwarz
*/

#pragma once

#include <algorithm>
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/CameraMatrix.h"
MODULE(BallSymbolsProvider,
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(JointCalibration),
  REQUIRES(RemoteBallModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(MotionInfo),
  USES(GameSymbols),
  PROVIDES(BallSymbols),
  LOADS_PARAMETERS(,
    (int) timeSinceTeamSeenForLostSwitch,
    (int) timeSinceSeenForLostSwitch,
    (float) timeForBallWasSeen,
    (int)(500) ballPredictionTimeHorizon,
    (float)(200) distanceXForBallClose,
    (float)(100) distanceYForBallClose,
    (float)(270) distanceYForBallNear,
    (float)(1.f) timeNeededTillStartOfBlockingInSec
  )
);

class BallSymbolsProvider : public BallSymbolsProviderBase
{
public:
  /** Constructor. */
  BallSymbolsProvider() {}

  /** updates the symbols */
  void update(BallSymbols& ballSymbols);

private:
  BallSymbols localBallSymbols;
  BallModel currentlyUsedBallModel;
  int timeSinceBallCouldHaveBeenSeen = 0;
  unsigned int lastFrameTime = 0;
  bool lastBallProbablyCloseButNotSeen = false;

  bool ballIsRolling = false;

  unsigned timeWhenBallHitTriggered = 0;
  unsigned timeWhenBallBlockableTriggered = 0;

  void chooseBallModel(BallSymbols& ballSymbols, BallModel& currentlyUsedBallModel);
  void setPredictedBallPosition(BallSymbols& ballSymbols);

  bool calculateBallCouldHaveBeenSeen(BallSymbols& ballSymbols);
  bool calculateBallHitsMe(BallSymbols& ballSymbols);
  bool calculateBallBlockable(BallSymbols& ballSymbols);
  bool calculateObstacleBlockingBall(BallSymbols& ballSymbols);
  bool calculateBallProbablyCloseButNotSeen(BallSymbols& ballSymbols);

  float getYPosWhenBallReachesOwnYAxis();
  float getYPosWhenBallReachesOwnGroundLine(const BallSymbols& ballSymbols);
};
