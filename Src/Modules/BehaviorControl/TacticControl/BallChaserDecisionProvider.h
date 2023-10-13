/**
* @file BallChaserDecisionProvider.h
*
* Declaration of class BallChaserDecisionProvider.
* Decides the best player number to follow the ball.
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/TacticSymbols.h"

STREAMABLE(TimeToBallParams,
  ENUM(TimeToBallCalculationType,
    distance,
    distanceAndBallBehind,
    path,
    distanceAndPosition
  ),
  (bool)(true) usePredictedBallPosition,
  (TimeToBallCalculationType)(distance) type,
  (float)(5.f) isBallMineHysteresisFactor,
  (float)(500.f) targetDistanceObstaclePenalty,
  (float)(1000.f) minimumDistanceForRotation,
  // normal penalties add distance and time penalties add time to timeToReachBall
  (float)(1000.f) fallDownPenalty,
  (float)(6.f) fallDownTimePenalty,
  (int)(4000) notSeenTime, // the time penalty for robots that do not see the ball
  (float)(1000.f) notSeenPenalty,
  (float)(2.f) notSeenTimePenalty,
  (float)(1000.f) ballBehindPenalty,
  (float)(0.f) obstaclePenalty,
  (float)(500.f) ballBehindRobotHysteresis, // time penalty for robot that has to walk around ball
  (float)(500.f) ballChaserBonus, // time bonus for ball chaser to stabilize decision
  (float)(1.f) ballChaserTimeBonus,
  (float)(600.f) distancePenaltyBallBehindRobot,
  (float)(4000.f) distancePenaltyBallBehindRobotFullDistance,
  (float)(1.f) targetDistanceRobotRotFactor,
  (float)(3.f) penaltyBallBehindFactorOwnGroundline,
  (float)(1.f) penaltyBallBehindFactorOpponentGroundline,
  (float)(1000.f) keeperPenaltyDuringOwnGoalKick
);

MODULE(BallChaserDecisionProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(WalkingEngineParams),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateData),
  REQUIRES(DangerMap),
  REQUIRES(RobotMap),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  USES(RoleSymbols),
  REQUIRES(TacticSymbols),
  PROVIDES(BallChaserDecision),
  LOADS_PARAMETERS(,
    (TimeToBallParams) timeToBallParams,
    (bool)(false) useLocalBallModelForDecision,
    (bool)(true) useLocalBallModelWhenNear,
    (float)(1000.f) maxBallDistanceForLocalDecision,
    (float)(750.f) maxBallDistanceForTeammateToBeNear
  )
);

/**
* @class RoleDynamicProvider
* Symbols for role decision
*/
class BallChaserDecisionProvider : public BallChaserDecisionProviderBase
{
public:
  /** Constructor */
  BallChaserDecisionProvider() = default;

private:
  /** Main update funtion, called every frame. */
  void update(BallChaserDecision& ballChaserDecision);

  void decideLocal(BallChaserDecision& ballChaserDecision);

  /** Update member variable isBallBehindPosition[]. */
  void updateBallBehindMe();

  /** Update member variable isBallBehindPosition[] for the numbers of all team mates. */
  void updateIsBallBehindPositionForMates(Vector2f localBallModelsPos);

  /** Chooses the coordinates used as the ball position when checking if the ball is behind a robot.*/
  Vector2f getBallPosForDecision(Vector2f currentBallPosition, Vector2f predictedBallPosition);

  /** Determines if the ball is located behind a robot. */
  bool isBallBehindRobot(int robotNumber, Vector2f robotPosition, Vector2f ballPosition);

  /** Checks if a position is located inside the own penalty area. */
  bool posInOwnPenaltyArea(Vector2f position, float boundaryModifier);

  std::tuple<int, int> getNewestTeamBallChaserNumber();

  /** Find player number who needs least time to ball. */
  int calcBallChaserNumber(BallChaserDecision& ballChaserDecision, bool useLocalBallModelForDecision);

  /** Calculates the distance to the ball for all teammates. */
  float getDistanceToBallForMate(const TeammateReceived& mate, bool useLocalBallModelForDecision, Vector2f localBallPosition);

  /** Calculates distance between a player to a position (here the ball position). */
  float calcDistanceToBall(const Pose2f& fromPose, const Pose2f& targetOnField, int playerNumber, bool wasBallChaser, bool upright, bool notSeen, unsigned timeSinceLastUpdate);

  /** Calculates a distance penalty based on the rotation difference between two poses. */
  float getRotationPenalty(const Pose2f& fromPose, const Pose2f& targetOnField);

  /** Calculates a distance penalty based on obstacles on the path between two poses. */
  float getObstaclePenalty(const Pose2f& fromPose, const Pose2f& targetOnField);

  /** Calculates a distance penalty based on the current situation of the robot. */
  float getStatusPenalty(const Pose2f& fromPose, const Pose2f& targetOnField, int playerNumber, bool wasBallChaser, bool upright, bool notSeen);

  // member variables
  // ----------------
  bool isBallBehindPosition[MAX_NUM_PLAYERS + 1];
  bool ballWasInPenaltyArea = false;
};
