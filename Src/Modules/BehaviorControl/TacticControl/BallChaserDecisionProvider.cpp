/**
* @file BallChaserDecisionProvider.cpp
*
* Implementation of class BallChaserDecisionProvider.
*/

#include "BallChaserDecisionProvider.h"
#include "BallChaserDecisionProvider.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Transformation.h"
#include <cmath>

/**
 * \brief Updates the information in the BallChaserDecision representation.
 * 
 * First the robot decides for itself which robot should chase the ball (= local decision). 
 * Afterwards it checks for decisions made by team mates and uses the decision made by the mate 
 * with the lowest playing number (= remote decision). By default all robots use the remote 
 * decision. In some cases however the decision might be outdated since communication is limited and
 * the decision could have been made several seconds ago. In those cases robots may ignore the 
 * remote decision and use the local decision instead.
*/
void BallChaserDecisionProvider::update(BallChaserDecision& ballChaserDecision)
{
  ballChaserDecision.dynamic = false;

  if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    ballChaserDecision.playerNumberToBall = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? theRobotInfo.number : 0;
    return;
  }

  // do not update decision when ball is lost
  if (theGameInfo.state == STATE_PLAYING && theBallSymbols.timeSinceLastSeenByTeam > 3000)
    return;

  if (!theTeammateData.commEnabled)
  {
    decideLocal(ballChaserDecision);
    return;
  }
  ballChaserDecision.dynamic = true;

  // Update member variables
  updateBallBehindMe();

  // fill ball chaser player number with newest team decision
  auto [teamBallChaserNumber, teamBallLastSeen] = getNewestTeamBallChaserNumber();
  ballChaserDecision.playerNumberToBall = teamBallChaserNumber;

  if (ballChaserDecision.playerNumberToBall == 0)
    ballChaserDecision.playerNumberToBall = theRobotInfo.number;

  // Keep ballchaser decision during ready
  if (theTacticSymbols.keepRoleAssignment)
    return;

  DEBUG_RESPONSE("module:BallChaserDecisionProvider:enable")
  {
    int localBallChaserNumber = calcBallChaserNumber(ballChaserDecision, true);
    /*
  * Ignore team decision if local decision is better. This is the case when:
  * 1. we are the local decision
  * 2. the goalie is currently not the team decision
  *    (removed, otherwise nobody will go to the ball if the goalie is penalized or cannot stand up)
  * 3. our ball percept is newer than the team's OR we are goalie
  */
    if (teamBallChaserNumber != localBallChaserNumber && localBallChaserNumber == theRobotInfo.number
        && (theBallSymbols.timeSinceLastSeen < theFrameInfo.getTimeSince(teamBallLastSeen) || theRobotInfo.number == 1))
    {
      ballChaserDecision.playerNumberToBall = localBallChaserNumber;
    }

    /*
  * if i am currently ballchaser, but my local decision is different
  * -> hand the ballchaser role to local decision number
  */
    if (teamBallChaserNumber == theRobotInfo.number && teamBallChaserNumber != localBallChaserNumber)
    {
      ballChaserDecision.playerNumberToBall = localBallChaserNumber;
    }
  }

  STREAMABLE(Overwrite,,
    (bool)(false) active,
    (bool)(true) isBallchaser
  );

  Overwrite overwrite;
  MODIFY("module:BallChaserDecisionProvider:overwrite", overwrite);
  if (overwrite.active)
    ballChaserDecision.playerNumberToBall = overwrite.isBallchaser ? theRobotInfo.number : (theRobotInfo.number % 2 == 0 ? theRobotInfo.number - 1 : theRobotInfo.number + 1);
}

void BallChaserDecisionProvider::decideLocal(BallChaserDecision& ballChaserDecision)
{
  if (theGameInfo.controllerConnected)
  {
    const auto isActive = [](const RoboCup::RobotInfo& robotInfo)
    {
      return robotInfo.penalty == PENALTY_NONE;
    };

    // Highest player number will be playerNumberToBall during initial/ready/set
    if (theGameInfo.inPreGame() || theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET)
    {
      for (unsigned char player = MAX_NUM_PLAYERS; player > 0; --player)
      {
        if (isActive(theOwnTeamInfo.players[player - 1]))
        {
          ballChaserDecision.playerNumberToBall = player;
          return;
        }
      }
    }
    // I will be playerNumberToBall if I am the only one
    else if (std::count_if(theOwnTeamInfo.players, theOwnTeamInfo.players + MAX_NUM_PLAYERS, isActive) == 1)
    {
      ballChaserDecision.playerNumberToBall = theRobotInfo.number;
      return;
    }
  }
  // If no game controller is connected in initial/ready/set, assume 7 robots are active and the highest number will be playerNumberToBall.
  else if (theGameInfo.inPreGame() || theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET)
  {
    ballChaserDecision.playerNumberToBall = MAX_NUM_PLAYERS;
    return;
  }

  // Go to ball if no teammate is near and close enough
  float ballDistance = theBallSymbols.ballPositionRelative.norm();
  const auto friendNearBall = [&](const RobotMapEntry& entry)
  {
    return entry.robotType == RobotEstimate::RobotType::teammateRobot && (entry.pose.translation - theBallSymbols.ballPositionField).norm() < maxBallDistanceForTeammateToBeNear;
  };
  const bool teammateNearBall = std::find_if(theRobotMap.robots.begin(), theRobotMap.robots.end(), friendNearBall) != theRobotMap.robots.end();

  if (!teammateNearBall && ballDistance < (ballChaserDecision.playerNumberToBall == theRobotInfo.number ? maxBallDistanceForLocalDecision + 500.f : maxBallDistanceForLocalDecision))
  {
    ballChaserDecision.playerNumberToBall = theRobotInfo.number;
  }
  else
  {
    ballChaserDecision.playerNumberToBall = 0;
  }
}

/**
 * \brief Determine whether the ball is behind the robot.
 * 
 * "Behind" in this case only refers to the x-component of the absolute field coordinates of ball
 * and robot.
 * The results are stored in the member variables of the BallChaserDecisionProvider.
*/
void BallChaserDecisionProvider::updateBallBehindMe()
{
  // in ready or set, decide as if ball is in center position, regardless of model
  // otherwise, use normal position or predicted position, depending on parameter
  Vector2f ballPositionField = getBallPosForDecision(theBallSymbols.ballPositionField, theBallSymbols.ballPositionFieldPredicted);
  // calculate isBallBehind for myself
  bool isBallBehindMe = theRoleSymbols.role != BehaviorData::RoleAssignment::keeper && isBallBehindRobot(theRobotInfo.number, theRobotPoseAfterPreview.translation, ballPositionField);
  isBallBehindPosition[theRobotInfo.number] = isBallBehindMe;
  // update isBallBehindMe[] for the player numbers of the team mates
  updateIsBallBehindPositionForMates(ballPositionField);
}

/**
 * \brief Fill isBallBehindMe for the other team mates.
 * 
 * \param localBallModelPos The coordinates used as the position of the local ball model.
 */
void BallChaserDecisionProvider::updateIsBallBehindPositionForMates(Vector2f localBallModelsPos)
{
  // The current robot pose is the basis for the ballchaser decision.
  // (Could be theoretical position to stablize decision?)
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.behaviorData.role != BehaviorData::keeper)
    {
      // Calculation of isBallBehindPosition analog to own calculation.
      Vector2f mateCurrentBallPos = mate.behaviorData.ballPositionField;
      // Passing mateCurrentBallPos twice here, if current or predicted position is used depends on sender
      Vector2f mateBallPosition = getBallPosForDecision(mateCurrentBallPos, mateCurrentBallPos);
      // useLocalBallModelForDecision == true => use my own ball model
      if (useLocalBallModelForDecision || (theGameInfo.state == STATE_READY && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK))
        mateBallPosition = localBallModelsPos;
      // calculate isBallBehind for mate using selected ball position as reference
      bool isBallBehindMate = isBallBehindRobot(mate.playerNumber, mate.robotPose.translation, mateBallPosition);
      isBallBehindPosition[mate.playerNumber] = isBallBehindMate;
    }
  }
  // the keeper never gets penalized if the ball is behind him
  isBallBehindPosition[1] = false;
}

/**
 * \brief Select which ball position should be used for the decisions made by this provider.
 * 
 * The selection depends on the current situation and the parameters. The possible coordinates are
 *    - The current position of the ball.
 *    - The predicted position of the ball.
 *    - The coordinates (0, 0) (aka the field center).
 * 
 * The field center is selected if the current state is not equal to playing so that we assume
 * the ball on the kick off point even if it is not placed there yet. In playing the parameters
 * determine whether the current or predicted position of the ball is used.
 * Since the decisions can be made for this robot (using the local ball model) and the
 * team mates (using their model) the positions used as current and predictedBallPosition need to
 * be passed as parameters.
 * 
 * \param currentBallPosition The coordinates used as the current position of the ball.
 * \param predictedBallPosition The coordinates used as the predicted position of the ball.
 * 
 * \return The coordinates which should be used as the ball position for the isBallBehind decision.
 */
Vector2f BallChaserDecisionProvider::getBallPosForDecision(Vector2f currentBallPosition, Vector2f predictedBallPosition)
{
  Vector2f selectedBallPosition = currentBallPosition;
  if (timeToBallParams.usePredictedBallPosition)
    selectedBallPosition = predictedBallPosition;
  if (theGameInfo.state == STATE_READY && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK)
    selectedBallPosition = Vector2f::Zero();
  return selectedBallPosition;
}

/**
 * \brief Checks if the ball is behind a given robot (on the absolute x-axis).
 * 
 * \param robotNumber The player number of the robot.
 * \param robotPosition The field position of the robot.
 * \param ballPosition The field position of the ball.
 */
bool BallChaserDecisionProvider::isBallBehindRobot(int robotNumber, Vector2f robotPosition, Vector2f ballPosition)
{
  // isBehindMeDistance acts as hysteresis for the switch of the isBallBehindMe boolean.
  // It is at minimum -250, since we dont want to think the ball is behind us while dribbling.
  float isBehindDistance = (isBallBehindPosition[robotNumber] ? 0.f : timeToBallParams.ballBehindRobotHysteresis);
  // We want to know if the robot would have to go around the ball, this is why we use ball to goal center as intended direction
  Pose2f ballPose((Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f) - ballPosition).angle(), ballPosition);
  Vector2f robotInBallPoseCoordinates = Transformation::fieldToRobot(ballPose, robotPosition);
  bool isBallBehind = robotInBallPoseCoordinates.x() > isBehindDistance;
  return isBallBehind;
}

/**
 * \brief Checks if a position is located inside the own penalty area.
 * 
 * A modifier can be added to the boundaries of the penalty area in order to manually increase or
 * decrease its size for certain situation (e.g. for the use with a hysteresis).
 * 
 * \param position The position which might be in the penalty area.
 * \param boundaryModifier Will be added to the x- and y- boundary of the penalty area to change
 *                           its size.
 * 
 * \return True if the positions x-value (absolute y-value) is smaller than the penalty area
 *          boundaries x-value (absolute y-value), else False.
 */
bool BallChaserDecisionProvider::posInOwnPenaltyArea(Vector2f position, float boundaryModifier)
{
  float xBoundary = theFieldDimensions.xPosOwnPenaltyArea + boundaryModifier;
  float yBoundary = theFieldDimensions.yPosLeftPenaltyArea + boundaryModifier;
  bool posInPenaltyAreaX = position.x() < xBoundary;
  bool posInPenaltyAreaY = std::abs(position.y()) < yBoundary;
  ballWasInPenaltyArea = posInPenaltyAreaX && posInPenaltyAreaY;
  return ballWasInPenaltyArea;
}

std::tuple<int, int> BallChaserDecisionProvider::getNewestTeamBallChaserNumber()
{
  const Teammate* newestEventMessage = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);
  if (!newestEventMessage)
    return {0, 0};

  return {newestEventMessage->behaviorData.playerNumberToBall, newestEventMessage->ballModel.timeWhenLastSeen};
}

/**
 * \brief Calculates the robot number of the robot that should be ballchaser.
 * 
 * Decision is based on the distance between the robot and the ball. Once the number is decided, 
 * the decision needs to be stable.
 * 
 * \param ballChaserDecision A copy of the current ballChaserDecision
 * \param useLocalBall Whether to use the robots own ball model when calculating for teammates.
 */
int BallChaserDecisionProvider::calcBallChaserNumber(BallChaserDecision& ballChaserDecision, bool useLocalBall)
{
  // in ready or set, decide as if ball is in center position, regardless of model
  // otherwise, use normal position or predicted position, depending on parameter
  Vector2f ballPositionField = getBallPosForDecision(theBallSymbols.ballPositionField, theBallSymbols.ballPositionFieldPredicted);
  // If the robots role is ballchaser, it had the ball last.
  bool wasBallMine = theRobotInfo.number == ballChaserDecision.playerNumberToBall;
  // Time to reach ball for each robot number.
  std::vector<float> timesToReachBall(MAX_NUM_PLAYERS + 1, std::numeric_limits<float>::infinity());
  float penaltyAreaModifier = (ballWasInPenaltyArea ? 350.f : 150.f);
  ballWasInPenaltyArea = posInOwnPenaltyArea(ballPositionField, penaltyAreaModifier);

  ballChaserDecision.ownTimeToBall = calcDistanceToBall(theRobotPoseAfterPreview,
      Pose2f(ballPositionField),
      theRobotInfo.number,
      wasBallMine,
      theFallDownState.state == FallDownState::upright,
      theBallSymbols.timeSinceLastSeen > timeToBallParams.notSeenTime,
      0);

  // The keeper is not allowed to chase the ball outside of the penalty area
  if (theRobotInfo.penalty == PENALTY_NONE && (theRobotInfo.number != 1 || ballWasInPenaltyArea))
    timesToReachBall[theRobotInfo.number] = ballChaserDecision.ownTimeToBall;

  for (auto& mate : theTeammateData.teammates)
  {
    timesToReachBall[mate.playerNumber] = getDistanceToBallForMate(mate, useLocalBall, ballPositionField);
  }
  int closestPlayerNumber = int(std::min_element(timesToReachBall.begin(), timesToReachBall.end()) - timesToReachBall.begin());

  // closestPlayerNumber == 0 ==> no player was closest, i am per default
  // this can e.g. happen if i am not upright
  if (closestPlayerNumber == 0)
    return theRobotInfo.number;
  else
    return closestPlayerNumber;
}


/**
 * \brief Calculates the distances between teammates and the ball.
 * 
 * The result will be stored in an array which is passed as a reference. The array needs to have
 * a length equal to the number of players in the own team + 1. The indices of the array
 * correspond to the player number of the team mates. This means the distance for the teammate 
 * with the player number n will be stored in the n-th element.
 */
float BallChaserDecisionProvider::getDistanceToBallForMate(const TeammateReceived& mate, bool useLocalBall, Vector2f localBallPosition)
{
  bool wasMateBallChaser = std::get<0>(getNewestTeamBallChaserNumber()) == mate.playerNumber;
  // Choose the ball position on which the decision for the team mate is based.
  // Passing ballPositionField twice here, if current or predicted position is used depends on sender
  Vector2f mateBallPosition = getBallPosForDecision(mate.behaviorData.ballPositionField, mate.behaviorData.ballPositionField);
  if (useLocalBall)
    mateBallPosition = localBallPosition;

  // TODO: what about replacement keeper?
  float matePenaltyAreaModifier = (wasMateBallChaser ? 350.f : 150.f);
  bool mateBallInPenaltyArea = posInOwnPenaltyArea(mateBallPosition, matePenaltyAreaModifier);

  const int timeSinceBallSeen = static_cast<int>(mate.sendTimestamp - mate.ballModel.timeWhenLastSeen);
  const bool ballNotSeen = theFrameInfo.getTimeSince(mate.sendTimestamp) < timeToBallParams.notSeenTime && timeSinceBallSeen > timeToBallParams.notSeenTime;
  float mateDistance = calcDistanceToBall(mate.robotPose, Pose2f(mateBallPosition), mate.playerNumber, wasMateBallChaser, !mate.fallen, ballNotSeen, theFrameInfo.getTimeSince(mate.sendTimestamp));

  bool mateIsValid = mate.status >= TeammateReceived::ACTIVE && (mate.playerNumber != 1 || mateBallInPenaltyArea);
  return mateIsValid ? mateDistance : std::numeric_limits<float>::max();
}


/**
 * \brief Calculates distance between a player to a position (here the ball position).
 *
 * How this is done is controlled via TimeToBallParameters.
 * 
 * \param fromPose The pose used as the starting poing.
 * \param targetOnField The pose used as the end point.
 * \param playerNumber The player number of the robot.
 * \param wasBallchaser Whether the robot was the ballchaser in the last frame.
 * \param upright Wether the robot is in an upright position.
 * \param notSeen Wether the robot is not able to see the ball itself.
 * 
 * \return The distance between the from pose and the target pose.
*/
float BallChaserDecisionProvider::calcDistanceToBall(const Pose2f& fromPose, const Pose2f& targetOnField, int playerNumber, bool wasBallChaser, bool upright, bool notSeen, unsigned timeSinceLastUpdate)
{
  // The keeper should never be the ballchaser in ready when other options are available.
  // Otherwise, we cannot replace a specific role by the ballchaser in RoleDynamicProvider.
  if ((theGameInfo.state == STATE_READY || theGameInfo.inPreGame()) && playerNumber == 1)
    return std::numeric_limits<float>::max(); // We cannot use infinity here. Otherwise, the robot will not be considered as a last choice.

  const Vector2f targetRelative = Transformation::fieldToRobot(fromPose, targetOnField.translation);
  float timeToBall = targetRelative.norm();

  // Assume the ballchaser goes to the ball and updates its position only every eventConfig.playerMovedEventDistanceForBallchaser (currently 500mm)
  if (wasBallChaser && upright && !notSeen)
  {
    // Assume the robot walks at about 80% speed on average
    float walkDistanceSinceUpdate = 0.8f * theWalkingEngineParams.speedLimits.xForward * timeSinceLastUpdate / 1000.f;

    // The robot cannot move more than 500mm without sending a message
    walkDistanceSinceUpdate = std::min(500.f, walkDistanceSinceUpdate);

    // The robot cannot come closer than 200mm to the ball
    timeToBall = std::max(200.f, timeToBall - walkDistanceSinceUpdate);
  }

  switch (timeToBallParams.type)
  {
  case TimeToBallParams::distanceAndPosition:
  {
    // initial : euclidian distance
    // adding penalty for wrong rotation towards target
    timeToBall += getRotationPenalty(fromPose, targetOnField);
    // obstacles through dangermap
    timeToBall += getObstaclePenalty(fromPose, targetOnField);
    break;
  }
  default:
    break;
    // euclidian distance only, nothing to do here
  }

  // add penalties based on the robots current situation.
  timeToBall += getStatusPenalty(fromPose, targetOnField, playerNumber, wasBallChaser, upright, notSeen);

  // Keeper should stay in the goal if other players are near
  if (playerNumber == 1 && theGameInfo.setPlay == SET_PLAY_GOAL_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
    timeToBall += timeToBallParams.keeperPenaltyDuringOwnGoalKick;

  return timeToBall;
}

/**
 * \brief Calculates a distance penalty based on the rotation difference between two poses.
 * 
 * This method is a helper function used in calcDistanceToBall.
 * The penalty is based on the total amount of rotation a robot would need to get from the
 * starting to the end point. This includes the rotation distance between the start and the path
 * as well as the rotation difference between the path and the end.
 * 
 * \param fromPose The pose used as the starting poing.
 * \param targetOnField The pose used as the end point.
 * 
 * \return The penalty that should be added to the distance between robot and ball.
 */
float BallChaserDecisionProvider::getRotationPenalty(const Pose2f& fromPose, const Pose2f& targetOnField)
{
  const Vector2f targetRelative = Transformation::fieldToRobot(fromPose, targetOnField.translation);
  float euclidianDistance = targetRelative.norm();
  // adding penalty for wrong rotation towards target - USEFUL? thinking about situations near target
  Vector2f pathVector = targetOnField.translation - fromPose.translation;
  float pathAngle = toDegrees(std::abs(Angle::normalize(pathVector.angle() - fromPose.rotation)));
  float targetRotationRelative = toDegrees(std::abs(Angle::normalize(targetOnField.rotation - pathVector.angle())));
  float isBallCloseFactor = 1.f;
  isBallCloseFactor = euclidianDistance < timeToBallParams.minimumDistanceForRotation ? 0.f : 1.f;
  float addedDistance = (pathAngle + targetRotationRelative) * timeToBallParams.targetDistanceRobotRotFactor * isBallCloseFactor;
  return addedDistance;
}

/**
 * \brief Calculates a distance penalty based on the obstacles between two poses.
 * 
 * This method is a helper function used in calcDistanceToBall.
 * The penalty is has a fixed value and is applied if there are obstacles between the starting and
 * end point. The danger map is used to check whether obstacles are present.
 * 
 * \param fromPose The pose used as the starting poing.
 * \param targetOnField The pose used as the end point.
 * 
 * \return The penalty that should be added to the distance between robot and ball.
 */
float BallChaserDecisionProvider::getObstaclePenalty(const Pose2f& fromPose, const Pose2f& targetOnField)
{
  const Vector2f targetRelative = Transformation::fieldToRobot(fromPose, targetOnField.translation);
  float euclidianDistance = targetRelative.norm();
  float dangerSum = 0;
  float stepSizeF = (float)DangerMap::stepSize;
  for (float i = 0; i * stepSizeF <= euclidianDistance; i++)
  {
    Vector2f toCell = (targetRelative / targetRelative.norm()) * i * stepSizeF;
    Vector2f toCellField = Transformation::robotToField(fromPose, toCell);
    dangerSum += theDangerMap.getDangerAt(toCellField, theFieldDimensions, (stepSizeF / 2));
  }
  return (dangerSum > 1.f ? timeToBallParams.targetDistanceObstaclePenalty : 0.f);
}

/**
 * \brief Calculates a distance penalty based on the robots current situation.
 * 
 * This method is a helper function used in calcDistanceToBall.
 * The penalty is based on different aspects. For example the robot gets a bonus if it was the
 * ballchaser before and a penalty if it is currently not in an upright position. The different
 * penalties and bonuses will be summed up and returned as one penalty.
 * 
 * \param fromPose The pose used as the starting poing.
 * \param targetOnField The pose used as the end point.
 * \param playerNumber The player number of the robot.
 * \param wasBallchaser Whether the robot was the ballchaser in the last frame.
 * \param upright Wether the robot is in an upright position.
 * \param notSeen Wether the robot is not able to see the ball itself.
 * 
 * \return The penalty that should be added to the distance between robot and ball.
 */
float BallChaserDecisionProvider::getStatusPenalty(const Pose2f& fromPose, const Pose2f& targetOnField, int playerNumber, bool wasBallChaser, bool upright, bool notSeen)
{
  const float cameraHeight = 500.f;
  const Vector2f targetRelative = Transformation::fieldToRobot(fromPose, targetOnField.translation);
  const float distanceHysteresis = std::max(100.f, toDegrees(std::atan2(targetRelative.norm(), cameraHeight)) * timeToBallParams.isBallMineHysteresisFactor);

  float penalty = 0;
  // less penalty for behind ball if closer to ball
  const float minPenalty = timeToBallParams.distancePenaltyBallBehindRobotFullDistance / 4;

  float specificBehindPenalty = timeToBallParams.distancePenaltyBallBehindRobot * std::min(1.f, (minPenalty + targetRelative.norm()) / timeToBallParams.distancePenaltyBallBehindRobotFullDistance);

  float xPosFactor = (targetOnField.translation.x() - theFieldDimensions.xPosOwnGroundline) / (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOwnGroundline);
  float behindBallPenaltyFactor = timeToBallParams.penaltyBallBehindFactorOwnGroundline
      + (timeToBallParams.penaltyBallBehindFactorOpponentGroundline - timeToBallParams.penaltyBallBehindFactorOwnGroundline) * xPosFactor;
  ;

  penalty += (isBallBehindPosition[playerNumber] ? specificBehindPenalty * behindBallPenaltyFactor : 0.f) - (wasBallChaser ? distanceHysteresis : 0.f);

  if (!upright && notSeen)
    penalty += std::max(timeToBallParams.fallDownPenalty, timeToBallParams.notSeenPenalty); // penalty
  else
  {
    penalty += upright ? 0.f : timeToBallParams.fallDownPenalty;
    penalty += notSeen ? timeToBallParams.notSeenPenalty : 0.f;
  }
  return penalty;
}

MAKE_MODULE(BallChaserDecisionProvider, behaviorControl)
