#include "TacticProvider.h"

#include "Modules/BehaviorControl/TacticControl/RoleProvider/RoleProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PathUtils.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include <algorithm>
#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Functions/ProbabilityFunctions.h>

void TacticProvider::update(TacticSymbols& tacticSymbols)
{
  if (theGameInfo.state == STATE_READY) // Decide kickoff direction
  {
    tacticSymbols.kickoffToTheLeft = decideKickoffDirection(tacticSymbols);
  }
  // Sets numberOfActiveFieldPlayers of PositioningSymbols.
  calcNumberOfActiveFieldPlayers(tacticSymbols);
  // Decides between defensive/offensive behavior based on the direction in which the ball has moved last.
  tacticSymbols.defensiveBehavior = decideDefensiveBehavior();
  tacticSymbols.activity = decideActivity();
  decideFightForBall(tacticSymbols, theBallSymbols.ballPositionFieldPredicted, theRobotMap);
  decideDefensiveCone(tacticSymbols, theBallSymbols.ballPositionFieldPredicted, theFieldDimensions);
  updateDanger(tacticSymbols);
  tacticSymbols.keepRoleAssignment = theGameInfo.state == STATE_READY && theGameSymbols.timeSinceGameState > static_cast<int>(timeTillKeepRoleAssignmentInReady);

  switch (currentDirection)
  {
  case TacticProvider::BallDirection::towardsEnemySide:
    tacticSymbols.currentDirection = "towardsEnemySide";
    break;
  case TacticProvider::BallDirection::towardsOwnSide:
    tacticSymbols.currentDirection = "towardsOwnSide";
    break;
  default:
    break;
  }
  switch (currentSide)
  {
  case TacticProvider::BallSide::back:
    tacticSymbols.currentSide = "back";
    break;
  case TacticProvider::BallSide::centerback:
    tacticSymbols.currentSide = "centerback";
    break;
  case TacticProvider::BallSide::centerfront:
    tacticSymbols.currentSide = "centerfront";
    break;
  case TacticProvider::BallSide::front:
    tacticSymbols.currentSide = "front";
    break;
  default:
    break;
  }
}

/**
* \brief Determines how many field player (me + team mates) are active.
*
* The keeper is not included in the total. The result will be stored in the TacticSymbols'
* numberOfActiveFieldPlayers attribute instead of being returned directly.
*/
void TacticProvider::calcNumberOfActiveFieldPlayers(TacticSymbols& tacticSymbols)
{
  if (theTeammateData.commEnabled)
  {
    tacticSymbols.numberOfActiveFieldPlayers = (theRobotInfo.number == 1 || theRobotInfo.penalty != PENALTY_NONE) ? 0 : 1; // myself
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.status >= TeammateReceived::Status::ACTIVE && mate.playerNumber != 1)
        tacticSymbols.numberOfActiveFieldPlayers++;
    }
  }
  else if (theGameInfo.controllerConnected)
  {
    const auto isActive = [](const RoboCup::RobotInfo& info)
    {
      return info.penalty == PENALTY_NONE;
    };

    // Skip keeper (player == 0)
    tacticSymbols.numberOfActiveFieldPlayers = static_cast<unsigned>(std::count_if(std::begin(theOwnTeamInfo.players) + 1, std::begin(theOwnTeamInfo.players) + MAX_NUM_PLAYERS, isActive));
  }
  else
  {
    tacticSymbols.numberOfActiveFieldPlayers = MAX_NUM_PLAYERS - 1;
  }
}

/**
* \brief Decides between defensive and offensive behavior based on the direction in which the ball has moved last.
*
* \return True if the defensive behavior should be used, false otherwise.
*/
bool TacticProvider::decideDefensiveBehavior()
{
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.status != mate.INACTIVE)
    {
      if (mate.enforceOffensiveRoles)
      {
        defensiveBehavior = false;
        return defensiveBehavior;
      }
      if (mate.enforceDefensiveRoles)
      {
        defensiveBehavior = true;
        return defensiveBehavior;
      }
    }
  }

  const bool setPlayOwn = (theGameSymbols.ownKickOff && theGameSymbols.currentSetPlay != SET_PLAY_NONE);
  const bool setPlayOpponent = (!theGameSymbols.ownKickOff && theGameSymbols.currentSetPlay != SET_PLAY_NONE);
  if (setPlayOwn)
  {
    defensiveBehavior = false;
  }
  else if (setPlayOpponent)
  {
    defensiveBehavior = true;
  }
  else
  {
    getBallDirection();
    if (defensiveBehavior && currentDirection == TacticProvider::BallDirection::towardsEnemySide)
    {
      defensiveBehavior = false;
    }
    else if (!defensiveBehavior && currentDirection == TacticProvider::BallDirection::towardsOwnSide)
    {
      defensiveBehavior = true;
    }
  }
  return defensiveBehavior;
}

float TacticProvider::decideActivity()
{
  // Activity is high after kickOff and setPlays
  const bool kickOffOngoing = theGameSymbols.kickoffInProgress && theGameSymbols.timeSincePlayingState < 10000; // TODO KickOff time constant
  const bool setPlayOngoing = theGameInfo.setPlay != SET_PLAY_NONE;
  if (kickOffOngoing || setPlayOngoing)
  {
    activity = 1;
    return activity;
  }

  // Activity is high if ball direction changed
  if (lastDirection != currentDirection)
  {
    activity = 1.f;
    return activity;
  }

  // Activity is high if ball side changed
  if (lastSide != currentSide)
  {
    activity = 1.f;
    return activity;
  }

  // Else activity gets decreased
  const float ACTIVITY_LOSS = 0.001f;
  activity = std::max(activity - ACTIVITY_LOSS, 0.f);
  return activity;
}

void TacticProvider::getBallDirection()
{
  lastDirection = currentDirection;

  // compute previous and current location of the ball
  TacticProvider::BallSide previousSide = TacticProvider::currentSide;
  getBallSide();
  TacticProvider::BallSide newSide = TacticProvider::currentSide;

  // compare sides to detect movement
  if ((previousSide == TacticProvider::BallSide::back && (newSide == TacticProvider::BallSide::centerback || newSide == TacticProvider::BallSide::centerfront || newSide == TacticProvider::BallSide::front))
      || (previousSide == TacticProvider::BallSide::centerback && (newSide == TacticProvider::BallSide::centerfront || newSide == TacticProvider::BallSide::front))
      || (previousSide == TacticProvider::BallSide::centerfront && newSide == TacticProvider::BallSide::front))
  {
    // offensive direction
    currentDirection = TacticProvider::BallDirection::towardsEnemySide;
  }
  else if ((previousSide == TacticProvider::BallSide::front
               && (newSide == TacticProvider::BallSide::centerfront || newSide == TacticProvider::BallSide::centerback || newSide == TacticProvider::BallSide::back))
      || (previousSide == TacticProvider::BallSide::centerfront && (newSide == TacticProvider::BallSide::centerback || newSide == TacticProvider::BallSide::back))
      || (previousSide == TacticProvider::BallSide::centerback && newSide == TacticProvider::BallSide::back))
  {
    // defensive direction
    currentDirection = TacticProvider::BallDirection::towardsOwnSide;
  }
  else
  {
    // no change of direction
  }
}

void TacticProvider::getBallSide()
{
  lastSide = currentSide;

  Vector2f ballPosition = theBallSymbols.ballPositionField;

  if (ballPosition.x() < 0.52 * theFieldDimensions.xPosOwnGroundline)
  {
    currentSide = TacticProvider::BallSide::back;
  }
  else if (ballPosition.x() > 0.48 * theFieldDimensions.xPosOwnGroundline && ballPosition.x() < 0.03 * theFieldDimensions.xPosOwnGroundline)
  {
    currentSide = TacticProvider::BallSide::centerback;
  }
  else if (ballPosition.x() > 0.03 * theFieldDimensions.xPosOpponentGroundline && ballPosition.x() < 0.48 * theFieldDimensions.xPosOpponentGroundline)
  {
    currentSide = TacticProvider::BallSide::centerfront;
  }
  else if (ballPosition.x() > 0.52 * theFieldDimensions.xPosOpponentGroundline)
  {
    currentSide = TacticProvider::BallSide::front;
  }
}

/**
* \brief Decides kickoff direction depending on to which sides kickoff was most successful.
*/
bool TacticProvider::decideKickoffDirection(TacticSymbols& tacticSymbols)
{
  // If dynamic side switching is disabled for kickoff, use kickOffToTheLeftSide parameter.
  if (!theBehaviorConfiguration.behaviorParameters.useDynamicKickoffSideSwitching)
    return theBehaviorConfiguration.behaviorParameters.kickOffToTheLeftSide;
  if (theOwnTeamInfo.score == 0 && theOpponentTeamInfo.score == 0) // Initial kickoff
    return theBehaviorConfiguration.behaviorParameters.kickOffToTheLeftSide;
  if (lastKickoffWasOwn)
  {
    if (theOwnTeamInfo.score > lastOwnScore) // Kickoff was successful
    {
      if (tacticSymbols.kickoffToTheLeft)
        tacticSymbols.numberOfLeftOwnKickOffSuccess++;
      else
        tacticSymbols.numberOfRightOwnKickOffSuccess++;
    }
    else if (theOpponentTeamInfo.score > lastOpponentScore) // Kickoff failed
    {
      if (tacticSymbols.kickoffToTheLeft)
        tacticSymbols.numberOfLeftOwnKickOffSuccess--;
      else
        tacticSymbols.numberOfRightOwnKickOffSuccess--;
    }
  }
  lastOwnScore = theOwnTeamInfo.score;
  lastOpponentScore = theOpponentTeamInfo.score;
  lastKickoffWasOwn = theGameSymbols.ownKickOff;
  return (tacticSymbols.numberOfLeftOwnKickOffSuccess >= tacticSymbols.numberOfRightOwnKickOffSuccess);
}

void TacticProvider::decideFightForBall(TacticSymbols& tacticSymbols, const Vector2f& ballPosition, const RobotMap& theRobotMap)
{
  const float WHILE_KICKING_OPPONENT_TO_BALL_DISTANCE = 200.f;

  tacticSymbols.ballToRobotDistance = -1.f;
  tacticSymbols.ballToOpponentRobotDistance = -1.f;

  for (const auto& robot : theRobotMap.robots)
  {
    const float distance = Geometry::distance(robot.pose.translation, ballPosition);
    const bool isOpponentRobot = robot.robotType != RobotEstimate::RobotType::teammateRobot;

    if (tacticSymbols.ballToRobotDistance == -1.f || distance < tacticSymbols.ballToRobotDistance)
    {
      tacticSymbols.ballToRobotDistance = distance;
      tacticSymbols.closestToBallRobot = robot.pose;
    }
    if (isOpponentRobot && (tacticSymbols.ballToOpponentRobotDistance == -1.f || distance < tacticSymbols.ballToOpponentRobotDistance))
    {
      tacticSymbols.ballToOpponentRobotDistance = distance;
      tacticSymbols.closestToBallOpponentRobot = robot.pose;
    }
  }

  // update ball steal time
  if (tacticSymbols.hasClosestOpponentRobot())
  {
    const Vector2f opponentToBall = ballPosition - tacticSymbols.closestToBallOpponentRobot.translation;
    const Angle assumedOpponentKickAngle = opponentToBall.angle();
    const Vector2f assumedOpponentKickPosition = ballPosition - opponentToBall.normalized(WHILE_KICKING_OPPONENT_TO_BALL_DISTANCE);
    const Pose2f assumedOpponentKickPose = Pose2f(assumedOpponentKickAngle, assumedOpponentKickPosition);

    const Pose2f assumedOppCurrentPose = Pose2f(assumedOpponentKickAngle, tacticSymbols.closestToBallOpponentRobot.translation); // TODO use detected angle soon

    const float assumedOpponentKickTime = 2.f;
    const float assumedOppPathTime = PathUtils::getPathTime(assumedOppCurrentPose, assumedOpponentKickPose, ballPosition);
    tacticSymbols.untilOpponentStealsBallTime = assumedOpponentKickTime + assumedOppPathTime;
  }
  else
  {
    tacticSymbols.untilOpponentStealsBallTime = -1.f;
  }
}

void TacticProvider::updateDanger(TacticSymbols& tacticSymbols)
{
  if (theGameSymbols.gameSituation == GameSymbols::GameSituation::regularPlay)
  {
    if (isDanger(450.f, tacticSymbols.ballInDanger == TacticSymbols::Danger::HIGH))
    {
      tacticSymbols.ballInDanger = TacticSymbols::Danger::HIGH;
    }
    else if (isDanger(800.f, tacticSymbols.ballInDanger == TacticSymbols::Danger::MEDIUM))
    {
      tacticSymbols.ballInDanger = TacticSymbols::Danger::MEDIUM;
    }
    else if (isDanger(1200.f, tacticSymbols.ballInDanger == TacticSymbols::Danger::LOW))
    {
      tacticSymbols.ballInDanger = TacticSymbols::Danger::LOW;
    }
    else
    {
      tacticSymbols.ballInDanger = TacticSymbols::Danger::NONE;
    }
  }
  else
  {
    tacticSymbols.ballInDanger = TacticSymbols::Danger::IMPOSSIBLE;
  }
}

bool TacticProvider::isDanger(float dangerDistance, const bool hysteresis) const
{
  dangerDistance = dangerDistance * (hysteresis ? 1.2f : 1.f);
  for (const auto& robot : theRobotMap.robots)
  {
    if (robot.robotType != RobotEstimate::teammateRobot)
    {
      const float robotDistance = Geometry::distance(theBallSymbols.ballPositionFieldPredicted, robot.pose.translation);
      if (robotDistance < dangerDistance)
      {
        return true;
      }
    }
  }
  return false;
}

void TacticProvider::decideDefensiveCone(TacticSymbols& theTacticSymbols, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
{
  const float defenseWidth = 180_deg;

  ASSERT(0_deg < defenseWidth && defenseWidth < 360_deg);

  const Angle step = defenseWidth / 2;
  Angle leftAngle;
  Angle rightAngle;

  const bool ballOnOwnSide = ballPosition.x() < 0.f;
  if (ballOnOwnSide)
  {
    const Vector2f ownGoalCenter = FieldUtils::getOwnGoalCenter(theFieldDimensions);
    const Angle ownGoalCenterAngle = (ownGoalCenter - ballPosition).angle();
    leftAngle = Angle(ownGoalCenterAngle + step).normalize();
    rightAngle = Angle(ownGoalCenterAngle - step).normalize();

    const Vector2f ownLeftGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal};
    const Vector2f ownRightGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal};
    const Angle ownLeftGoalPostAngle = (ownLeftGoalPost - ballPosition).angle();
    const Angle ownRightGoalPostAngle = (ownRightGoalPost - ballPosition).angle();
    const auto [ll, lr] = MathUtils::getLeftAndRightAngle(leftAngle, ownRightGoalPostAngle);
    const auto [rl, rr] = MathUtils::getLeftAndRightAngle(rightAngle, ownLeftGoalPostAngle);
    leftAngle = ll;
    rightAngle = rr;
  }
  else
  {
    rightAngle = 180_deg - step;
    leftAngle = -180_deg + step;
  }

  theTacticSymbols.defensiveCone = {leftAngle, rightAngle};
}

MAKE_MODULE(TacticProvider, behaviorControl)
