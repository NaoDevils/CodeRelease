/**
* @file KeeperProvider.cpp
*
* Implementation of class KeeperProvider.
*
*/

#include "KeeperProvider.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Math/Geometry.h"

void KeeperProvider::update(Keeper& keeper)
{
  updateDecisionVariables(keeper);
  decide(keeper, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void KeeperProvider::stateReady_kickOff_own(Keeper& positioningSymbols, const Vector2f& ballPosition)
{
  getReadyPosition(positioningSymbols); //TODO: What if this method is calld in state playing
}

void KeeperProvider::stateReady_kickOff_opponent(Keeper& positioningSymbols, const Vector2f& ballPosition)
{
  stateReady_kickOff_own(positioningSymbols, ballPosition);
}

float KeeperProvider::goalKick_own(Keeper& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::goalKick_opponent(Keeper& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::pushingFreeKick_own(Keeper& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::pushingFreeKick_opponent(Keeper& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::cornerKick_own(Keeper& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::cornerKick_opponent(Keeper& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::kickIn_own(Keeper& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::kickIn_opponent(Keeper& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::stateReady_penaltyKick_own(Keeper& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

float KeeperProvider::stateReady_penaltyKick_opponent(Keeper& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 0.f;
}

void KeeperProvider::regularPlay(Keeper& keeper)
{
  //Ball wasn't seen in at least timeUntilSearchBehavior ms
  if (theBallSymbols.timeSinceLastSeenByTeam > timeUntilSearchBehavior && theBallSymbols.timeSinceLastSeenByTeam < 20000)
  {
    keeperState = KeeperState::searchForBall;
    keeper.ballSearchState = Keeper::KeeperBallSearchState::wait; // default state
    searchForBall(keeper);
  }
  else if (theBallChaserDecision.playerNumberToBall == 1 && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK && theBallSymbols.ballInOwnPenaltyArea
      && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && theBallSymbols.timeSinceLastSeen < 3000)
  {
    keeperState = KeeperState::chaseBall;
    keeper.isBallchaser = true;
    updateBallchaserKeeper(keeper);
  }
  else if (theGameInfo.setPlay != SET_PLAY_NONE || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    keeperState = KeeperState::setPlay;
    handleSetPlays(keeper);
  }
  else
  {
    keeperState = KeeperState::wait;
    updateKeeper(keeper);
  }
}

void KeeperProvider::getReadyPosition(Keeper& keeper)
{
  keeperState = KeeperState::wait;
  keeper.catchBall = false;
  keeper.stopAtTarget = true;
  keeper.optPosition = Pose2f(0_deg, theFieldDimensions.xPosOwnGroundline + distanceFromGroundLine, theFieldDimensions.yPosCenterGoal);
  if (theGameSymbols.currentSetPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    keeper.optPosition.translation.x() = theFieldDimensions.xPosOwnGroundline;
    keeper.thresholdRotation = 5_deg;
    keeper.thresholdXBack = 40;
    keeper.thresholdXFront = 40;
    keeper.thresholdY = 50;
  }
  else
  {
    keeper.thresholdXBack = 50.f;
    keeper.thresholdXFront = 100.f;
    keeper.thresholdY = 100.f;
    keeper.thresholdRotation = 10_deg;
    keeper.stopAtTarget = true;
    keeper.previewArrival = true;
  }
}


void KeeperProvider::updateDecisionVariables(Keeper& keeper)
{
  keeper.ballSearchState = Keeper::KeeperBallSearchState::none;
  keeper.isBallchaser = false;
  keeper.catchBall = false;

  // when keeper becomes ballchaser and starts walking/standing, it may still be in prepare dive pose
  if (theMotionInfo.motion == MotionRequest::Motion::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::SpecialActionID::penaltyGoaliePrepareDive)
    lastInPrepareDive = theFrameInfo.time;
  keeper.isInPrepareDive = theFrameInfo.getTimeSince(lastInPrepareDive) < 350;

  //**** CALCULATE isSupported & ballPositionSupporter
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.status == TeammateReceived::Status::FULLY_ACTIVE
        && HelperFunctions::calcDistanceModified(theBehaviorConfiguration, theRobotMap, true, mate.robotPose, Pose2f(0, theBallSymbols.ballPositionField)) < 1500)
    {
      isSupported = true;
      if (theFrameInfo.getTimeSince(mate.ballModel.timeWhenLastSeen) < 3000)
      {
        ballPositionSupporter = Transformation::robotToField(mate.robotPose, mate.ballModel.position);
        supporterSeenBall = true;
      }
      else
      {
        supporterSeenBall = false;
      }
    }
  }

  //calculate if keeper is upright
  keeper.prepared = theFallDownState.state == FallDownState::upright;

  //**** CALCULATE catchBall
  //Is true if the goalie should catch the ball (dive/wideStance)
  Vector2f vecRobotToBall = theBallSymbols.ballPositionField - theRobotPoseAfterPreview.translation;
  float nearestBallDistance = Geometry::getDistanceToLine(Geometry::Line(theBallSymbols.ballPositionRelative, theBallSymbols.ballVelocityRelative), Vector2f::Zero());
  keeper.catchBall = /*theMotionSelection.ratios[MotionRequest::walk] == 1.f //dont trigger while in special action (dive/standup/..)
    && */
      theRobotPoseAfterPreview.translation.x() < (theFieldDimensions.xPosOwnGoalArea + 100) // only in goal area
      && std::abs(theRobotPoseAfterPreview.translation.y()) < theFieldDimensions.yPosLeftGoalArea - 100 // only in goal area
      && theBallSymbols.timeSinceLastSeen < 3000 //ball should be seen in the last 3 seconds
      && theBallSymbols.ballPositionFieldPredicted.x() < theRobotPoseAfterPreview.translation.x() // must stop behind robot
      && std::abs(nearestBallDistance) < maxDiveDistance
      && (std::abs(theBallSymbols.yPosWhenBallReachesGroundLine) < theFieldDimensions.yPosLeftGoal + 100.f
          || (std::abs(vecRobotToBall.angle()) > 70_deg && std::abs(theBallSymbols.yPosWhenBallReachesGroundLine) < theFieldDimensions.yPosLeftGoal + 1000.f)) // within goal posts
      && theBallSymbols.ballVelocityRelative.norm() > moveWithBallSpeed; // do not dive for slow ball

  //**** CALCULATE timeOfLastDive
  if (theMotionInfo.inBlockMotion())
  {
    keeper.timeOfLastDive = theFrameInfo.time;
  }
}

void KeeperProvider::searchForBall(Keeper& keeper)
{
  //If the ball was last seen in the penalty area, the search approach needs to be much more aggressive
  if (theBallSymbols.ballInOwnPenaltyArea && theBallSymbols.timeSinceLastSeenByTeam > timeUntilSearchBehavior && theBallSymbols.timeSinceLastSeenByTeam < 15000)
    keeper.ballSearchState = Keeper::KeeperBallSearchState::penalty;
  //After at least 10s assume location loss
  else if (theBallSymbols.timeSinceLastSeenByTeam < timeUntilSearchBehavior && theBallSymbols.timeSinceLastSeenByTeam > timeForSymmetrySearchBehavior)
    keeper.ballSearchState = Keeper::KeeperBallSearchState::locate;
  //Ball is seen by the supporter, but not the goalie. Indication for location loss
  //Try this only for 5s
  else if (isSupported && supporterSeenBall && theBallSymbols.timeSinceLastSeenByTeam > (timeUntilSearchBehavior + timeForSearchBehavior)
      && theBallSymbols.timeSinceLastSeenByTeam < (timeUntilSearchBehavior + timeForSearchBehavior + timeForSupporterSearchBehavior))
    keeper.ballSearchState = Keeper::KeeperBallSearchState::supporter;

  // If the ball was last seen in own half but wasn't seen after all, then try to rotate a little to the right and left
  // If team comm is down, do this without the ballPosition check
  if (keeper.ballSearchState != Keeper::KeeperBallSearchState::locate && keeper.ballSearchState != Keeper::KeeperBallSearchState::penalty
      && (theBallSymbols.ballPositionField.x() < borderForOwnHalfBehavior || !theTeammateData.commEnabled)
      && ((keeper.ballSearchState != Keeper::KeeperBallSearchState::supporter && theBallSymbols.timeSinceLastSeenByTeam > (timeUntilSearchBehavior + timeForSearchBehavior))
          || theBallSymbols.timeSinceLastSeenByTeam > (timeUntilSearchBehavior + timeForSearchBehavior + timeForMovingSearchBehavior)))
    keeper.ballSearchState = Keeper::KeeperBallSearchState::movingBall;

  float maxXDistanceFromGroundline = 100.f;
  switch (keeper.ballSearchState)
  {
  case Keeper::KeeperBallSearchState::supporter:
  {
    Pose2f optSearchingPosition = Pose2f(0, theFieldDimensions.xPosOwnGroundline + maxXDistanceFromGroundline, ballPositionSupporter.y() / 3000 * 600);
    optSearchingPosition.rotation = (ballPositionSupporter - keeper.optPosition.translation).angle();
    keeper.optPosition = optSearchingPosition;
    break;
  }
  default:
  {
    if (theBallSymbols.timeSinceLastSeen >= 3000 && theBallSymbols.timeSinceLastSeen < 7000)
    {
      //If the ball wasn't seen in the last 3 seconds, the goalie should keep it's yPosition, as long as it was inside the penalty area
      float yPosBlockGoal = sgn(theRobotPoseAfterPreview.translation.y()) * std::min(std::abs(theRobotPoseAfterPreview.translation.y()), std::abs(theFieldDimensions.yPosLeftGoal));
      keeper.optPosition = Pose2f(0, theFieldDimensions.xPosOwnGroundline + maxXDistanceFromGroundline, yPosBlockGoal);
    }
    else
    {
      keeper.optPosition = Pose2f(0, theFieldDimensions.xPosOwnGroundline + maxXDistanceFromGroundline, 0);
    }
    break;
  }
  }
}

void KeeperProvider::handleSetPlays(Keeper& keeper)
{
  if ((theGameSymbols.currentSetPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) && !theGameSymbols.ownKickOff)
  {
    // according to 2020/21 rules keeper has to touch the ground line
    keeper.optPosition = Pose2f(0_deg, theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal);
    keeper.stopAtTarget = true;
    keeper.thresholdRotation = 5_deg;
    keeper.thresholdXBack = 50.f;
    keeper.thresholdXFront = 50.f;
    // Last percept is too sensitive, especially when going to prepareDive.
    //float distanceToPenaltyMark =
    //  (Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosCenterGoal) - theRobotPoseAfterPreview.translation).norm();
    //keeper.catchBall = theGameInfo.state == STATE_PLAYING
    //  && theBallModelAfterPreview.lastPerception.x() < distanceToPenaltyMark - minBallMovementForPenaltyDive;
  }
  else // for now default behavior
    updateKeeper(keeper);
}

void KeeperProvider::updateKeeper(Keeper& keeper)
{
  calcOptPlayingPosition(keeper);
  // ball is far away and not moving, up thresholds to keep robot still
  if ((theBallSymbols.ballVelocityRelative.norm() < moveWithBallSpeed && theBallSymbols.ballPositionRelative.norm() > moveWithBallDistance)
      || theBehaviorConfiguration.behaviorParameters.goalieForEvents)
  {
    keeper.thresholdXBack = 100.f;
    keeper.thresholdXFront = 100.f;
    keeper.thresholdY = 200.f;
    keeper.thresholdRotation = 15_deg;
    keeper.stopAtTarget = true;
    keeper.previewArrival = true;
  }
  else
  {
    keeper.thresholdXBack = 75.f;
    keeper.thresholdXFront = 100.f;
    keeper.thresholdY = 150.f;
    keeper.thresholdRotation = 15_deg;
    keeper.stopAtTarget = true;
    keeper.previewArrival = true;
  }
}

void KeeperProvider::calcOptPlayingPosition(Keeper& keeper)
{
  Vector2f optPosition;
  // keep robot near own goal
  optPosition.y() = theBallSymbols.ballPositionField.y() * 650 / theFieldDimensions.yPosLeftSideline;
  // keep safety distance to goal post
  optPosition.x() = theFieldDimensions.xPosOwnGroundline + std::max(distanceFromGroundLine, std::max(0.f, 300.f - std::abs(theFieldDimensions.yPosLeftGoal - std::abs(optPosition.y()))));
  keeper.optPosition.translation = optPosition;
  keeper.optPosition.rotation = 0_deg;
  if (std::abs(keeper.optPosition.translation.y()) > theFieldDimensions.yPosLeftGoal - 250)
    keeper.optPosition.rotation = (theBallSymbols.ballPositionField - optPosition).angle();
}

void KeeperProvider::updateBallchaserKeeper(Keeper& keeper)
{

  /*
  ** calculation of the kick position of the goalie. Tries kick as soon as possible but ofc not into own goal..
  */
  keeper.optPosition = Pose2f(0, theBallSymbols.ballPositionField);

  static const Vector2f leftGoalPost(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal);
  static const Vector2f rightGoalPost(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal);
  Vector2f goalCenter(theFieldDimensions.xPosOwnGroundline - 500.f, 0.f);
  Angle optRotation = theGoalSymbols.centerAngleBallToOppGoalWC;
  if (theBallSymbols.ballPositionField.y() > theFieldDimensions.yPosLeftGoal + theFieldDimensions.goalPostRadius)
    optRotation = (theBallSymbols.ballPositionField - leftGoalPost).angle() + pi_4;
  else if (theBallSymbols.ballPositionField.y() < theFieldDimensions.yPosRightGoal - theFieldDimensions.goalPostRadius)
    optRotation = (theBallSymbols.ballPositionField - rightGoalPost).angle() - pi_4;
  else
    optRotation = (theBallSymbols.ballPositionField - goalCenter).angle() * 0.75f;

  if (kickLeft && keeper.optPosition.translation.y() < -150.f && optRotation < -20_deg)
    kickLeft = false;
  if (!kickLeft && keeper.optPosition.translation.y() > 150.f && optRotation > 20_deg)
    kickLeft = true;

  keeper.optPosition.rotation = optRotation;
  keeper.optPosition.translate(-theBehaviorConfiguration.optDistanceToBallX, theBehaviorConfiguration.optDistanceToBallY * (kickLeft ? -1.f : 1.f));
  keeper.thresholdRotation = 10_deg;
  keeper.thresholdXBack = 30;
  keeper.thresholdXFront = 30;
  keeper.thresholdY = 30;
  keeper.optKickTarget = Pose2f(keeper.optPosition).translate(2000.f, 0.f).translation;
  // TODO: select kick depending on situation
  keeper.useLongKick = false;
  keeper.walkKick = WalkRequest::StepRequest::kickHackLong;
  keeper.stopAtTarget = false;
  keeper.previewArrival = true;
}

MAKE_MODULE(KeeperProvider, behaviorControl)
