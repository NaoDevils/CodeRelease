/** 
* @file BallSymbolsProvider.cpp
*
* Implementation of class BallSymbolsProvider.
*
* @author Ingmar Schwarz
*/

#include "BallSymbolsProvider.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>

void BallSymbolsProvider::update(BallSymbols& ballSymbols)
{
  DECLARE_DEBUG_DRAWING("behavior:BallSymbols:predictedBall", "drawingOnField"); // drawing of the ball predicted model
  DECLARE_DEBUG_DRAWING("behavior:BallSymbols:getYPosWhenBallReachesOwnXPos", "drawingOnField"); // drawing of the ball predicted model
  DECLARE_DEBUG_DRAWING("behavior:BallSymbols:obstacleBlockingBall", "drawingOnField"); // drawing of the obstacle blocking the ball

  calculateBallCouldHaveBeenSeen(localBallSymbols);

  int timeSinceLastSeenLocal = theFrameInfo.getTimeSince(theBallModelAfterPreview.timeWhenLastSeen);
  int timeSinceLastSeenRemote = theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen);
  localBallSymbols.timeSinceLastSeen = timeSinceLastSeenLocal;
  localBallSymbols.timeSinceLastSeenByTeamMates = timeSinceLastSeenRemote;
  localBallSymbols.timeSinceLastSeenByTeam = std::min<int>(timeSinceLastSeenLocal, timeSinceLastSeenRemote);

  localBallSymbols.ballWasSeen = localBallSymbols.timeSinceLastSeen < timeForBallWasSeen;
  localBallSymbols.ballLostForTeam = localBallSymbols.timeSinceLastSeenByTeam > timeSinceTeamSeenForLostSwitch;
  if (localBallSymbols.timeSinceLastSeen < 100)
    localBallSymbols.ballLastSeenLeft = currentlyUsedBallModel.lastPerception.y() > 0;

  localBallSymbols.ballFoundAfterDropIn = theGameSymbols.timeSinceLastBallOut < 0 || theGameSymbols.timeSinceLastBallOut > localBallSymbols.timeSinceLastSeenByTeam / 1000
      || theGameSymbols.timeSinceLastBallOut > 10;
  localBallSymbols.ballFoundAfterGameStart = theBallModelAfterPreview.timeWhenLastSeen > 0;

  // --- decide which ball model to use ---
  chooseBallModel(localBallSymbols, currentlyUsedBallModel);

  // ball to field coordinates
  if (theGameInfo.state == STATE_READY)
  {
    // some roles use the ball for positioning during ready
    if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT || theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
    {
      const float xPosPenaltyMark = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark;
      localBallSymbols.ballPositionField << xPosPenaltyMark, theFieldDimensions.yPosCenterGoal;
    }
    else
      localBallSymbols.ballPositionField.setZero();

    localBallSymbols.ballPositionFieldPredicted = localBallSymbols.ballPositionField;
    localBallSymbols.ballPositionRelative = Transformation::fieldToRobot(theRobotPoseAfterPreview, localBallSymbols.ballPositionField);
    localBallSymbols.ballPositionRelativePredicted = localBallSymbols.ballPositionRelative;
    localBallSymbols.ballPositionRelativeWOPreview = localBallSymbols.ballPositionField;
    localBallSymbols.ballVelocityRelative.setZero();
    localBallSymbols.ballVelocityRelativeWOPreview.setZero();
  }
  else
  {
    localBallSymbols.ballPositionField = Transformation::robotToField(theRobotPoseAfterPreview, currentlyUsedBallModel.estimate.position);
    localBallSymbols.ballPositionRelative = currentlyUsedBallModel.estimate.position;
    localBallSymbols.ballVelocityRelative = currentlyUsedBallModel.estimate.velocity;

    // predicted ball position
    setPredictedBallPosition(localBallSymbols);
  }

  ballWasSeenRecently = localBallSymbols.timeSinceLastSeen < timeForBallWasSeen;
  ballIsRolling = currentlyUsedBallModel.estimate.velocity.norm() > 50.f; // Ball is faster than 50 mm/s
  Vector2f vecBalltoRobot = theRobotPoseAfterPreview.translation - localBallSymbols.ballPositionField;
  Angle difBallRobot = 180_deg;
  if (localBallSymbols.ballVelocityRelative.norm() > 0 && vecBalltoRobot.norm() > 0)
    difBallRobot = std::abs(localBallSymbols.ballVelocityRelative.angleTo(vecBalltoRobot));
  bool ballIsMovingTowardsTheRobot = ballIsRolling && difBallRobot < 70_deg;

  // for diving / ball stopping
  if (ballWasSeenRecently && ballIsMovingTowardsTheRobot && ballSymbols.ballPositionField.x() > theFieldDimensions.xPosOwnGroundline) // getYPosWhenBallReachesOwnGroundLine calc doesn't make sense otherwise
  {
    localBallSymbols.yPosWhenBallReachesOwnYAxis = getYPosWhenBallReachesOwnYAxis();
    localBallSymbols.yPosWhenBallReachesGroundLine = getYPosWhenBallReachesOwnGroundLine(localBallSymbols);
  }
  else
  {
    localBallSymbols.yPosWhenBallReachesOwnYAxis = std::numeric_limits<float>::max();
    localBallSymbols.yPosWhenBallReachesGroundLine = std::numeric_limits<float>::max();
  }

  localBallSymbols.ballHitsMe = calculateBallHitsMe(localBallSymbols);
  localBallSymbols.ballBlockable = calculateBallBlockable(localBallSymbols);
  localBallSymbols.obstacleBlockingBall = calculateObstacleBlockingBall(localBallSymbols); // TODO: use? was used in ballLost

  // wasSeen/inPenaltyArea/wasLeft
  const int bufferZoneGoal = (localBallSymbols.ballInOwnGoalArea ? 300 : 200);
  localBallSymbols.ballInOwnGoalArea = (std::abs(localBallSymbols.ballPositionField.y()) < theFieldDimensions.yPosLeftGoalArea + bufferZoneGoal)
      && (localBallSymbols.ballPositionField.x() < theFieldDimensions.xPosOwnGoalArea + bufferZoneGoal);
  const int bufferZonePenalty = (localBallSymbols.ballInOwnGoalArea ? 300 : 100);
  localBallSymbols.ballInOwnPenaltyArea = (std::abs(localBallSymbols.ballPositionField.y()) < theFieldDimensions.yPosLeftPenaltyArea + bufferZonePenalty)
      && (localBallSymbols.ballPositionField.x() < theFieldDimensions.xPosOwnPenaltyArea + bufferZonePenalty);

  localBallSymbols.ballProbablyCloseButNotSeen = calculateBallProbablyCloseButNotSeen(localBallSymbols);

  CIRCLE("behavior:BallSymbols:getYPosWhenBallReachesOwnYPos", 0, localBallSymbols.yPosWhenBallReachesOwnYAxis, 45, 5, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::red);
  DRAWTEXT("behavior:BallSymbols:getYPosWhenBallReachesOwnYPos", 0, localBallSymbols.yPosWhenBallReachesOwnYAxis + 30, 15, ColorRGBA::red, "Time: " << localBallSymbols.yPosWhenBallReachesOwnYAxis);

  PLOT("behavior:BallSymbols:ballY", currentlyUsedBallModel.estimate.position.y());
  CIRCLE("behavior:BallSymbols:predictedBall", localBallSymbols.ballPositionFieldPredicted.x(), localBallSymbols.ballPositionFieldPredicted.y(), 45, 5, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::red);
  PLOT("behavior:BallSymbols:ballVelocity", currentlyUsedBallModel.estimate.velocity.norm());

  if (localBallSymbols.ballProbablyCloseButNotSeen && lastBallProbablyCloseButNotSeen != localBallSymbols.ballProbablyCloseButNotSeen)
    ANNOTATION("Ball", "Ball was lost in a duel.");
  lastBallProbablyCloseButNotSeen = localBallSymbols.ballProbablyCloseButNotSeen;

  ballSymbols = localBallSymbols;
}

void BallSymbolsProvider::chooseBallModel(BallSymbols& ballSymbols, BallModel& currentlyUsedBallModel)
{
  Vector2f teamBallPositionAfterPreviewRelative = Transformation::fieldToRobot(theRobotPoseAfterPreview, theRemoteBallModel.position);
  float ballModelsPositionDiff = (float)(theBallModelAfterPreview.estimate.position - teamBallPositionAfterPreviewRelative).norm();

  bool remoteBallWasSeenLast = (theRemoteBallModel.timeWhenLastSeen + 1000) > theBallModelAfterPreview.timeWhenLastSeen;
  bool ballModelsPositionDifferAndMediumRemoteValidity = (ballModelsPositionDiff > 1000 && theRemoteBallModel.validity > theBallModelAfterPreview.validity);
  bool goodRemoteValidity = theRemoteBallModel.validity >= 0.5;
  bool localBallWasNotSeenForLong = theFrameInfo.getTimeSince(theBallModelAfterPreview.timeWhenLastSeen) - theFrameInfo.getTimeSince(theRemoteBallModel.timeWhenLastSeen) > 5000;

  bool isTeamBallBetter = remoteBallWasSeenLast && (ballModelsPositionDifferAndMediumRemoteValidity || goodRemoteValidity || localBallWasNotSeenForLong);
  bool localBallModelMayBeOutdated = theFrameInfo.getTimeSince(theBallModelAfterPreview.timeWhenLastSeen) > timeSinceSeenForLostSwitch; //TODO: maybe timeSinceBallCouldHaveBeenSeen?
  bool localBallModelBelowMediumValidity = theBallModelAfterPreview.validity < 0.5;

  if (isTeamBallBetter && (localBallModelMayBeOutdated || localBallModelBelowMediumValidity))
  {
    if (ballSymbols.useLocalBallModel)
      ANNOTATION("Ball", "Using REMOTE ball model.");
    currentlyUsedBallModel.validity = theRemoteBallModel.validity;
    currentlyUsedBallModel.estimate.position = teamBallPositionAfterPreviewRelative;
    currentlyUsedBallModel.estimate.velocity = Transformation::fieldVelocityToRobot(theRobotPoseAfterPreview, theRemoteBallModel.velocity);
    currentlyUsedBallModel.lastPerception = teamBallPositionAfterPreviewRelative;
    currentlyUsedBallModel.timeWhenLastSeen = theRemoteBallModel.timeWhenLastSeen;
    currentlyUsedBallModel.timeWhenLastSeenByTeamMate = theRemoteBallModel.timeWhenLastSeen;
    ballSymbols.useLocalBallModel = false;
    ballSymbols.ballPositionRelativeWOPreview = Transformation::fieldToRobot(theRobotPose, theRemoteBallModel.position);
    ballSymbols.ballVelocityRelativeWOPreview = Transformation::fieldVelocityToRobot(theRobotPose, theRemoteBallModel.velocity);
  }
  else
  {
    if (!ballSymbols.useLocalBallModel)
      ANNOTATION("Ball", "Go back to LOCAL ball model.");
    currentlyUsedBallModel = theBallModelAfterPreview;
    ballSymbols.useLocalBallModel = true;
    ballSymbols.ballPositionRelativeWOPreview = theBallModel.estimate.position;
    ballSymbols.ballVelocityRelativeWOPreview = theBallModel.estimate.velocity;
  }
}

void BallSymbolsProvider::setPredictedBallPosition(BallSymbols& ballSymbols)
{
  if (KickUtils::isBallKicked(theMotionInfo)) // kick will be executed by motion
  {
    timeWhenLastKickTriggered = theFrameInfo.time;
    ballSymbols.avoidBall = false;
    return;
  }

  if (theFrameInfo.getTimeSince(timeWhenLastKickTriggered) < ballPredictionTimeHorizon) // keep predicted position for some time after kick
  {
    ballSymbols.ballPositionRelativePredicted = Transformation::fieldToRobot(theRobotPoseAfterPreview, ballSymbols.ballPositionFieldPredicted);
    ballSymbols.avoidBall = false;
    return;
  }

  const float MIN_VELOCITY_X = 5.f;
  const float MIN_VELOCITY_Y = 10.f;
  if (std::abs(ballSymbols.ballVelocityRelative.x()) < MIN_VELOCITY_X && std::abs(ballSymbols.ballVelocityRelative.y()) < MIN_VELOCITY_Y)
  {
    ballSymbols.ballPositionRelativePredicted = ballSymbols.ballPositionRelative;
  }
  else
  {
    ballSymbols.ballPositionRelativePredicted = BallPhysics::getEndPosition(ballSymbols.ballPositionRelative, ballSymbols.ballVelocityRelative, theBallModelAfterPreview.friction);
  }
  ballSymbols.ballPositionFieldPredicted = Transformation::robotToField(theRobotPoseAfterPreview, ballSymbols.ballPositionRelativePredicted);
  ballSymbols.avoidBall = true;
}

bool BallSymbolsProvider::calculateBallCouldHaveBeenSeen(BallSymbols& ballSymbols)
{
  // update if the ball could have been seen
  // TODO: Unused! Maybe helpful?
  if (theBallModelAfterPreview.timeWhenLastSeen == theFrameInfo.time)
    timeSinceBallCouldHaveBeenSeen = 0;
  else if (theFrameInfo.time > lastFrameTime
      && ((Geometry::wouldPointBeVisible(theBallModelAfterPreview.estimate.position, theJointCalibration.joints[Joints::headYaw].maxAngle, theCameraInfo)
              && Geometry::ballShouldBeVisibleInImage(theBallModelAfterPreview.estimate.position, theFieldDimensions.ballRadius, theCameraMatrix, theCameraInfo))
          || (Geometry::wouldPointBeVisible(theBallModelAfterPreview.estimate.position, theJointCalibration.joints[Joints::headYaw].maxAngle, theCameraInfoUpper)
              && Geometry::ballShouldBeVisibleInImage(theBallModelAfterPreview.estimate.position, theFieldDimensions.ballRadius, theCameraMatrixUpper, theCameraInfoUpper))))
    timeSinceBallCouldHaveBeenSeen += theFrameInfo.getTimeSince(lastFrameTime);
  lastFrameTime = theFrameInfo.time;

  return timeSinceBallCouldHaveBeenSeen < timeForBallWasSeen;
}

bool BallSymbolsProvider::calculateObstacleBlockingBall(BallSymbols& ballSymbols)
{
  if (ballSymbols.timeSinceLastSeen > 5000)
    return false;
  Vector2f myPosition(theRobotPoseAfterPreview.translation.x(), theRobotPoseAfterPreview.translation.y());
  for (auto& robot : theRobotMap.robots)
  {
    Vector2f vMeToRobotWC = (robot.pose.translation - myPosition);
    float angleToRobotWC = vMeToRobotWC.angle();
    float distToRobot = vMeToRobotWC.norm();
    float angleToBallWC = (ballSymbols.ballPositionField - myPosition).angle();
    if (distToRobot < 500 && std::abs(Angle::normalize(angleToRobotWC - angleToBallWC)) < 60_deg)
    {
      CIRCLE("behavior:BallSymbols:obstacleBlockingBall", robot.pose.translation.x(), robot.pose.translation.y(), 100, 10, Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, ColorRGBA(255, 0, 0));
      return true;
    }
  }
  return false;
}

bool BallSymbolsProvider::calculateBallHitsMe(BallSymbols& ballSymbols)
{
  bool isBallModelCloseTheRobot = std::abs(ballSymbols.ballPositionRelative.x()) < distanceXForBallClose;
  bool landsBallCloseToTheRobotsSide = std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) < distanceYForBallClose;
  bool landsBallNearToTheRobotsSide = std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) < distanceYForBallNear;
  bool isPredictedBallBehindTheRobot = ballSymbols.ballPositionRelativePredicted.x() < 0.f;

  bool isBallModelClose = isBallModelCloseTheRobot && landsBallCloseToTheRobotsSide && isPredictedBallBehindTheRobot;
  bool isLastPerceptClose = landsBallNearToTheRobotsSide && std::abs(currentlyUsedBallModel.lastPerception.x()) < distanceXForBallClose
      && std::abs(currentlyUsedBallModel.lastPerception.y()) < distanceYForBallClose;

  if (!ballSymbols.ballHitsMe && ballSymbols.avoidBall && (isBallModelClose || isLastPerceptClose))
  {
    timeWhenBallHitTriggered = theFrameInfo.time;
  }

  return (theFrameInfo.getTimeSince(timeWhenBallHitTriggered) < 1000);
}

bool BallSymbolsProvider::calculateBallBlockable(BallSymbols& ballSymbols)
{
  bool isBallModelWithinRange = std::abs(ballSymbols.ballPositionRelative.x()) < std::abs(currentlyUsedBallModel.estimate.velocity.x() * timeNeededTillStartOfBlockingInSec);
  bool landsBallNearToTheRobotsSide = std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) >= distanceYForBallClose && std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) < distanceYForBallNear;

  bool isLastPerceptWithinRangeX = std::abs(currentlyUsedBallModel.lastPerception.x()) < std::abs(currentlyUsedBallModel.estimate.velocity.x() * timeNeededTillStartOfBlockingInSec);
  bool isLastPerceptWithinRangeY = std::abs(currentlyUsedBallModel.lastPerception.y()) >= distanceYForBallClose
      && std::abs(currentlyUsedBallModel.lastPerception.y()) < std::abs(currentlyUsedBallModel.estimate.velocity.y() * timeNeededTillStartOfBlockingInSec);
  bool isBallApproachingTheRobotsSide = std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) >= distanceYForBallClose && std::abs(ballSymbols.yPosWhenBallReachesOwnYAxis) < 400;

  bool isBallModelNear = isBallModelWithinRange && landsBallNearToTheRobotsSide;
  bool isLastPerceptNear = isLastPerceptWithinRangeX && isLastPerceptWithinRangeY && isBallApproachingTheRobotsSide;

  if (!ballSymbols.ballBlockable && !ballSymbols.ballHitsMe && (isBallModelNear || isLastPerceptNear))
  {
    timeWhenBallBlockableTriggered = theFrameInfo.time;
  }

  return (theFrameInfo.getTimeSince(timeWhenBallBlockableTriggered) < 1000);
}

bool BallSymbolsProvider::calculateBallProbablyCloseButNotSeen(BallSymbols& ballSymbols)
{
  bool lastBallWasClose = currentlyUsedBallModel.lastPerception.norm() < 1000;

  int timeForBallWasSeenBonus = (int)toDegrees(std::abs(theBallModelAfterPreview.estimate.position.angle())) * 10;
  int totalTimeForBallWasSeen = (int)timeForBallWasSeen + timeForBallWasSeenBonus;

  bool ballSeenRecently = ballSymbols.timeSinceLastSeen < (totalTimeForBallWasSeen + 20000);
  bool ballSeenVeryRecently = ballSymbols.timeSinceLastSeen < (totalTimeForBallWasSeen + 2000);
  ASSERT(!(!ballSeenRecently && ballSeenVeryRecently));

  return lastBallWasClose && ballSeenRecently && !ballSeenVeryRecently;
}

float BallSymbolsProvider::getYPosWhenBallReachesOwnYAxis()
{
  float yPosOwnAxis = 0.f;
  yPosOwnAxis =
      ((std::tan(currentlyUsedBallModel.estimate.position.angle()) - std::tan(currentlyUsedBallModel.estimate.velocity.angle())) * currentlyUsedBallModel.estimate.position.x());
  MODIFY("behavior:BallSymbols:yPosOwnAxis", yPosOwnAxis);
  return yPosOwnAxis;
}

float BallSymbolsProvider::getYPosWhenBallReachesOwnGroundLine(const BallSymbols& ballSymbols)
{
  float yPosGroundLine = 0.f;
  yPosGroundLine = -std::tan(currentlyUsedBallModel.estimate.getVelocityInFieldCoordinates(theRobotPoseAfterPreview).angle())
      * (ballSymbols.ballPositionField.x() - theFieldDimensions.xPosOwnGroundline);
  return yPosGroundLine;
}

MAKE_MODULE(BallSymbolsProvider, behaviorControl)
