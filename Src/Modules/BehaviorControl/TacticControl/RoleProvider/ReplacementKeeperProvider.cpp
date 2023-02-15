/**
* @file ReplacementKeeperProvider.cpp
*
* Implementation of class ReplacementKeeperProvider.
*
*/

#include "ReplacementKeeperProvider.h"

void ReplacementKeeperProvider::update(ReplacementKeeper& positioningSymbols)
{
  // Basic idea:
  // Cover line of ball to goal. Since this keeper can not dive we cannot stay centered
  // if the ball moves and is far away (e.g. opponent kicks from middle line)
  // Therefore we will move for interception on a line in front of the goal
  // and trigger block motion when it is feasable, configurable with parameters

  // reset special role output
  positioningSymbols.blockBall = false;
  positioningSymbols.kickIt = false;

  if (theGameInfo.state != STATE_PLAYING)
  {
    // in all non-playing states we assume the ball in the center of the field
    positioningSymbols.optPosition = Pose2f(0_deg, theFieldDimensions.xPosOwnGroundline + distanceFromGroundLine, 0.f);
    positioningSymbols.thresholdXBack = 100.f;
    positioningSymbols.thresholdXFront = 100.f;
    positioningSymbols.thresholdY = 150.f;
    positioningSymbols.thresholdRotation = 10_deg;
    positioningSymbols.stopAtTarget = true;
    positioningSymbols.previewArrival = true;
  }
  else // playing
  {
    calcOptPosition(positioningSymbols);
    // ball is far away and not moving, up thresholds to keep robot still
    if (theBallSymbols.ballVelocityRelative.norm() < moveWithBallSpeed && theBallSymbols.ballPositionRelative.norm() > moveWithBallDistance)
    {
      positioningSymbols.thresholdXBack = 100.f;
      positioningSymbols.thresholdXFront = 100.f;
      positioningSymbols.thresholdY = 200.f;
      positioningSymbols.thresholdRotation = 15_deg;
      positioningSymbols.stopAtTarget = true;
      positioningSymbols.previewArrival = true;
    }
    else
    {
      positioningSymbols.thresholdXBack = 50.f;
      positioningSymbols.thresholdXFront = 150.f;
      positioningSymbols.thresholdY = 100.f;
      positioningSymbols.thresholdRotation = 10_deg;
      positioningSymbols.stopAtTarget = true;
      positioningSymbols.previewArrival = true;
    }
  }
}

void ReplacementKeeperProvider::calcOptPosition(ReplacementKeeper& positioningSymbols)
{
  // Calculations are in field coordinates.
  // Note: own goal center/post should always be in front of ball for this calculation,
  // otherwise we run into strange singularities if ball is near own ground line in our goal.
  Vector2f ownGoalCenter = Vector2f(std::min(theFieldDimensions.xPosOwnGroundline, theBallSymbols.ballPositionField.x() - 200.f), 0.f);
  Vector2f vectorToBall = theBallSymbols.ballPositionField - ownGoalCenter;

  // we want to limit this angle to avoid running outside of the field
  Angle angleToCover = std::min(80_deg, std::max<Angle>(-80_deg, vectorToBall.angle()));

  // finally, calculate position to block this angle
  // move forward if not defensive behavior
  Vector2f vectorFromCoverPositionToOptPosition = Vector2f(distanceFromGroundLine, 0.f).rotate(angleToCover);
  Vector2f optPosition;
  // get intersection between defense line and vectorFromCoverPositionToOptPosition
  if (!Geometry::getIntersectionOfLines(Geometry::Line(Vector2f(theFieldDimensions.xPosOwnGroundline + distanceFromGroundLine, -10000.f), Vector2f(0.f, 100.f)),
          Geometry::Line(ownGoalCenter, vectorFromCoverPositionToOptPosition),
          optPosition))
    optPosition = ownGoalCenter + vectorFromCoverPositionToOptPosition;

  // if the ball is rolling fast, not within our penalty area and predicted to land in our goal, intercept and/or block
  bool interceptBall = false;
  if (theBallSymbols.ballPositionFieldPredicted.x() < theRobotPoseAfterPreview.translation.x() // ball will be behind us
      && !theBallSymbols.ballInOwnPenaltyArea // not already in penalty area
      && std::abs(theBallSymbols.yPosWhenBallReachesGroundLine) < theFieldDimensions.yPosLeftGoal + 100.f && theBallSymbols.ballVelocityRelative.norm() > moveWithBallSpeed) // moving fast
  {
    if (theBallSymbols.ballPositionRelative.norm() < maxBallDistanceForBlock && theBallSymbols.ballPositionRelative.norm() > minBallDistanceForBlock
        && theBallSymbols.ballPositionField.x() > theRobotPoseAfterPreview.translation.x() && theBallSymbols.yPosWhenBallReachesOwnYAxis < yReachableWithBlock
        && theFrameInfo.getTimeSince(lastBlockTimeStamp) > timeBetweenBlockMotions)
    {
      positioningSymbols.blockBall = true;
      lastBlockTimeStamp = theFrameInfo.time;
    }
    else
    {
      optPosition.y() = theRobotPoseAfterPreview.translation.y() + theBallSymbols.yPosWhenBallReachesOwnYAxis;
      interceptBall = true;
      return;
    }
  }

  // keep robot near own goal
  if (!interceptBall)
    optPosition.y() = std::min(theFieldDimensions.yPosLeftGoal + 100.f, std::max(theFieldDimensions.yPosRightGoal - 100.f, optPosition.y()));
  // keep safety distance to goal post
  optPosition.x() = theFieldDimensions.xPosOwnGroundline + std::max(distanceFromGroundLine, std::max(0.f, 300.f - std::abs(theFieldDimensions.yPosLeftGoal - std::abs(optPosition.y()))));
  positioningSymbols.optPosition.translation = optPosition;
  positioningSymbols.optPosition.rotation = (theBallSymbols.ballPositionField - optPosition).angle();
  if (interceptBall)
    positioningSymbols.optPosition.rotation = theRobotPoseAfterPreview.rotation + Angle::normalize(theBallSymbols.ballVelocityRelative.angle() + pi);
  if ((!interceptBall && !positioningSymbols.blockBall && (ownGoalCenter - theBallSymbols.ballPositionField).norm() < 2000 && theBallSymbols.ballInOwnPenaltyArea)
      || theBallSymbols.ballPositionRelative.norm() < 800)
  {
    positioningSymbols.kickIt = true;
    positioningSymbols.optPosition = theBallSymbols.ballPositionField;
    float optDistanceToBallY = sgn(theBallSymbols.ballPositionField.y()) * (theBehaviorConfiguration.optDistanceToBallY);
    // TODO
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

    positioningSymbols.optPosition.rotation = optRotation;
    positioningSymbols.optPosition.translate(-theBehaviorConfiguration.optDistanceToBallX, -optDistanceToBallY);
    positioningSymbols.thresholdRotation = 10_deg;
    positioningSymbols.thresholdXBack = 70;
    positioningSymbols.thresholdXFront = 70;
    positioningSymbols.thresholdY = 40;
    positioningSymbols.previewArrival = true;
  }
}

MAKE_MODULE(ReplacementKeeperProvider, behaviorControl)