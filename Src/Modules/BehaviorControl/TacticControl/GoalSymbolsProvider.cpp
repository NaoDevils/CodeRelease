/** 
* @file GoalSymbolsProvider.cpp
*
* Implementation of class GoalSymbolsProvider.
*
* @author Tim Laue
*/

#include "GoalSymbolsProvider.h"
#include "Tools/Math/Geometry.h"

void GoalSymbolsProvider::update(GoalSymbols& goalSymbols)
{
  // calculate angles for opponent goal
  Vector2f leftOppGoalPost(theFieldDimensions.xPosOpponentGroundline, (theFieldDimensions.yPosLeftGoal - 50));
  Vector2f rightOppGoalPost(theFieldDimensions.xPosOpponentGroundline, (theFieldDimensions.yPosRightGoal + 50));

  Pose2f ballPoseWC = Pose2f(0.f, theBallSymbols.ballPositionField);
  angleToLeftOppGoalPost = Geometry::angleTo(ballPoseWC, Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal - 50));
  angleToRightOppGoalPost = Geometry::angleTo(ballPoseWC, Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal + 50));
  goalSymbols.centerAngleBallToOppGoalWC = Angle::normalize((angleToLeftOppGoalPost + angleToRightOppGoalPost) / 2.f);

  if (angleToLeftOppGoalPost < angleToRightOppGoalPost) // only relevant of goal is behind the robot with one post on each side
  {
    angleToLeftOppGoalPost += 360_deg;
  }

  goalSymbols.leftAngleBallToOppGoalWC = angleToLeftOppGoalPost;
  goalSymbols.rightAngleBallToOppGoalWC = angleToRightOppGoalPost;

  goalInFrontOfMe = angleToLeftOppGoalPost > 0.f && angleToRightOppGoalPost < 0.f;

  const Angle safetyDist = 10_deg; // 10° to every side
  goalInFrontOfMeSafe = angleToLeftOppGoalPost > safetyDist && angleToRightOppGoalPost < -safetyDist;

  goalSymbols.openingAngleOfOppGoal = angleToLeftOppGoalPost - angleToRightOppGoalPost;

  centerAngleToOppGoal = Angle::normalize((angleToLeftOppGoalPost + angleToRightOppGoalPost) / 2.f);

  angleToleranceToOpponentGoal = std::max(5.f, goalSymbols.openingAngleOfOppGoal / 2.f - safetyDist);

  // now the own goal (only center angle, the rest is not needed)
  Angle angleToLeftOwnGoalPost = Geometry::angleTo(theRobotPoseAfterPreview, Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal));
  Angle angleToRightOwnGoalPost = Geometry::angleTo(theRobotPoseAfterPreview, Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal));
  if (angleToLeftOwnGoalPost < angleToRightOwnGoalPost) // only relevant of goal is behind the robot with one post on each side
  {
    angleToLeftOwnGoalPost += 360_deg;
  }
  goalSymbols.centerAngleToOwnGoal = Angle::normalize((angleToLeftOwnGoalPost + angleToRightOwnGoalPost) / 2.f);

  goalSymbols.approachAngleWC = Angle::normalize((theBallSymbols.ballPositionField - theRobotPoseAfterPreview.translation).angle() - goalSymbols.centerAngleBallToOppGoalWC);

  // now calculate the centerAngleToOppGoalAsSeenFromBall
  ballPoseWC.rotation = theRobotPoseAfterPreview.rotation;
  Angle angleToLeftOppGoalPostAsSeenFromBall = Geometry::angleTo(ballPoseWC, leftOppGoalPost);
  Angle angleToRightOppGoalPostAsSeenFromBall = Geometry::angleTo(ballPoseWC, rightOppGoalPost);
  if (angleToLeftOppGoalPostAsSeenFromBall < angleToRightOppGoalPostAsSeenFromBall) // only relevant of goal is behind the robot with one post on each side
  {
    angleToLeftOppGoalPostAsSeenFromBall += 360_deg;
  }
  goalSymbols.centerAngleToOppGoalAsSeenFromBall = Angle::normalize((angleToLeftOppGoalPostAsSeenFromBall + angleToRightOppGoalPostAsSeenFromBall) / 2);

  goalSymbols.smallestBallToGoalPostAngle = std::min(angleToLeftOppGoalPost, -(angleToRightOppGoalPost));
}

MAKE_MODULE(GoalSymbolsProvider, behaviorControl)
