/**
* @file BallchaserKeeperProvider.cpp
*
* Implementation of class BallchaserKeeperProvider.
*
*/

#include "BallchaserKeeperProvider.h"

void BallchaserKeeperProvider::update(BallchaserKeeper& positioningSymbols)
{
  /*
  ** calculation of the kick position of the goalie. Tries kick as soon as possible but ofc not into own goal..
  */
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
  positioningSymbols.optKickTarget = Pose2f(positioningSymbols.optPosition).translate(3000.f, 0.f).translation;
  positioningSymbols.previewArrival = true;
}

MAKE_MODULE(BallchaserKeeperProvider, behaviorControl)