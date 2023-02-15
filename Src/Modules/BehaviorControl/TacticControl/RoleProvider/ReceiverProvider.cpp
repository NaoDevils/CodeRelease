/**
* @file ReceiverProvider.cpp
*
* Implementation of class ReceiverProvider.
*
*/

#include "ReceiverProvider.h"

void ReceiverProvider::update(Receiver& positioningSymbols)
{
  updateBallchaserData();
  playDefensive = false;
  decide(positioningSymbols, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void ReceiverProvider::stateReady_kickOff_own(Receiver& positioningSymbols, const Vector2f& ballPosition)
{
  getReadyPosition(positioningSymbols, true); //TODO: What if this method is calld in state playing
}

void ReceiverProvider::stateReady_kickOff_opponent(Receiver& positioningSymbols, const Vector2f& ballPosition)
{
  getReadyPosition(positioningSymbols, false);
}

float ReceiverProvider::goalKick_own(Receiver& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::goalKick_opponent(Receiver& positioningSymbols, bool left)
{
  playDefensive = true;
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::pushingFreeKick_own(Receiver& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::pushingFreeKick_opponent(Receiver& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::cornerKick_own(Receiver& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  playDefensive = true;
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::cornerKick_opponent(Receiver& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::kickIn_own(Receiver& positioningSymbols, bool left)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::kickIn_opponent(Receiver& positioningSymbols, bool left)
{
  playDefensive = true;
  regularPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::stateReady_penaltyKick_own(Receiver& positioningSymbols)
{
  handleSetPlay(positioningSymbols);
  return 1.f;
}

float ReceiverProvider::stateReady_penaltyKick_opponent(Receiver& positioningSymbols)
{
  regularPlay(positioningSymbols);
  return 1.f;
}

void ReceiverProvider::regularPlay(Receiver& positioningSymbols)
{
  bool ballIsLeft = theBallSymbols.ballPositionFieldPredicted.y() > 0;
  bool transitionSafe = (ballchaserPose.translation - theRobotPoseAfterPreview.translation).norm() >= evadeDistanceToBallchaser + 100.f;

  if (evadeBallchaser)
  {
    //Check if a support state is safe:
    // 1) The ball is to the left/right of us
    // 2) The ballchaser is at least 500 to the right/left of us and the ball is right/left of us

    //bool toLeftTransitionSafe = (!ballIsLeft && theRobotPoseAfterPreview.translation.y() - 500.f > ballchaserPose.translation.y());
    //bool toRightTransitionSafe = (ballIsLeft && theRobotPoseAfterPreview.translation.y() + 500.f < ballchaserPose.translation.y());
    //bool safeTransitionToSupportState = toLeftTransitionSafe || toRightTransitionSafe;
    if (transitionSafe)
    {
      supportLeft = !ballIsLeft;
      evadeBallchaser = false;
    }
  }
  else
  {
    float previewedYPos = theRobotPoseAfterPreview.translation.y();
    float bcYPos = ballchaserPose.translation.y();

    //Check if we should no longer support left and instead evade
    // 1) The robot is too far left
    // 2) OR The robot is to the right of the ballchaser
    //bool transitionOutOfRight = previewedYPose < theFieldDimensions.yPosRightSideline + 1000.f
    //  || ballchaserPose.translation.y() < previewedYPose;
    //bool transitionOutOfLeft = previewedYPose > theFieldDimensions.yPosLeftSideline - 1000.f
    //  || ballchaserPose.translation.y() > previewedYPose;

    bool tooCloseToBC = (ballchaserPose.translation - theRobotPoseAfterPreview.translation).norm() < evadeDistanceToBallchaser;

    //We switch out of supporting left when:
    // 1) We are already on the right side and have enough space there
    // 2) We don't have enough space on the left side
    bool changeToRightSide1 = bcYPos >= theFieldDimensions.yPosRightSideline + (wantedYSpaceBorderToBC * 11.f / 8.f) && ballchaserPose.translation.y() > previewedYPos;
    bool changeToRightSide2 = bcYPos >= theFieldDimensions.yPosLeftSideline - (wantedYSpaceBorderToBC * 6.f / 6.f);

    //We switch out of supporting right when:
    // 1) We are already on the left side and have enough space there
    // 2) We don't have enough space on the right side
    bool changeToLeftSide1 = bcYPos <= theFieldDimensions.yPosLeftSideline - (wantedYSpaceBorderToBC * 11.f / 8.f) && ballchaserPose.translation.y() < previewedYPos;
    bool changeToLeftSide2 = bcYPos <= theFieldDimensions.yPosRightSideline + (wantedYSpaceBorderToBC * 6.f / 6.f);

    //Change state
    if (tooCloseToBC)
    {
      evadeBallchaser = true;
    }
    else if (supportLeft && (changeToRightSide1 || changeToRightSide2))
    {
      supportLeft = false;
      positioningSymbols.log_obj3 = changeToRightSide1 ? "Already on right" : "Not enough space on left";
    }
    else if (!supportLeft && (changeToLeftSide1 || changeToLeftSide2))
    {
      supportLeft = true;
      positioningSymbols.log_obj3 = changeToRightSide1 ? "Already on left" : "Not enough space on right";
    }
  }

  //Execute new state
  positioningSymbols.log_obj2 = playDefensive ? "playingDefensive" : "playingOffensive";
  if (evadeBallchaser)
  {
    positioningSymbols.log_currObj = "evade";
    calcEvadeBallchaserPosition(positioningSymbols);
  }
  else
  {
    positioningSymbols.log_currObj = supportLeft ? "supportLeft" : "supportRight";
    calcSupportBallchaserPosition(positioningSymbols);
  }
}

// set positions =========================================================================================================================================================

void ReceiverProvider::handleSetPlay(Receiver& receiver)
{
  receiver.stopAtTarget = true;
  receiver.previewArrival = true;
  receiver.thresholdRotation = 10_deg;
  receiver.thresholdXBack = 100.f;
  receiver.thresholdXFront = 50.f;
  receiver.thresholdY = 150.f;
  receiver.optPosition.translation = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea - 300.f,
      ballchaserPose.translation.y() + ((ballchaserPose.translation.y() > receiver.optPosition.translation.y()) ? -700.f : 700.f));
  receiver.optPosition.rotation = (Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f) - receiver.optPosition.translation).angle();
}

/**
* \brief Checks if the robot should position itself on the side or in the center.
*
* The decision is based on the ballchasers optimal position.
* The result is stored in takeWingPosition.
*/
void ReceiverProvider::calcSupportBallchaserPosition(Receiver& positioningSymbols)
{
  positioningSymbols.stopAtTarget = false;
  positioningSymbols.previewArrival = true;
  positioningSymbols.thresholdRotation = 10_deg;
  positioningSymbols.thresholdXBack = 150.f;
  positioningSymbols.thresholdXFront = 150.f;
  positioningSymbols.thresholdY = 150.f;

  float unrestrainedXPosition = ballchaserPose.translation.x() + (playDefensive ? 0.f : xSupportDistanceToBallchaser);
  float desiredXPosition = std::min(std::max(minXPosition, unrestrainedXPosition), theFieldDimensions.xPosOpponentPenaltyArea - 300);
  float unrestrainedYPosition = ballchaserPose.translation.y() + (supportLeft ? ySupportDistanceToBallchaser : -ySupportDistanceToBallchaser);
  float desiredYPosition = std::max(std::min(unrestrainedYPosition, theFieldDimensions.yPosLeftSideline - 1000.f), theFieldDimensions.yPosRightSideline + 1000.f);
  Vector2f optPosition = Vector2f(desiredXPosition, desiredYPosition);

  //PositionUtils::setPosition(positioningSymbols, desiredXPosition, supportLeft ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);

  if (useHeatMap)
  {
    checkHeat(optPosition, positioningSymbols);
  }
  else
  {
    positioningSymbols.optPosition.rotation = (theBallSymbols.ballPositionField - positioningSymbols.optPosition.translation).angle();
    PositionUtils::setPosition(positioningSymbols, optPosition);
  }
}

/**
 * @brief 
 * 
 * Heatmap used in Executable kick. In general use HeatmapCollection.
 * 
 * 
*/
void ReceiverProvider::checkHeat(Vector2f positionToTry, Receiver& positioningSymbols) //TODO: Small Modifier for closeness to penalty line?
{
  float const stepSize = 200.f;
  float const heatRadius = 300.f;
  int const nmbrSteps = 3;
  int const nmbrSkippedSteps = 2;

  float maxStepScore = -1000.f;
  Vector2f bestPosition;
  for (int xStep = -nmbrSteps + nmbrSkippedSteps; xStep <= nmbrSteps; xStep++)
  {
    for (int yStep = abs(xStep) - nmbrSteps; yStep <= nmbrSteps - abs(xStep); yStep++)
    {
      Vector2f positionToCheck = positionToTry + Vector2f(stepSize * xStep, stepSize * yStep);
      if (!FieldUtils::isIllegal(positionToCheck, theFieldDimensions))
      {
        const HeatMap::Area area = HeatMap::getArea(positionToCheck, heatRadius, theFieldDimensions);
        const auto sidesHeatScore = (-sidesHeatFactor * theHeatMapCollection.sidesHeatMap.getHeat(area, theFieldDimensions));
        const auto opponentsHeatScore = (-opponentsHeatFactor * theHeatMapCollection.opponentsMaxHeatMap.getHeat(area, theFieldDimensions));
        float stepScore = sidesHeatScore + opponentsHeatScore;
        if (stepScore > maxStepScore)
        {
          maxStepScore = stepScore;
          bestPosition = positionToCheck;
        }
      }
    }
  }

  PositionUtils::setPosition(positioningSymbols, bestPosition);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
}

/**
* \brief Determine target position for the ready state.
*
* The robot positions itself close to the halfway line in between the center and the
* left or right sideline (depending on the direction of the kickoff).
*/
void ReceiverProvider::getReadyPosition(Receiver& positioningSymbols, bool ownKickOff)
{
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
  positioningSymbols.thresholdRotation = 10_deg;
  positioningSymbols.thresholdXBack = 100.f;
  positioningSymbols.thresholdXFront = 50.f;
  positioningSymbols.thresholdY = 150.f;
  float desiredXPosition = ownKickOff ? -500.f : -1000.f;
  positioningSymbols.optPosition = Pose2f(0_deg, desiredXPosition, 1500);
}

/**
* \brief Evade ballchaser when close to sidelines by:
* 1) Going to the left/right depending on its position compared to the ballchaser
* 2) TODO If to close to border try moving opposite direction crossing ballchaser yPos
* 3) If crossing not collision free move up
*
* The robot should try to avoid crossing the kick/dribble path of the ballchaser.
*/
void ReceiverProvider::calcEvadeBallchaserPosition(Receiver& positioningSymbols)
{
  bool evadeToLeft = theBallSymbols.ballPositionField.y() < theRobotPoseAfterPreview.translation.y();
  bool sideEvadePossible = (evadeToLeft && (theRobotPoseAfterPreview.translation.y() + evadeDistanceToBallchaser < theFieldDimensions.yPosLeftSideline))
      || (!evadeToLeft && (theRobotPoseAfterPreview.translation.y() - 600 > theFieldDimensions.yPosRightSideline));
  positioningSymbols.stopAtTarget = false;
  positioningSymbols.previewArrival = true;
  positioningSymbols.thresholdRotation = 10_deg;
  positioningSymbols.thresholdXBack = 100.f;
  positioningSymbols.thresholdXFront = 50.f;
  positioningSymbols.thresholdY = 150.f;
  if (sideEvadePossible)
  {
    positioningSymbols.optPosition.translation.x() = ballchaserPose.translation.x() + evadeDistanceToBallchaser - 200.f;
    positioningSymbols.optPosition.translation.y() = ballchaserPose.translation.y() + (evadeToLeft ? 2 * evadeDistanceToBallchaser : -2 * evadeDistanceToBallchaser);
    positioningSymbols.log_obj1 = "sideEvade possible";
  }
  else
  {
    positioningSymbols.optPosition.translation.x() = ballchaserPose.translation.x() + evadeDistanceToBallchaser + 200.f;
    positioningSymbols.optPosition.translation.y() = evadeToLeft ? (theFieldDimensions.yPosLeftSideline - 200.f) : (theFieldDimensions.yPosRightSideline + 200.f);
    positioningSymbols.log_obj1 = "sideEvade not possible";
  }

  positioningSymbols.optPosition.rotation = 0_deg; // TODO: rotation should be towards the other side - do we need two evade states?
}

// update to avoid hysteresis ========================================================================================================================================

void ReceiverProvider::updateBallchaserData()
{
  for (auto& mate : theTeammateData.teammates)
  {
    if (mate.behaviorData.role == BehaviorData::RoleAssignment::ballchaser)
    {
      ballchaserPose = mate.pose;
      ballchaserAction = mate.behaviorData.soccerState;
      ballchaserKickTarget = mate.behaviorData.kickTarget.cast<float>();
    }
  }
}

MAKE_MODULE(ReceiverProvider, behaviorControl)
