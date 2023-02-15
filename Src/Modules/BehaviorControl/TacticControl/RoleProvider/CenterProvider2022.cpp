/**
* @file CenterProvider.cpp
*
* Implementation of class CenterProvider.
*
*/

#include "CenterProvider2022.h"

void CenterProvider2022::update(Center& positioningSymbols)
{
  DECLARE_DEBUG_DRAWING("behavior:CenterProvider2022:debugPositioning", "drawingOnField");

  bool isStateSetOrPlaying = theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING;
  Vector2f ballPosition = isStateSetOrPlaying ? theBallSymbols.ballPositionFieldPredicted : Vector2f::Zero();
  updateBallIsLeft(ballPosition);

  decide(positioningSymbols, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

void CenterProvider2022::stateReady_kickOff_own(Center& positioningSymbols, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, -500, -1500);
  PositionUtils::turnToPosition(positioningSymbols, ballPosition);
}

void CenterProvider2022::stateReady_kickOff_opponent(Center& positioningSymbols, const Vector2f& ballPosition)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, -1800, -700);
  PositionUtils::turnToPosition(positioningSymbols, ballPosition);
}

float CenterProvider2022::goalKick_own(Center& positioningSymbols, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, theBehaviorConfiguration.behaviorParameters.centerPositionConstraints.minX, 0);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float CenterProvider2022::goalKick_opponent(Center& positioningSymbols, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, 0, -1000.f);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float CenterProvider2022::pushingFreeKick_own(Center& positioningSymbols)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  regularPlay(positioningSymbols);
  return 0;
}

float CenterProvider2022::pushingFreeKick_opponent(Center& positioningSymbols)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  regularPlay(positioningSymbols);
  return 0;
}

float CenterProvider2022::cornerKick_own(Center& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  //PositionUtils::setPosition(positioningSymbols,
  //  true, 2000.f,
  //  !left, 500.f);
  PositionUtils::setPosition(positioningSymbols, 2000.f, left ? -500.f : 500.f);
  PositionUtils::turnToPosition(positioningSymbols, cornerKickPosition);
  //FrontUtils::freeSight(positioningSymbols, theRobotMap, cornerKickPosition, 300.f);
  return 1.f;
}

float CenterProvider2022::cornerKick_opponent(Center& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  //PositionUtils::setPosition(positioningSymbols,
  //  false, 2000.f,
  //  left, theFieldDimensions.yPosRightGoalArea + 200.f);
  PositionUtils::setPosition(positioningSymbols, -2000.f, left ? theFieldDimensions.yPosLeftGoalArea : 500.f);
  PositionUtils::turnToPosition(positioningSymbols, cornerKickPosition);
  return 1.f;
}

float CenterProvider2022::kickIn_own(Center& positioningSymbols, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  // receiver should be helping out offensively so the center should be a kick target/supporter at roughly the same x pos
  PositionUtils::setPosition(positioningSymbols,
      std::max(std::min(theBallSymbols.ballPositionFieldPredicted.x() - 200.f, theBehaviorConfiguration.behaviorParameters.centerPositionConstraints.maxX),
          theBehaviorConfiguration.behaviorParameters.centerPositionConstraints.minX),
      theBallSymbols.ballPositionFieldPredicted.y() + (left ? -kickInSupportDistanceY : kickInSupportDistanceY));
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float CenterProvider2022::kickIn_opponent(Center& positioningSymbols, bool left)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  //setPassivePosition(positioningSymbols, theBallSymbols.ballPositionFieldPredicted);
  float xCountStart = theFieldDimensions.xPosOwnPenaltyArea;
  float unrestrainedXPosition = (theBallSymbols.ballPositionFieldPredicted.x() * 2.f / 5.f + xCountStart * 3.f / 5.f);
  float xPos = std::max(theBehaviorConfiguration.behaviorParameters.centerPositionConstraints.minX, unrestrainedXPosition);
  float cappedDistToPenaltyLine = std::min(-xCountStart, xPos - xCountStart);
  float yPosBorder = theFieldDimensions.yPosRightFieldBorder + 900.f;
  float yPos = yPosBorder + (2 * yPosBorder) * cappedDistToPenaltyLine / xCountStart;
  PositionUtils::setPosition(positioningSymbols, xPos, left ? yPos : -yPos);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float CenterProvider2022::stateReady_penaltyKick_own(Center& positioningSymbols)
{
  ThresholdUtils::setThreshholdsHeigh(positioningSymbols);
  setPassivePosition(positioningSymbols, theBallSymbols.ballPositionFieldPredicted);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float CenterProvider2022::stateReady_penaltyKick_opponent(Center& positioningSymbols)
{
  return stateReady_penaltyKick_own(positioningSymbols);
}

void CenterProvider2022::regularPlay(Center& positioningSymbols)
{
  Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;
  if (theTacticSymbols.defensiveBehavior)
  {
    setDefensivePlayingPosition(positioningSymbols, ballPosition);
  }
  else
  {
    setAgressivePlayingPosition(positioningSymbols, ballPosition);
  }
  AvoidUtils::avoidBall(positioningSymbols, ballPosition);
  AvoidUtils::avoidPositionConflict(positioningSymbols, theBehaviorConfiguration, theTeammateData, ballPosition);
  AvoidUtils::avoidBallChaser(positioningSymbols, theBallchaser, theBallChaserDecision, theBehaviorConfiguration, theTeammateData, !ballIsLeft);
}

// set position ===========================================================================================================

void CenterProvider2022::setPassivePosition(Center& positioningSymbols, const Vector2f& ballPositionField)
{
  ThresholdUtils::setThreshholdsMedium(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, theBehaviorConfiguration.behaviorParameters.centerPositionConstraints.minX, 0);
  PositionUtils::turnToPosition(positioningSymbols, ballPositionField);
}

void CenterProvider2022::setAgressivePlayingPosition(Center& positioningSymbols, const Vector2f& ballPositionField)
{
  ThresholdUtils::setThreshholdsMedium(positioningSymbols);

  float xPos = 0.f;
  float yPos = 0.f;

  if (std::optional<Vector2f> ballChaserPositionOptional = TeamUtils::getRealBallChaserPosition(theBallChaserDecision, theTeammateData))
  {
    Vector2f realBallChaserPosition = *ballChaserPositionOptional;

    // follow ballchaser with specified distance
    if (realBallChaserPosition.y() > 100 && realBallChaserPosition.y() < theFieldDimensions.yPosLeftSideline - yDistanceToBallchaser)
    {
      xPos = realBallChaserPosition.x() - xDistanceToBallchaser;
      yPos = realBallChaserPosition.y() + yDistanceToBallchaser;
    }
    else if (realBallChaserPosition.y() < -100 && realBallChaserPosition.y() > theFieldDimensions.yPosRightSideline + yDistanceToBallchaser)
    {
      xPos = realBallChaserPosition.x() - xDistanceToBallchaser;
      yPos = realBallChaserPosition.y() - yDistanceToBallchaser;
    }
    else
    {
      xPos = realBallChaserPosition.x() - xDistanceToBallchaser;
      yPos = realBallChaserPosition.y();
    }

    // prevent robot from leaving the field or entering the penalty area
    xPos = xPos < 0.8f * theFieldDimensions.xPosOwnPenaltyArea ? 0.8f * theFieldDimensions.xPosOwnPenaltyArea : xPos;
    yPos = yPos >= theFieldDimensions.yPosLeftSideline ? theFieldDimensions.yPosLeftSideline - 100 : yPos;
    yPos = yPos <= theFieldDimensions.yPosRightSideline ? theFieldDimensions.yPosRightSideline + 100 : yPos;

    PositionUtils::setPosition(positioningSymbols, xPos, yPos);
    PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  }
  else
  {
    setPassivePosition(positioningSymbols, ballPositionField);
  }
}

void CenterProvider2022::setDefensivePlayingPosition(Center& positioningSymbols, const Vector2f& ballPositionField)
{
  ThresholdUtils::setThreshholdsLow(positioningSymbols);
  float destinationY = ballPositionField.y() / 2.f;

  // positioning between goal and ball on the other side as the defender without blocking keeper's view
  if (std::optional<Vector2f> keeperPositionOptional = TeamUtils::getTeammatePosition(theTeammateData, BehaviorData::keeper))
  {
    Vector2f keeperPosition = *keeperPositionOptional;
    positioningSymbols.keeperPosKnown = true;
    positioningSymbols.keeperPosY = keeperPosition.y();

    if (std::optional<Vector2f> defenderPositionOptional = TeamUtils::getDesiredTeammatePosition(theTeammateData, BehaviorData::defenderSingle))
    {
      Vector2f defenderPosition = *defenderPositionOptional;
      positioningSymbols.defenderPosKnown = true;
      positioningSymbols.defenderPosY = defenderPosition.y();

      float yDistanceKeeperDefender = (keeperPosition.y() > defenderPosition.y()) ? (keeperPosition.y() - defenderPosition.y()) : (defenderPosition.y() - keeperPosition.y());

      if (defenderPosition.y() > 0)
      {
        // defender is left of goal center, center should be right
        destinationY = destinationY - yDistanceKeeperDefender;
      }
      else
      {
        // defender is right of goal center, center should be left
        destinationY = destinationY + yDistanceKeeperDefender;
      }
    }
    else
    {
      positioningSymbols.defenderPosKnown = false;
    }

    // positioning between goal and ball without blocking only the keeper's view
    if (ballPositionField.y() > 0)
    {
      destinationY = destinationY - yDistanceToKeeper;
    }
    else
    {
      destinationY = destinationY + yDistanceToKeeper;
    }
  }
  else
  {
    positioningSymbols.keeperPosKnown = false;
    destinationY = destinationY - yDistanceToKeeper;
  }

  positioningSymbols.destinationY = destinationY;
  PositionUtils::setPosition(positioningSymbols, 0.8f * theFieldDimensions.xPosOwnPenaltyArea, destinationY);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
}

/**
 * @brief Avoids constant changes of the value
*/
void CenterProvider2022::updateBallIsLeft(const Vector2f& ballPosition)
{
  if (ballIsLeft && ballPosition.y() < -250.f)
    ballIsLeft = false;
  if (!ballIsLeft && ballPosition.y() > 250.f)
    ballIsLeft = true;
}

MAKE_MODULE(CenterProvider2022, behaviorControl)
