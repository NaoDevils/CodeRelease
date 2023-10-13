/**
* @file DefenderSingleProvider.cpp
*
* Implementation of class DefenderSingleProvider.
*
*/

#include "DefenderSingleProvider.h"

void DefenderSingleProvider::update(DefenderSingle& positioningSymbols)
{
  // General idea:
  // We know that we are the lone defender so we help the keeper but try not to impede his vision.
  // We keep the position near our own penalty area and cover the short angle to the goal.
  // This module's position should be synchronized with the keeper/replacement keeper positioning module!
  bool isStateSetOrPlaying = theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING;
  Vector2f ballPosition = isStateSetOrPlaying ? theBallSymbols.ballPositionFieldPredicted : Vector2f::Zero();
  updateBallIsLeft(ballPosition);

  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;

  decide(positioningSymbols, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
}

//Generalized set functions ==============================================================================================================

void DefenderSingleProvider::setPlayingOrSetThresholds(DefenderSingle& positioningSymbols)
{
  ThresholdUtils::setThreshholdsCustom(positioningSymbols, 10_deg, 50.f, 150.f, 100.f);
}

void DefenderSingleProvider::setNotPlayingOrSetThresholds(DefenderSingle& positioningSymbols)
{
  ThresholdUtils::setThreshholdsCustom(positioningSymbols, 15_deg, 50.f, 150.f, 150.f);
}

void DefenderSingleProvider::handleGeneralSetPlay(DefenderSingle& positioningSymbols)
{
  setPlayingOrSetThresholds(positioningSymbols);
  PositionUtils::setPosition(positioningSymbols, calculateSetPlayPosition(positioningSymbols));
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
}

//State functions ====================================================================================================================

void DefenderSingleProvider::stateReady_kickOff_own(DefenderSingle& positioningSymbols, const Vector2f& ballPosition) //TODO: Can be reached in STATE PLAYING
{
  if (theGameInfo.state == STATE_PLAYING)
    setPlayingOrSetThresholds(positioningSymbols);
  else
    setNotPlayingOrSetThresholds(positioningSymbols);
  //PositionUtils::setPosition(positioningSymbols, coverShortAngleToGoal(true, positioningSymbols));
  PositionUtils::setPosition(positioningSymbols, theFieldDimensions.xPosOwnGoalArea + 100.f, (theFieldDimensions.yPosLeftGoal + theFieldDimensions.yPosCenterGoal) / 2.f);
  PositionUtils::turnToPosition(positioningSymbols, ballPosition);
}

void DefenderSingleProvider::stateReady_kickOff_opponent(DefenderSingle& positioningSymbols, const Vector2f& ballPosition)
{
  if (theGameInfo.state == STATE_PLAYING)
    setPlayingOrSetThresholds(positioningSymbols);
  else
    setNotPlayingOrSetThresholds(positioningSymbols);
  //PositionUtils::setPosition(positioningSymbols, coverShortAngleToGoal(true, positioningSymbols));
  PositionUtils::setPosition(positioningSymbols, theFieldDimensions.xPosOwnGoalArea + 100.f, (theFieldDimensions.yPosRightGoal + theFieldDimensions.yPosCenterGoal) / 2.f);
  PositionUtils::turnToPosition(positioningSymbols, ballPosition);
}

float DefenderSingleProvider::goalKick_own(DefenderSingle& positioningSymbols, bool left)
{
  handleGeneralSetPlay(positioningSymbols);
  //Ballchaser handles ball so we guard center of the goal, since keeper handles relevant side
  PositionUtils::setPosition(positioningSymbols, theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  return 1.f;
}

float DefenderSingleProvider::goalKick_opponent(DefenderSingle& positioningSymbols, bool left)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 0;
}

float DefenderSingleProvider::pushingFreeKick_own(DefenderSingle& positioningSymbols)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 1.f;
}

float DefenderSingleProvider::pushingFreeKick_opponent(DefenderSingle& positioningSymbols)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 1.f;
}

float DefenderSingleProvider::cornerKick_own(DefenderSingle& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 1.f;
}

float DefenderSingleProvider::cornerKick_opponent(DefenderSingle& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  setPlayingOrSetThresholds(positioningSymbols);
  //Guard center of the goal, since keeper handles relevant side and we don't want to be outpassed
  PositionUtils::setPosition(positioningSymbols, theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
  return 1.f;
}

float DefenderSingleProvider::kickIn_own(DefenderSingle& positioningSymbols, bool left)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 0.f;
}

float DefenderSingleProvider::kickIn_opponent(DefenderSingle& positioningSymbols, bool left)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 0.f;
}

float DefenderSingleProvider::stateReady_penaltyKick_own(DefenderSingle& positioningSymbols)
{
  //Stay back and watch
  regularPlay(positioningSymbols);
  return 0.f;
}

float DefenderSingleProvider::stateReady_penaltyKick_opponent(DefenderSingle& positioningSymbols)
{
  //Get out of own penalty area
  float targetPosX = theFieldDimensions.xPosOwnGoalArea;
  bool robotIsLeft = (theRobotPose.translation.y() >= 0.f);
  float targetPosY = (robotIsLeft ? 1.f : -1.f) * (theFieldDimensions.yPosLeftPenaltyArea + 400.f);
  Angle targetRotation = robotIsLeft ? -90_deg : 90_deg;
  positioningSymbols.optPosition.translation = Vector2f(targetPosX, targetPosY);
  positioningSymbols.optPosition.rotation = targetRotation;
  positioningSymbols.thresholdXFront = 50.f;
  positioningSymbols.thresholdXBack = 150.f;
  return 1.f;
}

void DefenderSingleProvider::regularPlay(DefenderSingle& positioningSymbols)
{
  setPlayingOrSetThresholds(positioningSymbols);


  Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;
  guardPass = (guardPass && ballPosition.x() < theFieldDimensions.xPosOwnGoalArea + 100) || (!guardPass && ballPosition.x() < theFieldDimensions.xPosOwnGoalArea);
  if (guardPass)
  {
    guardPassOpportunities(positioningSymbols);
    positioningSymbols.log_currObj = "guardingPass";
  }
  else
  {
    Vector2f targetPos = coverShortAngleToGoal(true, positioningSymbols);
    PositionUtils::setPosition(positioningSymbols, targetPos);
  }
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  //AvoidUtils::avoidBall(positioningSymbols, ballPosition);
  //AvoidUtils::avoidPositionConflict(positioningSymbols, theBehaviorConfiguration, theTeammateData, ballPosition);
  //AvoidUtils::avoidBallChaser(positioningSymbols, theBallchaser, theBallChaserDecision, theBehaviorConfiguration, theTeammateData, !ballIsLeft);
}

// hysteresis helper functions ===============================================================================================================

/**
 * @brief Avoids constant changes of the value
*/
void DefenderSingleProvider::updateBallIsLeft(const Vector2f& ballPosition)
{
  if (ballIsLeft && ballPosition.y() < -250.f)
    ballIsLeft = false;
  if (!ballIsLeft && ballPosition.y() > 250.f)
    ballIsLeft = true;
}

// Position calculator functions ==============================================================================================================

Vector2f DefenderSingleProvider::coverShortAngleToGoal(bool usePredictedBallLocation, DefenderSingle& positioningSymbols)
{
  Vector2f ballPositionField = usePredictedBallLocation ? theBallSymbols.ballPositionFieldPredicted : theBallSymbols.ballPositionField;

  // Calculations are in field coordinates.
  // Note: own goal center/post should always be in front of ball for this calculation,
  // otherwise we run into strange singularities if ball is near own ground line in our goal.
  float xPosToCover = std::min(theFieldDimensions.xPosOwnGroundline, ballPositionField.x() - 200.f);
  float yPostOfPoast = ballIsLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal;
  float yPosToCover = (theFieldDimensions.yPosCenterGoal + yPostOfPoast * 2) / 3;
  Vector2f positionToCover = Vector2f(xPosToCover, yPosToCover); // At or left of ground line and between goal post and goal center
  Vector2f vectorShortAngleToBall = ballPositionField - positionToCover; // Vector from cover position to ball position

  // we want to limit this angle to avoid running outside of the field
  Angle angleToCover = std::min(70_deg, std::max<Angle>(-70_deg, vectorShortAngleToBall.angle()));

  // finally, calculate position to block this angle
  // move forward if not defensive behavior
  bool behaveDefensively = true; // theTacticSymbols.defensiveBehavior || theBallSymbols.ballPositionFieldPredicted.x() <= theFieldDimensions.xPosHalfWayLine - 1000.f;
  float ballFactor = behaveDefensively ? 0.f : //Walk to the ball if aggressive
      std::min(1.f, std::max(0.f, (ballPositionField.x() + theFieldDimensions.xPosOpponentGroundline) / (2.f * theFieldDimensions.xPosOpponentGroundline)));
  float xPos = theBehaviorConfiguration.behaviorParameters.defenderSinglePositionConstraints.minX + //-3700.f Slightly in front of goal area
      ballFactor
          * (theBehaviorConfiguration.behaviorParameters.defenderSinglePositionConstraints.maxX - theBehaviorConfiguration.behaviorParameters.defenderSinglePositionConstraints.minX);

  const float distanceToOwnGoal = xPos - theFieldDimensions.xPosOwnGroundline; //800.f if defensive

  Vector2f vectorFromCoverPositionToOptPosition = Vector2f(distanceToOwnGoal, 0.f).rotate(angleToCover); // 800.f away towards ball position
  Vector2f optPosition;
  // get intersection between line in front of penalty area and vectorFromCoverPositionToOptPosition
  Geometry::Line line1 = Geometry::Line(Vector2f(theFieldDimensions.xPosOwnGroundline + distanceToOwnGoal, -10000.f), Vector2f(0.f, 100.f)); //Parallel to ground line slightly in front of goal area
  Geometry::Line line2 = Geometry::Line(positionToCover, vectorFromCoverPositionToOptPosition); // Line through cover position and ball
  if (!Geometry::getIntersectionOfLines(line1, line2, optPosition))
  {
    optPosition = positionToCover + vectorFromCoverPositionToOptPosition; //Impossible to reach lines will always intersect
    positioningSymbols.log_currObj = "didNotIntersect";
  }
  else
  {
    positioningSymbols.log_currObj = "didIntersect";
  }


  // we could catch the case of the ball at the field side here and put the player next to the penalty area
  // but for now we keep the single defender in the center area
  bool opponentKickIn = (theGameInfo.setPlay == SET_PLAY_KICK_IN && !theGameSymbols.ownKickOff);
  positioningSymbols.log_obj1 = opponentKickIn ? "oppKickIn" : "no oppKickIn";
  optPosition.y() = std::min(theFieldDimensions.yPosLeftPenaltyArea, std::max(theFieldDimensions.yPosRightPenaltyArea, optPosition.y())); //Allow y placements just outside of penalty area. Decrease possible spread towards own goal.
  if (opponentKickIn)
  {
    optPosition.y() = std::min(theFieldDimensions.yPosLeftPenaltyArea - 400.f, std::max(theFieldDimensions.yPosRightPenaltyArea + 400.f, optPosition.y()));
  }
  return optPosition;
}

void DefenderSingleProvider::guardPassOpportunities(DefenderSingle& positioningSymbols)
{
  PositionUtils::setPosition(positioningSymbols, theFieldDimensions.xPosOwnGoalArea, ballIsLeft ? theFieldDimensions.yPosLeftGoalArea - 250 : theFieldDimensions.yPosRightGoalArea + 250);
}

Vector2f DefenderSingleProvider::calculateSetPlayPosition(DefenderSingle& positioningSymbols)
{
  // start with the default position and avoid if necessary
  // predicted is not necessary since ball does not move in set play
  Vector2f optPosition = coverShortAngleToGoal(false, positioningSymbols);

  // try to keep 800 mm distance
  const float minDistanceToBall = 800.f;
  Vector2f vectorFromBallPosition = (optPosition - theBallSymbols.ballPositionField);
  float distanceToBallPosition = vectorFromBallPosition.norm();
  if (distanceToBallPosition < minDistanceToBall)
  {
    Geometry::Line penaltyLine;
    penaltyLine.base = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, 0.f);
    penaltyLine.direction = Vector2f(0.f, 1.f);
    Vector2f firstIntersection, secondIntersection;
    int noOfIntersections = Geometry::getIntersectionOfLineAndCircle(penaltyLine, Geometry::Circle(theBallSymbols.ballPositionField, minDistanceToBall), firstIntersection, secondIntersection);
    if (noOfIntersections < 2)
      optPosition = theBallSymbols.ballPositionField + vectorFromBallPosition.normalize(distanceToBallPosition);
    else
    {
      // use ballIsLeft as a helper for hysteresis
      bool firstIntersectionIsCloserToCenter = std::abs(firstIntersection.y()) < std::abs(secondIntersection.y() + (ballIsLeft ? 100.f : 0.f));
      optPosition = firstIntersectionIsCloserToCenter ? firstIntersection : secondIntersection;
    }
  }
  return optPosition;
}

MAKE_MODULE(DefenderSingleProvider, behaviorControl)
