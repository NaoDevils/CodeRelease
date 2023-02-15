/**
* @file DefenderLeftProvider.cpp
*
* Implementation of class DefenderLeftProvider.
*
*/

#include "DefenderLeftProvider.h"

void DefenderLeftProvider::update(DefenderLeft& positioningSymbols)
{
  // stay in one spot if ball is far away from own goal
  getStandardPosition(positioningSymbols);

  // only check for alternative Position when in playing, otherwise keep standard position
  if (theGameInfo.state == STATE_PLAYING)
  {
    // if set play is ongoing, get to predefined position
    if (theGameInfo.setPlay != SET_PLAY_NONE)
    {
      if (calculateSetPlayPosition(positioningSymbols))
        return;
    }
    // if ball gets too close cover way to own goal
    if (theBallSymbols.ballPositionField.x() < 0.3f * theFieldDimensions.xPosOwnGroundline)
      getDefensivePosition(positioningSymbols);

    // move the optimal Position a bit if you would get too close to the ballchaser position
    avoidBallchaserConflict(positioningSymbols);

    if (theGameSymbols.avoidCenterCircle)
    {
      if (Geometry::getDistanceToLine(Geometry::Line(theRobotPoseAfterPreview.translation, positioningSymbols.optPosition.translation - theRobotPoseAfterPreview.translation), Vector2f::Zero())
          < theFieldDimensions.centerCircleRadius + 350.f)
        positioningSymbols.optPosition.translation = theRobotPoseAfterPreview.translation;
    }

    // positioningSymbols.optPosition.rotation = (theBallSymbols.ballPositionField - positioningSymbols.optPosition.translation).angle();
  }
}

/**
* \brief Calculate defender Position for situation where the ball is not close to the own goal.
*/
void DefenderLeftProvider::getStandardPosition(DefenderLeft& positioningSymbols)
{
  positioningSymbols.optPosition = Pose2f(0_deg, 0.8f * theFieldDimensions.xPosOwnGroundline, 0.5f * theFieldDimensions.yPosLeftGoal);
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
}

/**
* \brief Standard position if ball is not close, else coordinate with defenderRight
*
* If ball is not close and we have to move, use standard position.
* If ball is close, move to left intersection of circle around ball and defense line.
*/
bool DefenderLeftProvider::calculateSetPlayPosition(DefenderLeft& positioningSymbols)
{
  // start with the default position and avoid if necessary
  // predicted is not necessary since ball does not move in set play
  getStandardPosition(positioningSymbols);

  bool useStandardPosition = true;

  // try to keep 800 mm distance
  const float minDistanceToBall = 800.f;
  // try not to touch penalty area line, since defender right is also supposed to be there
  const float distanceFromPenaltyLine = 250.f;
  Vector2f vectorFromBallPosition = (theRobotPoseAfterPreview.translation - theBallSymbols.ballPositionField);
  float distanceToBallPosition = vectorFromBallPosition.norm();
  if (distanceToBallPosition < minDistanceToBall)
  {
    // left side of the ball-> all good
    if (theBallSymbols.ballPositionField.y() < theRobotPoseAfterPreview.translation.y())
    {
      Geometry::Line penaltyLine;
      penaltyLine.base = Vector2f(theFieldDimensions.xPosOwnPenaltyArea + distanceFromPenaltyLine, 0.f);
      penaltyLine.direction = Vector2f(0.f, 1.f);
      Vector2f firstIntersection, secondIntersection;
      int noOfIntersections = Geometry::getIntersectionOfLineAndCircle(penaltyLine, Geometry::Circle(theBallSymbols.ballPositionField, minDistanceToBall), firstIntersection, secondIntersection);
      if (noOfIntersections < 2)
        getStandardPosition(positioningSymbols);
      else
      {
        useStandardPosition = false;
        bool useFirstIntersection = firstIntersection.y() > secondIntersection.y();
        // since we are the left defender, use left intersection point
        positioningSymbols.optPosition.translation = useFirstIntersection ? firstIntersection : secondIntersection;
        // pull at least 30 cm apart, since other point is taken by defenderRight
        const float distanceToOtherIntersection = 300.f;
        float newYPosition = useFirstIntersection ? secondIntersection.y() + distanceToOtherIntersection : firstIntersection.y() + distanceToOtherIntersection;
        positioningSymbols.optPosition.translation.y() = std::max(positioningSymbols.optPosition.translation.y(), newYPosition);
      }
    }
    else // wrong side of the ball for default behavior
    {
      useStandardPosition = false;
      positioningSymbols.optPosition.translation = theRobotPoseAfterPreview.translation + vectorFromBallPosition.normalize(minDistanceToBall);
      positioningSymbols.optPosition.rotation = (positioningSymbols.optPosition.translation - theBallSymbols.ballPositionField).angle();
    }
  }
  positioningSymbols.thresholdRotation = 10_deg;
  positioningSymbols.thresholdXBack = 50.f;
  positioningSymbols.thresholdXFront = 100.f;
  positioningSymbols.thresholdY = 100.f;
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
  return !useStandardPosition;
}

/**
* \brief Place position on a fixed line so that way from ball to goal is blocked.
*
* The line is given by two positions, innerPosition (central area in front of the goal) and outerPosition (close to the field corner).
* The intersection between the line and the path from ball to goal is calculated and if the intersection lies between the
* inner and outer position the robot moves to the intersection point. If the intersection does not lie in between those
* positions the robot moves to the position among the two thats closer to the intersection point.
*/
void DefenderLeftProvider::getDefensivePosition(DefenderLeft& positioningSymbols)
{
  // the defensive position will be somewhere in between these two points
  Vector2f innerPosition(theFieldDimensions.xPosOwnPenaltyMark, 0.5f * theFieldDimensions.yPosLeftGoal);
  Vector2f outerPosition(0.8f * theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea);
  Vector2f vOuterToInner = innerPosition - outerPosition;

  Vector2f leftGoalHalfPosition(theFieldDimensions.xPosOwnGroundline, 0.5f * theFieldDimensions.yPosLeftGoal);
  Vector2f vBallToGoal = leftGoalHalfPosition - theBallSymbols.ballPositionField;
  // construct poses to define lines from one point to another
  Pose2f pBallToGoal(vBallToGoal.angle(), theBallSymbols.ballPositionField);
  Pose2f pOuterToInner(vOuterToInner.angle(), outerPosition);
  // calculate intersection of the positioning line and the path from ball to goal
  Geometry::Line lineOuterToInnerPosition(pOuterToInner, vOuterToInner.norm());
  Geometry::Line lineBallToGoal(pBallToGoal, vBallToGoal.norm());
  Vector2f lineIntersection;
  bool intersectionFound = Geometry::getIntersectionOfLines(lineOuterToInnerPosition, lineBallToGoal, lineIntersection);

  // since lines have infinite length the intersection point does not have to lie between innerPosition and outerPosition
  // to prevent the robot from moving somewhere unintended it will be checked if the intersection is between the two points
  float dOuterInner = (innerPosition - outerPosition).norm();
  float dIntersectionOuter = (outerPosition - lineIntersection).norm();
  float dIntersectionInner = (innerPosition - lineIntersection).norm();
  float dBallOuter = (theBallSymbols.ballPositionField - outerPosition).norm();
  float dBallInner = (theBallSymbols.ballPositionField - innerPosition).norm();
  // a point on line is between two points if its not farther away from both then they are to eachother
  bool isIntersectionBetweenOuterInnerPosition = (intersectionFound && dIntersectionOuter <= dOuterInner && dIntersectionInner <= dOuterInner);

  // do not move to the intersection if it is not in between the outer and inner positions
  // instead move to the either outer or inner, whichever is closer to the intersection
  if (!isIntersectionBetweenOuterInnerPosition)
  {
    // ball is closer to innerPosition => go to innerPosition
    if (dBallInner < dBallOuter)
    {
      Angle anglePositionToBall = (theBallSymbols.ballPositionField - innerPosition).angle();
      positioningSymbols.optPosition = Pose2f(anglePositionToBall, innerPosition);
    }
    // intersection is closer to outerPosition => go to outerPosition
    else
    {
      Angle anglePositionToBall = (theBallSymbols.ballPositionField - outerPosition).angle();
      positioningSymbols.optPosition = Pose2f(anglePositionToBall, outerPosition);
    }
  }
  else
  {
    Angle anglePositionToBall = (theBallSymbols.ballPositionField - lineIntersection).angle();
    positioningSymbols.optPosition = Pose2f(anglePositionToBall, lineIntersection);
  }
  positioningSymbols.stopAtTarget = true;
  positioningSymbols.previewArrival = true;
}

/**
* \brief Move the calculated optPosition if it conflicts with the ballchasers optimal position.
*
* The ballchaser always has priority to get to its position so the other roles should actively try to
* stay out of its way.
*/
void DefenderLeftProvider::avoidBallchaserConflict(DefenderLeft& positioningSymbols)
{
  Vector2f optimalBallchaserPosition = theBallchaser.optPosition.translation;
  Vector2f optimalPosition = positioningSymbols.optPosition.translation;
  Vector2f bcPositionToOptPosition = optimalPosition - optimalBallchaserPosition;
  float distanceBetweenPositions = bcPositionToOptPosition.norm();
  // come up with alternative position if too close to the ballchaser's optimal position
  if (distanceBetweenPositions < theBehaviorConfiguration.behaviorParameters.positionConflictDistance)
  {
    // position to the left of original position to avoid ballchaser
    positioningSymbols.optPosition.translation.y() += (theBehaviorConfiguration.behaviorParameters.positionConflictDistance);
    positioningSymbols.stopAtTarget = true;
    positioningSymbols.previewArrival = true;
  }
}


MAKE_MODULE(DefenderLeftProvider, behaviorControl)