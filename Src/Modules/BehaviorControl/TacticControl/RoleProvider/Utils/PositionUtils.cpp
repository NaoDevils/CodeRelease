#include "PositionUtils.h"

Angle PositionUtils::getTurnRobotByAngle(const Pose2f& robotPose, const Vector2f& targetPosition)
{
  return Transformation::fieldToRobot(robotPose, targetPosition).angle();
}

Pose2f PositionUtils::getCurrentKickFootPose(const bool kickWithLeft, const BehaviorConfiguration& theBehaviorConfiguration, const RobotPoseAfterPreview& theRobotPoseAfterPreview)
{
  return {theRobotPoseAfterPreview.rotation, theRobotPoseAfterPreview * Vector2f{0.f, (kickWithLeft ? +1.f : -1.f) * theBehaviorConfiguration.optDistanceToBallY}};
}

Angle PositionUtils::getTurnToBallAngle(const Vector2f& ownPosition, const BallSymbols& theBallSymbols)
{
  return getTurnToPositionAngle(ownPosition, theBallSymbols.ballPositionFieldPredicted);
}

Angle PositionUtils::getTurnToPositionAngle(const Vector2f& ownPosition, const Vector2f& turnToPosition)
{
  return (turnToPosition - ownPosition).angle();
}

Vector2f PositionUtils::getPosition(const bool& xOnEnemySide, const float& x, const bool& yOnLeftSide, const float& y)
{
  return Vector2f(xOnEnemySide ? x : -x, yOnLeftSide ? y : -y);
}

Vector2f PositionUtils::getPosition(const float& x, const bool& yOnLeftSide, const float& y)
{
  return getPosition(true, x, yOnLeftSide, y);
}

Vector2f PositionUtils::getPosition(const bool& xOnEnemySide, const float& x, const float& y)
{
  return getPosition(xOnEnemySide, x, true, y);
}

bool PositionUtils::isPathBlocked(const Vector2f& startPosition, const Vector2f& targetPosition, const Vector2f& blockedPosition, const float& blockDistance)
{
  const Vector2f startToTargetVector = startPosition - targetPosition;
  const float fromPathToBlockedPositionDistance = Geometry::getDistanceToEdge(Geometry::Line(targetPosition, startToTargetVector), blockedPosition);
  return fromPathToBlockedPositionDistance < blockDistance;
}

// ===================================================================================================================

void PositionUtils::turnTowardsBall(PositioningSymbols& positioningSymbols, const BallSymbols& theBallSymbols)
{
  turnToPosition(positioningSymbols, theBallSymbols.ballPositionFieldPredicted);
}

void PositionUtils::turnToPosition(PositioningSymbols& positioningSymbols, const Vector2f& turnToPosition)
{
  positioningSymbols.optPosition.rotation = getTurnToPositionAngle(positioningSymbols.optPosition.translation, turnToPosition);
}

void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const bool& xOnEnemySide, const float& x, const bool& yOnLeftSide, const float& y)
{
  positioningSymbols.optPosition.translation = getPosition(xOnEnemySide, x, yOnLeftSide, y);
}

void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const float& x, const bool& yOnLeftSide, const float& y)
{
  setPosition(positioningSymbols, true, x, yOnLeftSide, y);
}

void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const bool& xOnEnemySide, const float& x, const float& y)
{
  setPosition(positioningSymbols, xOnEnemySide, x, true, y);
}

void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const float& x, const float& y)
{
  positioningSymbols.optPosition.translation.x() = x;
  positioningSymbols.optPosition.translation.y() = y;
}

void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const Vector2f& pos)
{
  positioningSymbols.optPosition.translation = pos;
}
