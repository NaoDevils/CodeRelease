#pragma once

#include <Representations/BehaviorControl/BallSymbols.h>
#include <Representations/BehaviorControl/BehaviorConfiguration.h>
#include <Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h>
#include <Representations/Modeling/RobotPose.h>

class PositionUtils
{
public:
  static Angle getTurnRobotByAngle(const Pose2f& robotPose, const Vector2f& targetPosition);

  static Pose2f getCurrentKickFootPose(const bool kickWithLeft, const BehaviorConfiguration& theBehaviorConfiguration, const RobotPoseAfterPreview& theRobotPoseAfterPreview);

  static Angle getTurnToBallAngle(const Vector2f& ownPosition, const BallSymbols& theBallSymbols);

  static Angle getTurnToPositionAngle(const Vector2f& ownPosition, const Vector2f& turnToPosition);

  static Vector2f getPosition(const bool xOnEnemySide, const float x, const bool yOnLeftSide, const float y);

  static Vector2f getPosition(const float x, const bool yOnLeftSide, const float y);

  static Vector2f getPosition(const bool xOnEnemySide, const float x, const float y);

  static bool isPathBlocked(const Vector2f& startPosition, const Vector2f& targetPosition, const Vector2f& blockedPosition, const float blockDistance);

  // ===================================================================================================================

  static void turnTowardsBall(PositioningSymbols& positioningSymbols, const BallSymbols& theBallSymbols);

  static void turnToPosition(PositioningSymbols& positioningSymbols, const Vector2f& turnToPosition);

  static void setPosition(PositioningSymbols& positioningSymbols, const bool xOnEnemySide, const float x, const bool yOnLeftSide, const float y);

  static void setPosition(PositioningSymbols& positioningSymbols, const float x, const bool yOnLeftSide, const float y);

  static void setPosition(PositioningSymbols& positioningSymbols, const bool xOnEnemySide, const float x, const float y);

  static void setPosition(PositioningSymbols& positioningSymbols, const float x, const float y);

  static void setPosition(PositioningSymbols& positioningSymbols, const Vector2f& pos);
};

inline Angle PositionUtils::getTurnRobotByAngle(const Pose2f& robotPose, const Vector2f& targetPosition)
{
  return Angle::normalize((targetPosition - robotPose.translation).angle() - robotPose.rotation);
}

inline Pose2f PositionUtils::getCurrentKickFootPose(const bool kickWithLeft, const BehaviorConfiguration& theBehaviorConfiguration, const RobotPoseAfterPreview& theRobotPoseAfterPreview)
{
  return Pose2f(theRobotPoseAfterPreview.rotation, theRobotPoseAfterPreview * Vector2f{0.f, (kickWithLeft ? +1.f : -1.f) * theBehaviorConfiguration.optDistanceToBallY});
}

inline Angle PositionUtils::getTurnToBallAngle(const Vector2f& ownPosition, const BallSymbols& theBallSymbols)
{
  return getTurnToPositionAngle(ownPosition, theBallSymbols.ballPositionFieldPredicted);
}

inline Angle PositionUtils::getTurnToPositionAngle(const Vector2f& ownPosition, const Vector2f& turnToPosition)
{
  return (turnToPosition - ownPosition).angle();
}

inline Vector2f PositionUtils::getPosition(const bool xOnEnemySide, const float x, const bool yOnLeftSide, const float y)
{
  return Vector2f(xOnEnemySide ? x : -x, yOnLeftSide ? y : -y);
}

inline Vector2f PositionUtils::getPosition(const float x, const bool yOnLeftSide, const float y)
{
  return getPosition(true, x, yOnLeftSide, y);
}

inline Vector2f PositionUtils::getPosition(const bool xOnEnemySide, const float x, const float y)
{
  return getPosition(xOnEnemySide, x, true, y);
}

inline bool PositionUtils::isPathBlocked(const Vector2f& startPosition, const Vector2f& targetPosition, const Vector2f& blockedPosition, const float blockDistance)
{
  const Vector2f startToTargetVector = startPosition - targetPosition;
  const float fromPathToBlockedPositionDistance = Geometry::getDistanceToEdge(Geometry::Line(targetPosition, startToTargetVector), blockedPosition);
  return fromPathToBlockedPositionDistance < blockDistance;
}

// ===================================================================================================================

inline void PositionUtils::turnTowardsBall(PositioningSymbols& positioningSymbols, const BallSymbols& theBallSymbols)
{
  turnToPosition(positioningSymbols, theBallSymbols.ballPositionField);
}

inline void PositionUtils::turnToPosition(PositioningSymbols& positioningSymbols, const Vector2f& turnToPosition)
{
  positioningSymbols.optPosition.rotation = getTurnToPositionAngle(positioningSymbols.optPosition.translation, turnToPosition);
}

inline void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const bool xOnEnemySide, const float x, const bool yOnLeftSide, const float y)
{
  positioningSymbols.optPosition.translation = getPosition(xOnEnemySide, x, yOnLeftSide, y);
}

inline void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const float x, const bool yOnLeftSide, const float y)
{
  setPosition(positioningSymbols, true, x, yOnLeftSide, y);
}

inline void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const bool xOnEnemySide, const float x, const float y)
{
  setPosition(positioningSymbols, xOnEnemySide, x, true, y);
}

inline void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const float x, const float y)
{
  positioningSymbols.optPosition.translation.x() = x;
  positioningSymbols.optPosition.translation.y() = y;
}

inline void PositionUtils::setPosition(PositioningSymbols& positioningSymbols, const Vector2f& pos)
{
  positioningSymbols.optPosition.translation = pos;
}
