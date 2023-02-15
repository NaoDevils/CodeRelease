#pragma once

#include <Representations/BehaviorControl/BallSymbols.h>
#include <Representations/BehaviorControl/PositioningSymbols.h>
#include <Representations/BehaviorControl/BehaviorConfiguration.h>
#include <Representations/Modeling/RobotPose.h>

class PositionUtils
{
public:
  static Angle getTurnRobotByAngle(const Pose2f& robotPose, const Vector2f& targetPosition);

  static Pose2f getCurrentKickFootPose(const bool kickWithLeft, const BehaviorConfiguration& theBehaviorConfiguration, const RobotPoseAfterPreview& theRobotPoseAfterPreview);

  static Angle getTurnToBallAngle(const Vector2f& ownPosition, const BallSymbols& theBallSymbols);

  static Angle getTurnToPositionAngle(const Vector2f& ownPosition, const Vector2f& turnToPosition);

  static Vector2f getPosition(const bool& xOnEnemySide, const float& x, const bool& yOnLeftSide, const float& y);

  static Vector2f getPosition(const float& x, const bool& yOnLeftSide, const float& y);

  static Vector2f getPosition(const bool& xOnEnemySide, const float& x, const float& y);

  static bool isPathBlocked(const Vector2f& startPosition, const Vector2f& targetPosition, const Vector2f& blockedPosition, const float& blockDistance);

  // ===================================================================================================================

  static void turnTowardsBall(PositioningSymbols& positioningSymbols, const BallSymbols& theBallSymbols);

  static void turnToPosition(PositioningSymbols& positioningSymbols, const Vector2f& turnToPosition);

  static void setPosition(PositioningSymbols& positioningSymbols, const bool& xOnEnemySide, const float& x, const bool& yOnLeftSide, const float& y);

  static void setPosition(PositioningSymbols& positioningSymbols, const float& x, const bool& yOnLeftSide, const float& y);

  static void setPosition(PositioningSymbols& positioningSymbols, const bool& xOnEnemySide, const float& x, const float& y);

  static void setPosition(PositioningSymbols& positioningSymbols, const float& x, const float& y);

  static void setPosition(PositioningSymbols& positioningSymbols, const Vector2f& pos);
};
