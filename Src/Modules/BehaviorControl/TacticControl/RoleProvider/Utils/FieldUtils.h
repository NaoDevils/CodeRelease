#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "MathUtils.h"

class FieldUtils
{
public:
  static bool isIllegal(const Vector2f& position, const FieldDimensions& theFieldDimensions)
  {
    if (position.x() < theFieldDimensions.xPosOwnFieldBorder)
      return true;
    if (position.x() > theFieldDimensions.xPosOpponentFieldBorder)
      return true;
    if (position.y() < theFieldDimensions.yPosRightFieldBorder)
      return true;
    if (position.y() > theFieldDimensions.yPosLeftFieldBorder)
      return true;
    return false;
  }

  static Vector2f clamp(const Vector2f& position, const FieldDimensions& theFieldDimensions)
  {
    const float x = MathUtils::clamp_f(position.x(), theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.xPosOpponentFieldBorder);
    const float y = MathUtils::clamp_f(position.y(), theFieldDimensions.yPosRightFieldBorder, theFieldDimensions.yPosLeftFieldBorder);
    return {x, y};
  }

  static float getMaxDistanceOnField(const FieldDimensions& theFieldDimensions)
  {
    const Vector2f cornerLeftOpponent = {theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftFieldBorder};
    const Vector2f cornerRightOwn = {theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightFieldBorder};
    return (cornerLeftOpponent - cornerRightOwn).norm();
  }

  static Vector2f getOpponentGoalLeftPost(const FieldDimensions& theFieldDimensions)
  {
    return Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal);
  }

  static Vector2f getOpponentGoalRightPost(const FieldDimensions& theFieldDimensions)
  {
    return Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal);
  }

  static Vector2f getOpponentGoalCenter(const FieldDimensions& theFieldDimensions)
  {
    return Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
  }

  static Vector2f getOwnGoalLeftPost(const FieldDimensions& theFieldDimensions) { return Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal); }

  static Vector2f getOwnGoalRightPost(const FieldDimensions& theFieldDimensions) { return Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal); }

  static Vector2f getOwnGoalCenter(const FieldDimensions& theFieldDimensions) { return Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal); }

  static bool isOnOwnSide(const Vector2f& position) { return isOnOwnSide(position.x()); }

  static bool isOnOwnSide(const float x) { return x < 0; }

  static bool isIntoOpponentsGoal(const Vector2f& origin, const Vector2f& target, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal};
    const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal};
    return Geometry::checkIntersectionOfLines(origin, target, leftGoalPostPosition, rightGoalPostPosition);
  }

  static bool isInDirectionOfOpponentsGoal(const Vector2f& origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f target = origin + 2 * theFieldDimensions.xPosOpponentGroundline * direction;
    return isIntoOpponentsGoal(origin, target, theFieldDimensions);
  }

  static float getDistanceToGoalLine(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    float goalPostInnerOffset = 10.f; // TODO Constant

    // TODO: check free space in goal
    const Vector2f leftGoalPost{theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal - goalPostInnerOffset};
    const Vector2f rightGoalPost{theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal + goalPostInnerOffset};

    const Geometry::Line goalLine{rightGoalPost, leftGoalPost - rightGoalPost};
    return Geometry::getDistanceToEdge(goalLine, ballPosition);
  }

  static Vector2f getLeftAngleForGoalKickTooSharpPosition(const float limitY, const Vector2f& leftGoalPost, const FieldDimensions& theFieldDimensions)
  {
    const Angle MAX_ANGLE = 74_deg; // TODO Constant

    const Geometry::Line leftFieldLine = {Vector2f(theFieldDimensions.xPosOpponentFieldBorder, std::min(limitY, theFieldDimensions.yPosLeftSideline)), Vector2f(-1.f, 0.f)};
    const Geometry::Line leftAngleLine = {leftGoalPost, MathUtils::angleToVector(-MAX_ANGLE)};
    Vector2f leftOutsideIntersection;
    VERIFY(Geometry::getIntersectionOfLines(leftFieldLine, leftAngleLine, leftOutsideIntersection));
    return leftOutsideIntersection;
  }

  static Vector2f getRightAngleForGoalKickTooSharpPosition(const float limitY, const Vector2f& rightGoalPost, const FieldDimensions& theFieldDimensions)
  {
    const Angle MAX_ANGLE = 74_deg; // TODO Constant

    const Geometry::Line rightFieldLine = {Vector2f(theFieldDimensions.xPosOpponentFieldBorder, std::max(limitY, theFieldDimensions.yPosRightSideline)), Vector2f(-1.f, 0.f)};
    const Geometry::Line rightAngleLine = {rightGoalPost, MathUtils::angleToVector(MAX_ANGLE)};
    Vector2f rightOutsideIntersection;
    VERIFY(Geometry::getIntersectionOfLines(rightFieldLine, rightAngleLine, rightOutsideIntersection));
    return rightOutsideIntersection;
  }
};
