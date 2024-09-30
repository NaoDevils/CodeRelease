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

  static bool isIntoOwnGoal(const Vector2f& origin, const Vector2f& target, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal};
    const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal};
    return Geometry::checkIntersectionOfLines(origin, target, leftGoalPostPosition, rightGoalPostPosition);
  }

  static bool isInDirectionOfOpponentsGoal(const Vector2f& insideField_origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f lookingFromCenter_leftGoalPost = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal};
    const Vector2f lookingFromCenter_rightGoalPost = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal};
    return isInDirectionOfGoal(insideField_origin, direction, lookingFromCenter_leftGoalPost, lookingFromCenter_rightGoalPost);
  }

  static bool isInDirectionOfOwnGoal(const Vector2f& insideField_origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    const Vector2f lookingFromCenter_leftGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal};
    const Vector2f lookingFromCenter_rightGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal};
    return isInDirectionOfGoal(insideField_origin, direction, lookingFromCenter_leftGoalPost, lookingFromCenter_rightGoalPost);
  }

  static bool isInDirectionOfGoal(const Vector2f& insideField_origin, const Vector2f& direction, const Vector2f& lookingFromCenter_leftGoalPost, const Vector2f& lookingFromCenter_rightGoalPost)
  {
    const Angle originToTargetAngle = direction.angle();
    const Angle originToLeftGoalPostAngle = (lookingFromCenter_leftGoalPost - insideField_origin).angle();
    const Angle originToRightGoalPostAngle = (lookingFromCenter_rightGoalPost - insideField_origin).angle();
    if (MathUtils::getLeftToRightAngleDiff(originToLeftGoalPostAngle, originToRightGoalPostAngle) > 270)
    {
      // ball is on
    }
    return MathUtils::isBetweenAngles(originToTargetAngle, originToLeftGoalPostAngle, originToRightGoalPostAngle);
  }

  /**
   * @return a negative value if it is not into the direction of the goal or behind the goal
   */
  static float getDistanceToOpponentsGoal(const Vector2f& origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    return getDistanceToGoal(false, origin, direction, theFieldDimensions);
  }

  /**
   * @return a negative value if it is not into the direction of the goal or behind the goal
   */
  static float getDistanceToOwnGoal(const Vector2f& origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    return getDistanceToGoal(true, origin, direction, theFieldDimensions);
  }

  /**
   * @return a negative value if it is not into the direction of the goal. Switch goal posts if behind the goal
   */
  static float getDistanceToGoal(const bool ownSide, const Vector2f& origin, const Vector2f& direction, const FieldDimensions& theFieldDimensions)
  {
    const bool isBehindGroundLine = ownSide ? origin.x() < theFieldDimensions.xPosOwnGroundline : origin.x() > theFieldDimensions.xPosOpponentGroundline;

    if (isBehindGroundLine && (origin.y() > theFieldDimensions.yPosLeftGoal || theFieldDimensions.yPosRightGoal > origin.y()))
    {
      return -1.f; // ball cant be kicked through the net
    }

    const float BALL_RADIUS = 5.f; // todo
    const float GOAL_POST_RADIUS = theFieldDimensions.goalPostRadius;
    const float xPadding = 2.f * GOAL_POST_RADIUS;
    const float yPadding = (BALL_RADIUS + GOAL_POST_RADIUS) * 1.1f;

    Vector2f lookingFromCenter_leftGoalPost;
    Vector2f lookingFromCenter_rightGoalPost;
    if (ownSide)
    {
      lookingFromCenter_leftGoalPost = {theFieldDimensions.xPosOwnGroundline + xPadding, theFieldDimensions.yPosRightGoal - yPadding}; // use padding to make own goal bigger
      lookingFromCenter_rightGoalPost = {theFieldDimensions.xPosOwnGroundline + xPadding, theFieldDimensions.yPosLeftGoal + yPadding};
    }
    else
    {
      lookingFromCenter_leftGoalPost = {theFieldDimensions.xPosOpponentGroundline + xPadding, theFieldDimensions.yPosLeftGoal - yPadding}; // use padding to make opponents goal smaller
      lookingFromCenter_rightGoalPost = {theFieldDimensions.xPosOpponentGroundline + xPadding, theFieldDimensions.yPosRightGoal + yPadding};
    }

    if (isInDirectionOfGoal(origin, direction, lookingFromCenter_leftGoalPost, lookingFromCenter_rightGoalPost))
    {
      const Geometry::Line goalLine = {lookingFromCenter_leftGoalPost, lookingFromCenter_rightGoalPost - lookingFromCenter_leftGoalPost};
      const Geometry::Line kickLine = {origin, direction};
      if (isBehindGroundLine)
      {
        return 1.f; // ball is detected inside goal but keep playing by setting the distance to the goal line to a minimum
      }
      Vector2f intersection;
      VERIFY(Geometry::getIntersectionOfLines(goalLine, kickLine, intersection));
      return Geometry::distance(origin, intersection);
    }
    else
    {
      return -1.f;
    }
  }

  static float getDistanceToOpponentsGoal(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
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
