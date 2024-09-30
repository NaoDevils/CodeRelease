#pragma once

#include "Representations/Modeling/RobotMap.h"

class BlockedUtils
{
public:
  static Vector2f moveIfBlocked(
      const float xPos, const float yPos, float& framesSinceLastSideStep, const Vector2f& currentTarget, bool& blockedByRobot, float& activity, const RobotMap& theRobotMap, const BallSymbols& theBallSymbols)
  {
    bool pathBlocked = false;
    Vector2f newPosition;
    bool newPositionBlocked = false;
    for (auto& otherRobot : theRobotMap.robots)
    {
      if (PositionUtils::isPathBlocked(Vector2f(xPos, yPos), theBallSymbols.ballPositionField, otherRobot.pose.translation, 100.f))
      {
        CROSS("behavior:FrontWingProvider", otherRobot.pose.translation.x(), otherRobot.pose.translation.y(), 100, 30, Drawings::dottedPen, ColorRGBA::black);
        pathBlocked = true;
        Vector2f startToTargetVector = (otherRobot.pose.translation - Vector2f(xPos, yPos));
        if ((startToTargetVector.x() > 0 && yPos > 0) || (startToTargetVector.x() <= 0 && yPos <= 0))
        {
          newPosition = Vector2f(xPos, yPos) + (startToTargetVector.rotateLeft().normalized() * 500);
        }
        else
        {
          newPosition = Vector2f(xPos, yPos) + (startToTargetVector.rotateRight().normalized() * 500);
        }
        newPositionBlocked = PositionUtils::isPathBlocked(newPosition, theBallSymbols.ballPositionField, otherRobot.pose.translation, 100.f);
      }
    }

    float x;
    float y;
    if (!blockedByRobot && !pathBlocked)
    {
      blockedByRobot = false;
      x = xPos;
      y = yPos;
    }
    else if (!blockedByRobot && pathBlocked)
    {
      blockedByRobot = true;
      framesSinceLastSideStep = 0.f;
      if (!newPositionBlocked)
      {
        x = newPosition.x();
        y = newPosition.y();
        activity = 1.1f;
      }
      else
      {
        x = xPos;
        y = yPos;
      }
    }
    else
    {
      // path was blocked in last frame -> keep previous decision, unless new target is far away (ball has moved)
      if (Geometry::distance(Vector2f(xPos, yPos), currentTarget) < 700.f)
      {
        // path was blocked some time ago -> keep previous decision, unless 3 seconds have passed
        if (framesSinceLastSideStep > 90.f)
        {
          blockedByRobot = false;
          x = xPos;
          y = yPos;
        }
        else
        {
          blockedByRobot = true;
          x = currentTarget.x();
          y = currentTarget.y();
        }
      }
      else
      {
        blockedByRobot = false;
        x = xPos;
        y = yPos;
      }
    }
    return Vector2f(x, y);
  }
};
