#pragma once

#include <optional>
#include <Representations/Modeling/DangerMap.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/GameSymbols.h"

class DangerUtils
{

public:
  static bool isDanger(
      const Vector2f& playerPosition, const Vector2f& ballPosition, const float distanceMultiplier, const bool hysteresis, const DangerMap& theDangerMap, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap)
  {
    if (isThereATroublemaker(ballPosition, distanceMultiplier, hysteresis, theRobotMap))
    {
      return true;
    }

    const float dangerDistance = distanceMultiplier * (hysteresis ? 1.2f : 1.f) * 500.f; // TODO Constant
    const float dangerThreshold = 0.1f; // TODO Constant
    if (theDangerMap.getDangerAt(ballPosition, theFieldDimensions, dangerDistance) > dangerThreshold)
    {
      return true;
    }

    return false;
  }

  static bool isThereATroublemaker(const Vector2f& ballPosition, const float distanceMultiplier, const bool hysteresis, const RobotMap& theRobotMap)
  {
    const float dangerRadiusFrontX = distanceMultiplier * (hysteresis ? 1.2f : 1.f) * 600.f; // TODO Constants
    const float dangerRadiusBackX = dangerRadiusFrontX;
    const float danderRadiusY = dangerRadiusFrontX;

    const Vector2f rectangleBottomLeftCorner = ballPosition - Vector2f(dangerRadiusBackX, danderRadiusY);
    const Vector2f rectangleTopRightCorner = ballPosition + Vector2f(dangerRadiusFrontX, danderRadiusY);

    return isDangerInRectangle(rectangleBottomLeftCorner, rectangleTopRightCorner, theRobotMap);
  }

  static bool isThereATroublemaker(const Vector2f& ballPosition, const Angle& ballToTargetAngle, const RobotMap& theRobotMap)
  {
    const float dangerRadiusBackX = 300.f; // TODO Constants
    const float dangerRadiusFrontX = 1000.f; // TODO Constants
    const float danderRadiusY = 800.f; // TODO Constants

    const Vector2f rectangleBottomLeftCorner = ballPosition - Vector2f(dangerRadiusBackX, danderRadiusY).rotate(ballToTargetAngle);
    const Vector2f rectangleTopRightCorner = ballPosition + Vector2f(dangerRadiusFrontX, danderRadiusY).rotate(ballToTargetAngle);

    return isDangerInRectangle(rectangleBottomLeftCorner, rectangleTopRightCorner, theRobotMap);
  }

private:
  static bool isDangerInRectangle(const Vector2f& rectangleBottomLeftCorner, const Vector2f& rectangleTopRightCorner, const RobotMap& theRobotMap)
  {
    const Vector2f topLeftCorner = {rectangleBottomLeftCorner.x(), rectangleTopRightCorner.y()};
    const Vector2f bottomRightCorner = {rectangleTopRightCorner.x(), rectangleBottomLeftCorner.y()};
    LINE(DRAW_KICK_DANGER_NAME, rectangleBottomLeftCorner.x(), rectangleBottomLeftCorner.y(), topLeftCorner.x(), topLeftCorner.y(), 50, Drawings::solidPen, ColorRGBA::orange);
    LINE(DRAW_KICK_DANGER_NAME, topLeftCorner.x(), topLeftCorner.y(), rectangleTopRightCorner.x(), rectangleTopRightCorner.y(), 50, Drawings::solidPen, ColorRGBA::orange);
    LINE(DRAW_KICK_DANGER_NAME, rectangleTopRightCorner.x(), rectangleTopRightCorner.y(), bottomRightCorner.x(), bottomRightCorner.y(), 50, Drawings::solidPen, ColorRGBA::orange);
    LINE(DRAW_KICK_DANGER_NAME, bottomRightCorner.x(), bottomRightCorner.y(), rectangleBottomLeftCorner.x(), rectangleBottomLeftCorner.y(), 50, Drawings::solidPen, ColorRGBA::orange);

    bool trouble = false;

    for (auto& robot : theRobotMap.robots)
    {
      const Vector2f position = robot.pose.translation;

      if (robot.robotType == RobotEstimate::teammateRobot)
      {
        continue;
      }

      if (Geometry::isPointInsideRectangle(rectangleBottomLeftCorner, rectangleTopRightCorner, position))
      {
        trouble = true;
      }
    }
    return trouble;
  }
};
