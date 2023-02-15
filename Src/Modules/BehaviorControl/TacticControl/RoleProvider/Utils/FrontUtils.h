#pragma once

#include "TeamUtils.h"
#include "PositionUtils.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotMap.h"

class FrontUtils
{
public:
  static void freeSight(Vector2f& position, const RobotMap& theRobotMap, const Vector2f& freeSightOnPosition, const float& maxDistanceFromCurrentPosition)
  {
    const int MAX_STEPS = 10;
    const float checkStepsWidth = 30.f;
    const float freeSigtDistance = 200.f;

    Vector2f toSightVector = freeSightOnPosition - position;
    toSightVector.normalize();
    const Vector2f toRoboRightNormVector = turnLeft(toSightVector);

    Vector2f testPosition = Vector2f(position);
    bool freeSight = true;

    for (int left = -1; left <= 1; left = left + 2)
    {
      for (int i = 0; i <= MAX_STEPS; i++) // start with 0 to test current posisiton
      {
        if (i == 0 && left == 1) // skip second test of current position
        {
          continue;
        }

        for (auto& robot : theRobotMap.robots)
        {
          testPosition = Vector2f(position);
          testPosition += left * i * toRoboRightNormVector * checkStepsWidth;

          if (PositionUtils::isPathBlocked(testPosition, freeSightOnPosition, robot.pose.translation, freeSigtDistance))
          {
            freeSight = false;
            break;
          }
        }
      }
    }

    if (freeSight)
    {
      position.x() = testPosition.x();
      position.y() = testPosition.y();
    }
  }

  static Vector2f turnLeft(const Vector2f& vec) // TODO Auslagern
  {
    return -turnRight(vec);
  }

  static Vector2f turnRight(const Vector2f& vec) // TODO Auslagern
  {
    return Vector2f(-vec.y(), vec.x());
  }
};
