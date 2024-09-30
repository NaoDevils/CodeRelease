#include "DirectionInfo.h"

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Tools/Debugging/DebugDrawings.h"

int DirectionInfo::getIndex(const Angle& angle) const
{
  ASSERT(-180_deg <= angle && angle <= 180_deg);
  ASSERT(angles.at(0) < angles.at(1));

  const int anglesSize = (int)angles.size();
  const Angle biggestAngle = angles.at(anglesSize - 1);
  const Angle secondBiggestAngle = angles.at(anglesSize - 2);
  const Angle stepSizeAngle = biggestAngle - secondBiggestAngle;

  const float stepsToAngle = (angle + 180_deg) / stepSizeAngle;
  const int fullStepsToAngle = (int)stepsToAngle;
  const bool toSmallerIndex = stepsToAngle - (float)fullStepsToAngle < 0.5f;
  if (toSmallerIndex)
  {
    const int index = fullStepsToAngle;
    ASSERT(index >= 0);
    ASSERT(index <= (int)angles.size() - 1);
    return index;
  }
  else
  {
    const int index = fullStepsToAngle + 1;
    ASSERT(index >= 0);
    ASSERT(index <= (int)angles.size() - 1);
    return index;
  }
}

void DirectionInfo::draw(const Vector2f& ballPosition) const
{
  const bool DRAW_RAYS_NOT_CONES = true;

  COMPLEX_DRAWING(DRAW_KICK_WHEEL)
  {
    if (DRAW_RAYS_NOT_CONES)
    {
      for (int i = 0; i < (int)angles.size(); ++i)
      {
        float fetchedDistance = blockedDistances.at(i) == -1 ? std::numeric_limits<float>::max() : blockedDistances.at(i);
        fetchedDistance = std::min(fetchedDistance, intoGoalKickOutsideDistances.at(i) == -1 ? std::numeric_limits<float>::max() : intoGoalKickOutsideDistances.at(i));
        fetchedDistance = std::min(fetchedDistance, intoKickInOutsideDistances.at(i) == -1 ? std::numeric_limits<float>::max() : intoKickInOutsideDistances.at(i));
        fetchedDistance = std::min(fetchedDistance, intoCornerKickOutsideDistances.at(i) == -1 ? std::numeric_limits<float>::max() : intoCornerKickOutsideDistances.at(i));
        fetchedDistance = std::min(fetchedDistance, intoOpponentGoalDistances.at(i) == -1 ? std::numeric_limits<float>::max() : intoOpponentGoalDistances.at(i));
        fetchedDistance = std::min(fetchedDistance, intoOwnGoalDistances.at(i) == -1 ? std::numeric_limits<float>::max() : intoOwnGoalDistances.at(i));

        if (fetchedDistance < 100.f)
        {
          continue;
        }
        const float distance = std::min(5000.f, fetchedDistance);

        const Angle angle = angles.at(i);
        const Vector2f direction = MathUtils::angleToVector(angle);

        const Vector2f target = ballPosition + direction * distance;

        LINE(DRAW_KICK_WHEEL, ballPosition.x(), ballPosition.y(), target.x(), target.y(), 10, Drawings::solidPen, ColorRGBA::black);
      }
    }
    else
    {
      Vector2f firstTarget = {};
      Vector2f lastTarget = {};
      const int maxI = (int)angles.size();
      for (int i = 0; i < maxI; ++i)
      {
        const Angle angle = angles.at(i);
        const Vector2f direction = MathUtils::angleToVector(angle);
        float fetchedDistance = intoCornerKickOutsideDistances.at(i);
        if (fetchedDistance < 100.f)
        {
          const float MIN_DRAW_DISTANCE = 100.f;
          OUTPUT_WARNING("Drawn distance for DirectionInfo is " << MIN_DRAW_DISTANCE << " the original distance was " << fetchedDistance);
          fetchedDistance = MIN_DRAW_DISTANCE;
        }
        const float distance = std::min(10000.f, fetchedDistance);
        const Vector2f target = ballPosition + direction * distance;
        if (i == 0)
        {
          firstTarget = target;
        }
        else
        {
          LINE(DRAW_KICK_WHEEL, lastTarget.x(), lastTarget.y(), target.x(), target.y(), 10, Drawings::solidPen, ColorRGBA::black);
        }
        lastTarget = target;
      }
      LINE(DRAW_KICK_WHEEL, firstTarget.x(), firstTarget.y(), lastTarget.x(), lastTarget.y(), 10, Drawings::solidPen, ColorRGBA::black);
    }
  }
}
