#include "KickWheel.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h>

/**
 * @param angle has to be normalized
 */
DistanceInfo KickWheel::getDistance(const Angle& angle, const bool hysteresis) const
{
  ASSERT(-180_deg <= angle && angle <= 180_deg);
  int smallerAngleIndex = 0;
  int biggerAngleIndex = (int)angles.size() - 1;
  int index = 0;
  for (const Angle& a : angles)
  {
    if (a < angle)
    {
      smallerAngleIndex = index;
      index++;
    }
    else
    {
      biggerAngleIndex = index;
      break;
    }
  }
  const float BLOCKED_MIN_DISTANCE = 10.f;
  float blockedDistance;
  float outsideDistance;
  if (hysteresis)
  {
    blockedDistance = std::max(blockedDistances.at(biggerAngleIndex), blockedDistances.at(smallerAngleIndex));
    outsideDistance = std::max(outsideDistances.at(biggerAngleIndex), outsideDistances.at(smallerAngleIndex));
  }
  else
  {
    blockedDistance = (blockedDistances.at(biggerAngleIndex) + blockedDistances.at(smallerAngleIndex)) / 2.f;
    outsideDistance = (outsideDistances.at(biggerAngleIndex) + outsideDistances.at(smallerAngleIndex)) / 2.f;
  }
  if (blockedDistance < outsideDistance)
  {
    return {std::max(0.f, blockedDistance - BLOCKED_MIN_DISTANCE), true, false};
  }
  else
  {
    return {outsideDistance, false, true};
  }
}

void KickWheel::draw(const Vector2f& ballPosition) const
{
  Vector2f firstTarget = {};
  Vector2f lastTarget = {};
  const int maxI = (int)angles.size();
  for (int i = 0; i < maxI; ++i)
  {
    const Angle angle = angles.at(i);
    const Vector2f direction = MathUtils::angleToVector(angle);
    const float fetchedDistance = blockedDistances.at(i);
    if (fetchedDistance < 100.f)
    {
      continue;
    }
    const float distance = std::min(5000.f, fetchedDistance);
    const Vector2f target = ballPosition + direction * distance;
    if (i == 0)
    {
      firstTarget = target;
    }
    else
    {
      LINE("behavior:KickWheelProvider:kickWheel", lastTarget.x(), lastTarget.y(), target.x(), target.y(), 10, Drawings::solidPen, ColorRGBA::black);
    }
    lastTarget = target;
  }
  LINE("behavior:KickWheelProvider:kickWheel", firstTarget.x(), firstTarget.y(), lastTarget.x(), lastTarget.y(), 10, Drawings::solidPen, ColorRGBA::black);
}
