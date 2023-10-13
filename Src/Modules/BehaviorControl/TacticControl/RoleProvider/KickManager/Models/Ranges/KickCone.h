#pragma once

#include "KickRange.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickWithLeftUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/DistanceRequirement.h"
#include <optional>
#include <utility>

class KickCone : public KickRange
{

public:
  KickCone(const Vector2f& ballPosition, Angle left, Angle right) : KickRange(ballPosition), left(left), right(right) { draw(); }

  explicit KickCone(const Vector2f& ballPosition) : KickRange(ballPosition), left(180_deg), right(-180_deg) {}

  [[nodiscard]] bool istValidDirection(const Angle& angle) const override { return MathUtils::isBetweenAngles(angle, left, right); }

  [[nodiscard]] bool istValidTarget(const Vector2f& target) const override { return true; }

  void draw() const override
  {
    const Vector2f leftPosition = ballPosition + MathUtils::angleToVector(left) * 1000;
    const Vector2f rightPosition = ballPosition + MathUtils::angleToVector(right) * 1000;
    KickRange::draw(ballPosition, leftPosition);
    KickRange::draw(ballPosition, rightPosition);
  }

  Angle left;
  Angle right;
};