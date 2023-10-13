#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/DistanceRequirement.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/KickPlan.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Tools/Math/Eigen.h"
#include <optional>
#include <utility>

class KickPlan;
class KickRange
{

public:
  explicit KickRange(const Vector2f& ballPosition) : ballPosition(ballPosition) {}

  [[nodiscard]] virtual bool istValidDirection(const Angle& angle) const = 0;

  [[nodiscard]] virtual bool istValidTarget(const Vector2f& target) const = 0;

  virtual void draw() const = 0;

  static void draw(const Vector2f& ballPosition, const Geometry::Line& line, const int minAhead, const int maxAhead)
  {
    const Vector2f& target1 = line.base + line.direction * minAhead;
    const Vector2f& target2 = line.base + line.direction * maxAhead;
    draw(ballPosition, target1);
    draw(ballPosition, target2);
  }

  static void draw(const Vector2f& ballPosition, const Vector2f& target)
  {
    LINE(DRAW_KICK_RANGE_NAME, ballPosition.x(), ballPosition.y(), target.x(), target.y(), 20, Drawings::solidPen, ColorRGBA::blue);
  }

protected:
  const Vector2f ballPosition;
};
