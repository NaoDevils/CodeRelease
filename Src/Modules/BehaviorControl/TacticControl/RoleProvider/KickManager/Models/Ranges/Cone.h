#pragma once

#include <optional>
#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h"

STREAMABLE(Cone,

  Cone(const Angle& left, const Angle& right)
  {
    this->left = left;
    this->right = right;
  };

  Cone()
  {
    this->left = 180_deg;
    this->right = -180_deg;
  };

  [[nodiscard]] bool istInside(const Angle& angle) const
  {
    return MathUtils::isBetweenAngles(angle, left, right);
  };

  void draw(const Vector2f& position, const float length) const
  {
    const Vector2f leftPosition = position + length * MathUtils::angleToVector(left);
    const Vector2f rightPosition = position + length * MathUtils::angleToVector(right);
    LINE(DRAW_KICK_RANGE_NAME, position.x(), position.y(), leftPosition.x(), leftPosition.y(), 20, Drawings::solidPen, ColorRGBA::blue);
    LINE(DRAW_KICK_RANGE_NAME, position.x(), position.y(), rightPosition.x(), rightPosition.y(), 20, Drawings::solidPen, ColorRGBA::blue);
  },

  (Angle)(0) left,
  (Angle)(0) right
);