#pragma once

#include <Representations/BehaviorControl/RoleSymbols/Ballchaser.h>

#include <utility>
#include <Representations/Modeling/HeatMapCollection.h>
#include "Tools/Math/Eigen.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/DistanceRequirement.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"

class ExecutableKick
{
public:
  ExecutableKick(Kick* kick, Vector2f target, const bool hysteresis) : kick(kick), target(std::move(target)), hysteresis(hysteresis) {}
  virtual ~ExecutableKick() = default;

  void setWidth(const float newWidth)
  {
    width = newWidth;
    widthSet = true;
  }

  [[nodiscard]] float getWidth() const
  {
    if (!widthSet)
    {
      throw std::logic_error("Calculate width before check if blocked!");
    }
    return width;
  }

  [[nodiscard]] bool isWidthSet() const { return widthSet; }

  friend std::ostream& operator<<(std::ostream& os, const ExecutableKick& e);

  [[nodiscard]] std::string toString() const
  {
    std::stringstream ss;
    ss << (*this);
    return ss.str();
  }

  Kick* kick;
  Vector2f target;
  bool hysteresis;

  float width = 0;
  bool widthSet = false;
};
