#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/ExecutableShot.h"
#include "Tools/Math/Geometry.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class CurrentKick
{

public:
  CurrentKick() : target(Vector2f()), kick(nullptr), kickWithLeft(false) {}

  CurrentKick(const Vector2f& target, Kick* kick, const bool kickWithLeft) : target(std::move(target)), kick(kick), kickWithLeft(kickWithLeft) {}

  Vector2f target;
  Kick* kick;
  bool kickWithLeft;
};
