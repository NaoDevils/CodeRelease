#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class SelectableKick
{

public:
  SelectableKick(Kick* kick) : kick(kick) {}

  Kick* kick;
  float score;
};
