#pragma once

#include "SelectableKick.h"
#include "Tools/Math/Geometry.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class SelectableTarget
{

public:
  SelectableTarget(const SelectableKick selectableKick, const Vector2f& ballPosition, const Vector2f& target)
      : selectableKick(selectableKick), ballPosition(ballPosition), target(target)
  {
  }

  SelectableKick selectableKick;
  Vector2f ballPosition;
  Vector2f target;
  float score;
};
