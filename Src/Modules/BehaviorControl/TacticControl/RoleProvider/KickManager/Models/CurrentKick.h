#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/KickPlan.h"
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

  [[nodiscard]] KickPlan toKickPlan(const Pose2f& playerPose, const Vector2f& ballPosition) const
  {
    SelectableKick selectableKick = {kick};
    SelectableTarget selectableTarget = {selectableKick, ballPosition, target};
    Pose2f kickPose = kick->getKickPose(ballPosition, target, kickWithLeft);
    SelectablePose selectablePose = {playerPose, selectableTarget, kickPose, kickWithLeft};
    return {selectablePose, true};
  }

  Vector2f target;
  Kick* kick;
  bool kickWithLeft;
};
