#pragma once

#include "Tools/Math/Geometry.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"

class SimpleExecutableShot
{

public:
  SimpleExecutableShot(const Vector2f& ballPosition, Kick* kick, const Pose2f& kickPose, const bool kickWithLeft, const Vector2f& target)
      : ballPosition(ballPosition), kick(kick), kickPose(kickPose), kickWithLeft(kickWithLeft), target(target)
  {
  }

  const Vector2f& ballPosition;
  Kick* kick;
  const Pose2f& kickPose;
  const bool kickWithLeft;
  const Vector2f& target;
};
