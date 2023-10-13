#pragma once

#include <utility>
#include <optional>
#include <vector>
#include <functional>
#include "SelectableTarget.h"

class SelectablePose
{

public:
  SelectablePose(const Pose2f& playerPose, const SelectableTarget& selectableTarget, const Pose2f& pose, const bool kickWithLeft)
      : playerPose(playerPose), selectableTarget(std::move(selectableTarget)), pose(std::move(pose)), kickWithLeft(kickWithLeft)
  {
  }

  Pose2f playerPose;
  SelectableTarget selectableTarget;
  Pose2f pose;
  bool kickWithLeft;
  float score;
};
