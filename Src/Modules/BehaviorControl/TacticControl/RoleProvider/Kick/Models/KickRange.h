#pragma once

#include <utility>
#include <optional>
#include "DistanceRequirement.h"
#include "Tools/Math/Eigen.h"

class KickRange
{

public:
  KickRange(Vector2f ballPosition, const Angle angle1, const Angle angle2, const float targetsDistance, DistanceRequirement distanceRequirement)
      : ballPosition(std::move(ballPosition))
  {
    targets = {};
    targets.emplace_back(this->ballPosition + MathUtils::angleToVector(angle1).normalized() * targetsDistance);
    targets.emplace_back(this->ballPosition + MathUtils::angleToVector(angle2).normalized() * targetsDistance);
    distanceRequirements = {};
    distanceRequirements.push_back(distanceRequirement);
  }

  KickRange(Vector2f ballPosition, Vector2f target1, Vector2f target2, DistanceRequirement distanceRequirement) : ballPosition(std::move(ballPosition))
  {
    targets = {};
    targets.push_back(std::move(target1));
    targets.push_back(std::move(target2));
    distanceRequirements = {};
    distanceRequirements.push_back(distanceRequirement);
  }

  KickRange(Vector2f ballPosition, std::vector<Vector2f> targets, DistanceRequirement distanceRequirement) : ballPosition(std::move(ballPosition)), targets(std::move(targets))
  {
    const size_t count = this->targets.size() - 1;
    distanceRequirements = {};
    for (size_t i = 0; i < count; ++i)
    {
      distanceRequirements.push_back(distanceRequirement);
    }
  }

  KickRange(Vector2f ballPosition, std::vector<Vector2f> targets, std::vector<DistanceRequirement> distanceRequirements)
      : ballPosition(std::move(ballPosition)), targets(std::move(targets)), distanceRequirements(std::move(distanceRequirements))
  {
    ASSERT(this->targets.size() == this->distanceRequirements.size() + 1);
  }

  virtual ~KickRange() = default;

  Vector2f ballPosition;
  std::vector<Vector2f> targets;
  std::vector<DistanceRequirement> distanceRequirements;
};
