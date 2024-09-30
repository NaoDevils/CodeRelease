#pragma once

#include "ExecutableShot.h"
#include <vector>

class ExecutablePlan
{

public:
  ExecutablePlan(const float futureMultiplier) : futureMultiplier(futureMultiplier) {}

  void addAtFront(const ExecutableShot& executableKick) { executableKicks.insert(executableKicks.begin(), executableKick); }

  [[nodiscard]] bool isEmpty() const { return executableKicks.empty(); }

  [[nodiscard]] bool has_value() const { return !executableKicks.empty(); }

  [[nodiscard]] ExecutableShot& value() const { return const_cast<ExecutableShot&>(executableKicks.front()); }

  [[nodiscard]] float getScore(const int expectedDepth) const
  {
    ASSERT(expectedDepth > 0);
    float score = 0.f;
    for (int depth = 0; depth < expectedDepth; depth++)
    {
      const int index = std::min(depth, (int)executableKicks.size() - 1);
      const float rawScore = executableKicks.at(index).getScore();
      score += rawScore * std::pow(futureMultiplier, (float)depth);
    }
    return score;
  }

  void draw() const
  {
    for (const ExecutableShot& executableKick : executableKicks)
    {
      executableKick.selectablePose.selectableShot.selectableTarget.draw();
    }
  }

  std::vector<ExecutableShot> executableKicks;
  float futureMultiplier;

private:
};
