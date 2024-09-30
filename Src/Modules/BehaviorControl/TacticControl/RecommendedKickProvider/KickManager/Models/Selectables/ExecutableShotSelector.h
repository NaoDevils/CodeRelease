#pragma once

#include "ExecutableShot.h"
#include "../../../../../../../Tools/Math/Angle.h"
#include "../../../../../../../Tools/Math/Eigen.h"
#include "../../../../../../../Tools/Math/Geometry.h"
#include "vector"

class ExecutableShotSelector
{

public:
  void add(const ExecutableShot& es)
  {
    if (sameScoreEss.empty())
    {
      sameScoreEss.push_back(es);
      return;
    }

    const float scoreDiff = es.getScore() - sameScoreEss.front().getScore();

    const float EQUAL_CONSTANT = 0.0001f;

    if (scoreDiff > EQUAL_CONSTANT)
    {
      sameScoreEss.clear();
      sameScoreEss.push_back(es);
    }
    else if (scoreDiff < -EQUAL_CONSTANT)
    {
      return;
    }
    else
    {
      if (sameScoreEss.front().hysteresis)
      {
        return;
      }
      sameScoreEss.push_back(es);
    }
  }

  std::optional<ExecutableShot> get()
  {
    if (sameScoreEss.empty())
    {
      return {};
    }
    const int index = (int)std::roundf(((float)sameScoreEss.size() + 1.f) / 2.f - 1.f);
    return sameScoreEss.at(index);
  }

private:
  std::vector<ExecutableShot> sameScoreEss = {};
};