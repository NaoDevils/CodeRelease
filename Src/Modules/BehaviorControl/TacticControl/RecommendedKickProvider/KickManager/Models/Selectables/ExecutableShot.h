#pragma once

#include "SelectablePose.h"

class ExecutableShot
{

public:
  ExecutableShot(const SelectablePose& selectablePose, const bool hysteresis) : selectablePose(std::move(selectablePose)), hysteresis(hysteresis) {}

  SelectablePose selectablePose;
  bool hysteresis;

  void setSuccessProbability(const float newSuccessProbability)
  {
    ASSERT(successProbability == -1.f);
    successProbability = newSuccessProbability;
  }

  [[nodiscard]] float getSuccessProbability() const
  {
    ASSERT(successProbability != -1.f);
    return successProbability;
  }

  void setScore(const float newScore)
  {
    ASSERT(!scoreSet);
    score = newScore;
    scoreSet = true;
  }

  [[nodiscard]] float getScore() const
  {
    ASSERT(scoreSet);
    return score;
  }

private:
  float successProbability = -1.f;
  float score = 0.f;
  bool scoreSet = false;
};
