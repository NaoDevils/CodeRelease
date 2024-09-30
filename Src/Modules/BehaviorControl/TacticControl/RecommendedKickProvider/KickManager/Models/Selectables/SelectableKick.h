#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"

class SelectableKick
{

public:
  explicit SelectableKick(Kick* kick) : kick(kick) {}
  Kick* kick;

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
  float score;
  bool scoreSet = false;
};
