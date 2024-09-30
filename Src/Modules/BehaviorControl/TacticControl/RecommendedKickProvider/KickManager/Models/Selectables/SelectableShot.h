#pragma once

#include "SelectableTarget.h"

class SelectableShot
{

public:
  explicit SelectableShot(const SelectableTarget& selectableTarget) : selectableTarget(selectableTarget) {}

  [[nodiscard]] bool isFiltered() const { return selectableTarget.isFiltered(); }

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

  const SelectableShot* leftSelectableShot = nullptr;
  SelectableTarget selectableTarget;
  const SelectableShot* rightSelectableShot = nullptr;

private:
  float score;
  bool scoreSet = false;
};
