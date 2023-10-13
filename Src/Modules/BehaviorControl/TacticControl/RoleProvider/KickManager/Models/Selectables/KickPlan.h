#pragma once

#include <utility>
#include "Tools/Math/Eigen.h"
#include "SelectablePose.h"

class KickPlan
{

public:
  KickPlan(const SelectablePose& selectablePose, const bool hysteresis) : selectablePose(std::move(selectablePose)), hysteresis(hysteresis) {}

  void calculateScore() { score = selectablePose.score + selectablePose.selectableTarget.score + selectablePose.selectableTarget.selectableKick.score; }

  SelectablePose selectablePose;
  float score = 0.f;
  bool hysteresis;
};
