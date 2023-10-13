#pragma once

#include "Selectables/KickPlan.h"
#include "../../../../../../Tools/Math/Geometry.h"
#include <functional>
#include <optional>
#include <utility>
#include <vector>

class Factors
{

public:
  Factors(const float widthFactor,
      const float sidesHeatFactor,
      const float goalsHeatFactor,
      const float teammatesKickHeatFactor,
      const float teammatesGoalKickHeatFactor,
      const float opponentsKickHeatFactor,
      const float opponentsGoalKickHeatFactor,
      const float dontRuntIntoFactor,
      const float blockDefensiveCone,
      const float blockOpponentFactor,
      const float timeFactor,
      const float inaccuracyFactor)
      : widthFactor(widthFactor), sidesHeatFactor(sidesHeatFactor), goalsHeatFactor(goalsHeatFactor), teammatesKickHeatFactor(teammatesKickHeatFactor),
        teammatesGoalKickHeatFactor(teammatesGoalKickHeatFactor), opponentsKickHeatFactor(opponentsKickHeatFactor), opponentsGoalKickHeatFactor(opponentsGoalKickHeatFactor),
        dontRuntIntoFactor(dontRuntIntoFactor), blockDefensiveCone(blockDefensiveCone), blockOpponentFactor(blockOpponentFactor), timeFactor(timeFactor), inaccuracyFactor(inaccuracyFactor)
  {
    ASSERT(MathUtils::isEqual(widthFactor, 0.f) || MathUtils::isEqual(sidesHeatFactor, 0.f) || MathUtils::isEqual(goalsHeatFactor, 0.f)
        || MathUtils::isEqual(teammatesKickHeatFactor, 0.f) || MathUtils::isEqual(teammatesGoalKickHeatFactor, 0.f) || MathUtils::isEqual(opponentsKickHeatFactor, 0.f)
        || MathUtils::isEqual(opponentsGoalKickHeatFactor, 0.f) || MathUtils::isEqual(dontRuntIntoFactor, 0.f) || MathUtils::isEqual(blockDefensiveCone, 0.f)
        || MathUtils::isEqual(blockOpponentFactor, 0.f) || MathUtils::isEqual(timeFactor, 0.f) || MathUtils::isEqual(inaccuracyFactor, 0.f));
  }

  Factors& addCustomScoreFunction(std::function<float(KickPlan&)>& customScoreFunction)
  {
    customScoreFunctions.push_back(customScoreFunction);
    return *this;
  }

  const float widthFactor;
  const float sidesHeatFactor;
  const float goalsHeatFactor;
  const float teammatesKickHeatFactor;
  const float teammatesGoalKickHeatFactor;
  const float opponentsKickHeatFactor;
  const float opponentsGoalKickHeatFactor;
  const float dontRuntIntoFactor;
  const float blockDefensiveCone;
  const float blockOpponentFactor;
  const float timeFactor;
  const float inaccuracyFactor;

  std::vector<std::function<float(KickPlan&)>> customScoreFunctions = {};
};
