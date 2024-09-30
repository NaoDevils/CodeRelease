#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysteresisUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/PathUtils.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RecommendedKick/DirectionInfo.h"
#include "Representations/Modeling/RecommendedKick/PositionInfo/PositionInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

class ProbabilityFunctions
{

public:
  /**
   * @param checkDistance pass -1.f as no valid distance for a probability of 0.f
   */
  static float getOvershootDistanceProbability(const float checkDistance, const float optimalDistance, const float inaccuracy)
  {
    ASSERT(optimalDistance >= 0.f);
    ASSERT(inaccuracy >= 0.f);

    if (checkDistance == -1.f)
    {
      return 0.f;
    }
    ASSERT(checkDistance >= 0.f);

    if (checkDistance > optimalDistance)
    {
      const float checkDivergence = checkDistance - optimalDistance;
      ASSERT(checkDivergence >= 0.f);
      if (checkDivergence > inaccuracy)
      {
        return 0.f;
      }
      const float probability = (1.f - checkDivergence / inaccuracy) / 2.f; // divide by 2 because there is a 50% chance it is shorter than optimal
      ASSERT(probability >= 0.f);
      ASSERT(probability <= 1.f);
      return probability;
    }
    else
    {
      const float checkDivergence = optimalDistance - checkDistance;
      ASSERT(checkDivergence >= 0.f);
      if (checkDivergence > inaccuracy)
      {
        return 1.f;
      }
      const float probability = 0.5f + (checkDivergence / inaccuracy) / 2.f; // add 50% because it has a 50% chance to go further and thereby outside
      ASSERT(probability > 0.f);
      ASSERT(probability <= 1.f);
      return probability;
    }
  }

  static float getTimeProbability(const float robotTimeToKick, const TacticSymbols& theTacticSymbols)
  {
    const float MINIMAL_PROBABILITY = 0.2f; // Because there can always be something wrong with the opponent

    if (theTacticSymbols.ballInDanger == TacticSymbols::Danger::NONE || theTacticSymbols.ballInDanger == TacticSymbols::Danger::IMPOSSIBLE)
    {
      return 1.f;
    }
    if (!theTacticSymbols.hasClosestOpponentRobot())
    {
      return 1.f;
    }

    const std::function<float(float)> f = [](const float x)
    {
      ASSERT(x >= 0.f);
      return 1.f - 1.f / std::pow(2.f, (2.f / 3.f) * x);
    };

    const float opponentTimeToKick = theTacticSymbols.untilOpponentStealsBallTime;

    float probability;
    if (robotTimeToKick > opponentTimeToKick)
    {
      const float x = std::abs(robotTimeToKick - opponentTimeToKick);
      probability = 0.5f - 0.5f * f(x);
    }
    else
    {
      const float x = std::abs(opponentTimeToKick - robotTimeToKick);
      probability = 0.5f + 0.5f * f(x);
    }
    probability = MINIMAL_PROBABILITY + (1.f - MINIMAL_PROBABILITY) * probability;
    ASSERT(probability >= 0.f && probability <= 1.f);
    return probability;
  }
};