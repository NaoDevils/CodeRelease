#pragma once

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Functions/ProbabilityFunctions.h"
#include "SelectableDirection.h"
#include "SelectableKick.h"
#include "Tools/Math/Geometry.h"

class SelectableTarget
{

public:
  class DistanceInfo
  {
  public:
    DistanceInfo(const float targetDistance,
        const bool isIntoBlocked,
        const float intoGoalKickOutsideProbability,
        const float intoKickInOutsideProbability,
        const float intoCornerKickOutsideProbability,
        const float intoOpponentsGoalProbability,
        const float intoOwnGoalProbability)
        : targetDistance(targetDistance), isIntoBlocked(isIntoBlocked), intoGoalKickOutsideProbability(intoGoalKickOutsideProbability), intoKickInOutsideProbability(intoKickInOutsideProbability),
          intoCornerKickOutsideProbability(intoCornerKickOutsideProbability), intoOpponentsGoalProbability(intoOpponentsGoalProbability), intoOwnGoalProbability(intoOwnGoalProbability)
    {
      ASSERT(targetDistance >= 0.f);
      ASSERT(0.f <= intoGoalKickOutsideProbability && intoGoalKickOutsideProbability <= 1.f);
      ASSERT(0.f <= intoKickInOutsideProbability && intoKickInOutsideProbability <= 1.f);
      ASSERT(0.f <= intoCornerKickOutsideProbability && intoCornerKickOutsideProbability <= 1.f);
      ASSERT(0.f <= intoOpponentsGoalProbability && intoOpponentsGoalProbability <= 1.f);
      ASSERT(0.f <= intoOwnGoalProbability && intoOwnGoalProbability <= 1.f);
    }
    static DistanceInfo create(const SelectableKick& selectableKick, const SelectableDirection& selectableDirection, const float targetedDistance)
    {
      const bool isIntoBlocked = selectableDirection.isFirstIntoBlocked && targetedDistance > selectableDirection.intoBlockedDistance;

      if (isIntoBlocked)
      {
        return {selectableDirection.intoBlockedDistance, isIntoBlocked, 0.f, 0.f, 0.f, 0.f, 0.f};
      }
      else
      {
        const float targetDistance = targetedDistance;
        const float verticalInaccuracy = selectableKick.kick->verticalInaccuracy;
        return {targetDistance,
            isIntoBlocked,
            ProbabilityFunctions::getOvershootDistanceProbability(selectableDirection.intoGoalKickOutsideDistance, targetDistance, verticalInaccuracy),
            ProbabilityFunctions::getOvershootDistanceProbability(selectableDirection.intoKickInOutsideDistance, targetDistance, verticalInaccuracy),
            ProbabilityFunctions::getOvershootDistanceProbability(selectableDirection.intoCornerKickOutsideDistance, targetDistance, verticalInaccuracy),
            ProbabilityFunctions::getOvershootDistanceProbability(selectableDirection.intoOpponentsGoalDistance, targetDistance, verticalInaccuracy),
            ProbabilityFunctions::getOvershootDistanceProbability(selectableDirection.intoOwnGoalDistance, targetDistance, verticalInaccuracy)};
      }
    }
    float targetDistance;
    bool isIntoBlocked;
    float intoGoalKickOutsideProbability;
    float intoKickInOutsideProbability;
    float intoCornerKickOutsideProbability;
    float intoOpponentsGoalProbability;
    float intoOwnGoalProbability;
  };

  SelectableTarget(const SelectableKick& selectableKick, const SelectableDirection& selectableDirection, const float targetedDistance)
      : selectableKick(selectableKick), selectableDirection(selectableDirection), distanceInfo(DistanceInfo::create(selectableKick, selectableDirection, targetedDistance))
  {
    target = selectableDirection.ballPosition + distanceInfo.targetDistance * selectableDirection.direction;
  }

  [[nodiscard]] bool isIntoOutside() const
  {
    return distanceInfo.intoGoalKickOutsideProbability >= 0.5f || distanceInfo.intoKickInOutsideProbability >= 0.5f
        || distanceInfo.intoCornerKickOutsideProbability >= 0.5f; // equal to target is outside because 50% go further and 50% shorter
  }

  [[nodiscard]] bool isIntoOpponentsGoal() const
  {
    return distanceInfo.intoOpponentsGoalProbability >= 0.5f; // equal to target is in opponents goal because 50% go further and 50% shorter
  }

  [[nodiscard]] bool isIntoOwnGoal() const
  {
    return distanceInfo.intoOwnGoalProbability >= 0.5f; // equal to target is in own goal because 50% go further and 50% shorter
  }

  void setScore(const float newScore)
  {
    score = newScore;
    scoreSet = true;
  }

  [[nodiscard]] float getScore() const
  {
    ASSERT(scoreSet);
    return score;
  }

  void setFiltered(const bool newFiltered) { filtered = newFiltered; }

  [[nodiscard]] bool isFiltered() const { return filtered; }

  SelectableKick selectableKick;
  SelectableDirection selectableDirection;
  Vector2f target;
  DistanceInfo distanceInfo;

  void draw() const { ARROW(DRAW_RECOMMENDED_KICK, selectableDirection.ballPosition.x(), selectableDirection.ballPosition.y(), target.x(), target.y(), 25, 25, ColorRGBA::black); }

private:
  float score;
  bool scoreSet = false;

  bool filtered = false;
};
