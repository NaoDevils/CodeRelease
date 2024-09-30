#pragma once

#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(RecommendedKick, PositioningAndKickSymbols,
  ,
  (bool)(false) hasRecommendation,
  (float)(-1.f) estimatedKickTime,
  (float)(-1.f) estimatedPoseTime,
  (float)(-1.f) successProbability,
  (float)(-1.f) score,
  (unsigned int)(0) recommendationTimeStamp,
  (unsigned int)(0) sinceRecommendationTime
);
