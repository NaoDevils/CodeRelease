/**
 * @file Time.h
 * This contains TimeOffsets and TimeResponses representations.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include <limits>

STREAMABLE(TimeOffsets,
  TimeOffsets()
  { bestRTT.fill(std::numeric_limits<int>::max());
  }
  void convertRemoteTimeInLocalTime(unsigned& timestamp, int playerNum) const
  ,
  (std::array<int, MAX_NUM_PLAYERS>)({0}) bestRTT,
  (std::array<int, MAX_NUM_PLAYERS>)({0}) bestOffset
);

STREAMABLE(TimeSynchronization,
  STREAMABLE(Measurement,,
    (unsigned)(0) sent,
    (unsigned)(0) received
  );
  STREAMABLE_WITH_BASE(MeasurementFromPlayer, Measurement,,
    (unsigned char)(0) player
  );
  ,
  (std::vector<MeasurementFromPlayer>)({}) receivedRequests,
  (unsigned char)(0) requestFrom /**< Bitmask indicating robot numbers. */
);

static_assert(MAX_NUM_PLAYERS <= sizeof(TimeSynchronization::requestFrom) * 8);
