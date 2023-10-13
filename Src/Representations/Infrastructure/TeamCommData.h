/**
 * @file TeamCommData.h
 * These classes encapsulate binary team communication data.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include <vector>
#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(TeamCommData,
  static constexpr size_t maximumSize = 128; // SPL rule

  TeamCommData()
  { data.reserve(maximumSize);
  }
  ,
  (std::vector<char>) data
);

STREAMABLE_WITH_BASE(TeamCommDataReceived, TeamCommData,
 ,
  (unsigned int)(0) receiveTimestamp,
  (std::array<uint8_t,4>)({0}) remoteIp
);

STREAMABLE_WITH_BASE(TeamCommOutput, TeamCommData,
 ,
  (bool)(false) sendThisFrame
);

STREAMABLE(TeamCommInput,,
  (std::vector<TeamCommDataReceived>) messages
);
