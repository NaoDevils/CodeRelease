#include "Time.h"
#include "Platform/BHAssert.h"

void TimeOffsets::convertRemoteTimeInLocalTime(unsigned& timestamp, int playerNum) const
{
  ASSERT(0 < playerNum && playerNum <= MAX_NUM_PLAYERS);
  --playerNum;

  if (timestamp == 0)
    return;

  // Force online sync if time difference is > 60s.
  // Do online sync if time difference error (=bestRTT) is smaller than time difference.
  // This prefers offline sync, if the error is larger than the difference.
  const int timeDiff = std::abs(bestOffset[playerNum]);
  if (timeDiff < 60000 && timeDiff <= bestRTT[playerNum])
    return;

  int remoteTimestamp = static_cast<int>(timestamp) - bestOffset[playerNum];
  if (remoteTimestamp < 0)
    timestamp = 0;
  else
    timestamp = static_cast<unsigned>(remoteTimestamp);
}
