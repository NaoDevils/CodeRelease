
#include "NTP.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/BHAssert.h"

void NTP::sendResponse(uint8_t playerNum, const NTPData& request)
{
  ASSERT(0 < playerNum && playerNum <= MAX_NUM_PLAYERS);
  ASSERT(playerNum != Global::getSettings().playerNumber);
  --playerNum;

  this->ntpResponses[playerNum] = request;
}

void NTP::receivedResponse(uint8_t playerNum, const NTPData& request, const NTPData& response)
{
  ASSERT(0 < playerNum && playerNum <= MAX_NUM_PLAYERS);
  ASSERT(playerNum != Global::getSettings().playerNumber);
  --playerNum;

  if (!request.valid() || !response.valid()) return;

  int reqSent = static_cast<int>(request.sentTimeStamp);
  int reqRcvd = static_cast<int>(request.receivedTimeStamp);
  int resSent = static_cast<int>(response.sentTimeStamp);
  int resRcvd = static_cast<int>(response.receivedTimeStamp);

  int rtt = (resRcvd - reqSent) - (resSent - reqRcvd);
  int offset = ((reqRcvd - reqSent) + (resSent - resRcvd)) / 2;

  if (rtt < bestRTT[playerNum] || std::abs(bestOffset[playerNum] - offset) > 60000)
  {
    bestRTT[playerNum] = rtt;
    bestOffset[playerNum] = offset;
  }
}

void NTP::convertRemoteTimeInLocalTime(unsigned& timestamp, int playerNum) const
{
  ASSERT(0 < playerNum && playerNum <= MAX_NUM_PLAYERS);
  ASSERT(playerNum != Global::getSettings().playerNumber);
  --playerNum;

  if (timestamp == 0) return;

  // Force online sync if time difference is > 60s.
  // Do online sync if time difference error (=bestRTT) is smaller than time difference.
  // This prefers offline sync, if the error is larger than the difference.
  const int timeDiff = std::abs(bestOffset[playerNum]);
  if (timeDiff < 60000 && timeDiff <= bestRTT[playerNum]) return;

  int remoteTimestamp = static_cast<int>(timestamp) - bestOffset[playerNum];
  if (remoteTimestamp < 0)
    timestamp = 0;
  else
    timestamp = static_cast<unsigned>(remoteTimestamp);
}

const std::array<NTPData, MAX_NUM_PLAYERS>& NTP::getNTPResponses()
{
  return this->ntpResponses;
}

void NTP::cleanNTPResponses()
{
  this->ntpResponses.fill({ 0,0 });
}

const std::array<int, MAX_NUM_PLAYERS>& NTP::getOffsets()
{
  return bestOffset;
}
const std::array<int, MAX_NUM_PLAYERS>& NTP::getRTTs()
{
  return bestRTT;
}
