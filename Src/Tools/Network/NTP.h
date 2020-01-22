
#pragma once
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include <limits>
#include <array>

struct NTPData
{
  unsigned sentTimeStamp{ 0 };
  unsigned receivedTimeStamp{ 0 };

  bool valid() const
  {
    return sentTimeStamp > 0 && receivedTimeStamp > 0;
  }
};

class NTP
{
private:
  std::array<NTPData, MAX_NUM_PLAYERS> ntpResponses;
  std::array<int, MAX_NUM_PLAYERS> bestRTT{ 0 };
  std::array<int, MAX_NUM_PLAYERS> bestOffset{ 0 };

public:
  NTP() { bestRTT.fill(std::numeric_limits<int>::max());  }

  void sendResponse(uint8_t playerNum, const NTPData& response);
  void receivedResponse(uint8_t playerNum, const NTPData& request, const NTPData& response);
  void convertRemoteTimeInLocalTime(unsigned& timestamp, int playerNum) const;
  const std::array<NTPData, MAX_NUM_PLAYERS>& getNTPResponses();
  void cleanNTPResponses();
  const std::array<int, MAX_NUM_PLAYERS>& getOffsets();
  const std::array<int, MAX_NUM_PLAYERS>& getRTTs();
};
