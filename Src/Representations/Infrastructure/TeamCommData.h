/**
 * @file TeamCommData.h
 * This makes RoboCup::SPLStandardMessage streamable.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Streams/AutoStreamable.h"

class MessageQueue;

struct NaoDevilsHeader
{
  static constexpr std::array<char, 4> desiredTeamName = {'N', 'D', '0', '8'};
  static constexpr int desiredVersion = 2;

  std::array<char, 4> teamName = desiredTeamName;
  unsigned int version = desiredVersion;
  unsigned int sendTimestamp = 0;

  void init();
  bool verify() const;
};

struct TeamCommData : public RoboCup::SPLStandardMessage, public Streamable
{
public:
  static constexpr int headerSize = sizeof(RoboCup::SPLStandardMessage) - SPL_STANDARD_MESSAGE_DATA_SIZE;

  unsigned int receiveTimestamp = 0;
  unsigned int remoteIp = 0;

  TeamCommData();

  NaoDevilsHeader& getNDHeader() { return reinterpret_cast<NaoDevilsHeader&>(*data); }
  const NaoDevilsHeader& getNDHeader() const { return reinterpret_cast<const NaoDevilsHeader&>(*data); }
  uint8_t* getNDData() { return data + sizeof(NaoDevilsHeader); }
  const uint8_t* getNDData() const { return data + sizeof(NaoDevilsHeader); }

  char* getData() { return reinterpret_cast<char*>(&header); }
  const char* getData() const { return reinterpret_cast<const char*>(&header); }
  int getSize() const { return headerSize + numOfDataBytes; }

  bool verify() const;

  void fillMessageQueue(MessageQueue& queue) const;

  virtual Streamable& operator=(const Streamable&) noexcept;
  virtual void serialize(In* in, Out* out);
};

STREAMABLE_WITH_BASE(TeamCommOutput, TeamCommData,
 ,
  (bool)(false) sendThisFrame
);

STREAMABLE(TeamCommInput,,
  (std::vector<TeamCommData>) messages
);
