#include "TeamCommData.h"
#include "Tools/MessageQueue/MessageQueue.h"

bool NaoDevilsHeader::verify() const
{
  return teamName == desiredTeamName && version == desiredVersion;
}

TeamCommData::TeamCommData()
{
  numOfDataBytes = sizeof(NaoDevilsHeader);
  new (&getNDHeader()) NaoDevilsHeader();
}

bool TeamCommData::verify() const
{
  return strncmp(header, SPL_STANDARD_MESSAGE_STRUCT_HEADER, sizeof(SPL_STANDARD_MESSAGE_STRUCT_HEADER)) && version == SPL_STANDARD_MESSAGE_STRUCT_VERSION
      && numOfDataBytes >= sizeof(NaoDevilsHeader) && getNDHeader().verify();
}

void TeamCommData::fillMessageQueue(MessageQueue& queue) const
{
  queue.setSize(sizeof(RoboCup::SPLStandardMessage)); // isn't this too big?

  InBinaryMemory memory(data + sizeof(NaoDevilsHeader), static_cast<size_t>(numOfDataBytes) - sizeof(NaoDevilsHeader));
  memory >> queue;
}

Streamable& TeamCommData::operator=(const Streamable& other) noexcept
{
  return *this = dynamic_cast<const TeamCommData&>(other);
}

void TeamCommData::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(header);
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(numOfDataBytes);
  STREAM(data);
  STREAM(receiveTimestamp);
  STREAM_REGISTER_FINISH;
}
