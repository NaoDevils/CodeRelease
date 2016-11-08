/**
 * @file Tools/ProcessFramework/TeamHandler.cpp
 * The file implements a class for team communication between robots.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "TeamHandler.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

#include "Representations/Infrastructure/SPLStandardMessage.h"

TeamHandler::TeamHandler(TeamDataIn& in, TeamDataOut& out) :
  in(in), out(out)
{}

void TeamHandler::startLocal(int port, unsigned localId)
{
  ASSERT(!this->port);
  this->port = port;
  this->localId = localId;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); //keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
  socket.setLoopback(true);
}

void TeamHandler::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setTarget(subnet, port);
  socket.setLoopback(false);
}

void TeamHandler::send()
{
  if(!port || out.isEmpty())
    return;

  if(socket.write((char*)&out.message, teamCommHeaderSize + ndevilsHeaderSize + out.message.numOfDataBytes))
    out.clear();

  // Plot usage of data buffer in percent:
  DECLARE_PLOT("module:TeamHandler:standardMessageDataBufferUsageInPercent");
  const float usageInPercent = out.message.numOfDataBytes * 100.f / static_cast<float>(SPL_STANDARD_MESSAGE_DATA_SIZE);
  PLOT("module:TeamHandler:standardMessageDataBufferUsageInPercent", usageInPercent);
}

unsigned TeamHandler::receive()
{
  if(!port)
    return 0; // not started yet

  RoboCup::SPLStandardMessage inMsg;
  int size;
  unsigned remoteIp = 0;
  unsigned receivedSize = 0;

  do
  {
    size = localId ? socket.readLocal((char*)&inMsg, sizeof(RoboCup::SPLStandardMessage))
                   : socket.read((char*)&inMsg, sizeof(RoboCup::SPLStandardMessage), remoteIp);
    if(size >= teamCommHeaderSize && size <= static_cast<int>(sizeof(RoboCup::SPLStandardMessage)))
    {
      receivedSize = static_cast<unsigned>(size);
      if (checkMessage(inMsg, remoteIp, receivedSize - teamCommHeaderSize - ndevilsHeaderSize))
      {
        in.messages.push_back(inMsg);
        if (Global::getSettings().gameMode != Settings::dropIn)
          addDataToQueue(inMsg, remoteIp);
      }
    }
  }
  while(size > 0);

  return receivedSize;
}

bool TeamHandler::checkMessage(RoboCup::SPLStandardMessage& msg, const unsigned remoteIp, const unsigned realNumOfDataBytes)
{
  if (msg.header[0] != 'S' || msg.header[1] != 'P' || msg.header[2] != 'L' || msg.header[3] != ' ')
  {
    OUTPUT_WARNING("Received package from ip " << remoteIp << " with Header '" << msg.header[0] << msg.header[1] << msg.header[2] << msg.header[3] << "' but should be 'SPL '. Ignoring package...");
    return false;
  }

  if (msg.version != SPL_STANDARD_MESSAGE_STRUCT_VERSION)
  {
    OUTPUT_WARNING("Received package from ip " << remoteIp << " with SPL_STANDARD_MESSAGE_STRUCT_VERSION '" << msg.version << "' but should be '" << SPL_STANDARD_MESSAGE_STRUCT_VERSION << "'.Ignoring package...");
    return false;
  }

  if (msg.numOfDataBytes != static_cast<uint16_t>(realNumOfDataBytes))
  {
    OUTPUT_WARNING("SPL Message: numOfDataBytes is '" << msg.numOfDataBytes << "' but realNumOfDataBytes is '" << realNumOfDataBytes << "'.");
    msg.numOfDataBytes = std::min(msg.numOfDataBytes, static_cast<uint16_t>(realNumOfDataBytes));

    if (Global::getSettings().gameMode != Settings::dropIn)
    {
      OUTPUT_WARNING("... ignoring package!");
      return false;
    }
  }

  if (Global::getSettings().gameMode != Settings::dropIn && msg.numOfDataBytes < ndevilsHeaderSize)
  {
    OUTPUT_WARNING("Ignoring SPL Message because: 'numOfDataBytes < ndevilsHeaderSize'.");
    return false;
  }
  return true;
}

void TeamHandler::addDataToQueue(const RoboCup::SPLStandardMessage& msg, const unsigned remoteIp)
{
  const NDevilsHeader& header = (const NDevilsHeader&)*msg.data;

  in.queue.out.bin << (remoteIp ? remoteIp : static_cast<int>(msg.playerNum));
  in.queue.out.bin << header.timeStampSent;
  in.queue.out.bin << SystemCall::getCurrentSystemTime();
  in.queue.out.finishMessage(idNTPHeader);

  in.queue.out.bin << static_cast<int>(msg.playerNum);
  in.queue.out.finishMessage(idRobot);

  InBinaryMemory memory(msg.data + ndevilsHeaderSize, msg.numOfDataBytes - ndevilsHeaderSize);
  memory >> in.queue;
}
