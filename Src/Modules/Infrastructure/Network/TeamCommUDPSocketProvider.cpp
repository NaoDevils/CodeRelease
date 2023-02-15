#include "TeamCommUDPSocketProvider.h"

#include "Tools/Build.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(TeamCommUDPSocketProvider, cognitionInfrastructure);

TeamCommUDPSocketProvider::TeamCommUDPSocketProvider() {}

void TeamCommUDPSocketProvider::update(TeamCommSocket& teamCommSocket)
{
  int teamPort = 0;
  if constexpr (Build::target == Build::Target::Simulator)
    teamPort = Global::getSettings().teamPort;
  else
    teamPort = theOwnTeamInfo.teamPort;

  if (this->port != teamPort)
  {
    if (this->port)
    {
      stop();
      OUTPUT_TEXT("TeamCommUDPSocketProvider: Set port to " << teamPort);
    }

    if constexpr (Build::target == Build::Target::Simulator)
      startLocal(teamPort);
    else
      start(teamPort, "10.0.255.255");
  }

  teamCommSocket.send = [=](const TeamCommData& msg)
  {
    return this->send(msg);
  };

  teamCommSocket.receive = [=]()
  {
    return this->receive();
  };
}

void TeamCommUDPSocketProvider::startLocal(int port)
{
  ASSERT(!this->port);
  this->port = port;
  this->local = true;

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

void TeamCommUDPSocketProvider::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setTarget(subnet, port);
  socket.setLoopback(false);
}

void TeamCommUDPSocketProvider::stop()
{
  socket = UdpComm();
  this->port = 0;
}

bool TeamCommUDPSocketProvider::send(const TeamCommData& teamCommData)
{
  if (!port)
    return false;

  // Plot usage of data buffer in percent:
  DECLARE_PLOT("module:TeamCommUDPSocketProvider:standardMessageDataBufferUsageInPercent");
  const float usageInPercent = teamCommData.numOfDataBytes * 100.f / static_cast<float>(SPL_STANDARD_MESSAGE_DATA_SIZE);
  PLOT("module:TeamCommUDPSocketProvider:standardMessageDataBufferUsageInPercent", usageInPercent);

  TeamCommData sendTeamCommData = teamCommData;
  sendTeamCommData.getNDHeader().sendTimestamp = SystemCall::getCurrentSystemTime();

  return socket.write(sendTeamCommData.getData(), sendTeamCommData.getSize());
}

std::vector<TeamCommData> TeamCommUDPSocketProvider::receive()
{
  std::vector<TeamCommData> messages;

  if (!port)
    return messages; // not started yet

  messages.reserve(maxNumOfTcMessages);

  int size;
  do
  {
    TeamCommData message;

    size = local ? socket.readLocal(message.getData(), sizeof(RoboCup::SPLStandardMessage)) : socket.read(message.getData(), sizeof(RoboCup::SPLStandardMessage), message.remoteIp);
    if (size >= TeamCommData::headerSize && size <= static_cast<int>(sizeof(RoboCup::SPLStandardMessage)))
    {
      // This could be much later than the package was received by the hardware.
      // For our time measurements, this should be enough.
      message.receiveTimestamp = SystemCall::getCurrentSystemTime();

      if (messages.size() >= maxNumOfTcMessages)
      {
        OUTPUT_WARNING("Received too many packages, ignoring package from " << message.remoteIp);
        return messages;
      }

      if (checkMessage(message, size - TeamCommData::headerSize))
        messages.push_back(std::move(message));
    }
  } while (size > 0);

  return messages;
}

bool TeamCommUDPSocketProvider::checkMessage(const TeamCommData& msg, int realNumOfDataBytes)
{
  if (msg.playerNum < Global::getSettings().lowestValidPlayerNumber || msg.playerNum > Global::getSettings().highestValidPlayerNumber || msg.teamNum != theOwnTeamInfo.teamNumber)
    return false;

  if (msg.numOfDataBytes != static_cast<uint16_t>(realNumOfDataBytes))
  {
    OUTPUT_WARNING("SPL Message: numOfDataBytes is '" << msg.numOfDataBytes << "' but realNumOfDataBytes is '" << realNumOfDataBytes << "'.  Ignoring package...");
    return false;
  }

  if (!msg.verify())
  {
    OUTPUT_WARNING("Invalid package from ip " << msg.remoteIp << ". Ignoring package...");
    return false;
  }

  if constexpr (Build::target == Build::Target::Robot)
  {
    if (msg.teamNum != theOwnTeamInfo.teamNumber)
    {
      OUTPUT_WARNING("Received package from ip " << msg.remoteIp << " with team number '" << msg.teamNum << "'. Ignoring package...");
      return false;
    }
  }

  return true;
}
