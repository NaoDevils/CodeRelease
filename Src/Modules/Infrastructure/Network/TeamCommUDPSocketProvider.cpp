#include "TeamCommUDPSocketProvider.h"

#include "Tools/Build.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(TeamCommUDPSocketProvider, cognitionInfrastructure);

TeamCommUDPSocketProvider::TeamCommUDPSocketProvider() {}

void TeamCommUDPSocketProvider::update(TeamCommSocket& teamCommSocket)
{
  bool local = Build::targetSimulator();
  MODIFY("module:TeamCommUDPSocketProvider:local", local);

  if (this->port != theOwnTeamInfo.teamPort || this->local != local)
  {
    if (this->port)
    {
      stop();
      OUTPUT_TEXT("TeamCommUDPSocketProvider: Set port to " << theOwnTeamInfo.teamPort);
    }

    if (local)
      startLocal(theOwnTeamInfo.teamPort);
    else
      start(theOwnTeamInfo.teamPort, "10.0.255.255");
  }

  teamCommSocket.send = [this](const TeamCommData& msg)
  {
    return this->send(msg);
  };

  teamCommSocket.receive = [this]()
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
  this->local = false;

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

  return socket.write(teamCommData.data.data(), static_cast<int>(teamCommData.data.size()));
}

std::vector<TeamCommDataReceived> TeamCommUDPSocketProvider::receive()
{
  std::vector<TeamCommDataReceived> messages;

  if (!port)
    return messages; // not started yet

  messages.reserve(maxNumOfTcMessages);

  int size = 0;
  do
  {
    TeamCommDataReceived& message = messages.emplace_back();

    // Allocate one byte more than necessary
    message.data.resize(TeamCommDataReceived::maximumSize + 1);

    size = local
        ? socket.readLocal(message.data.data(), static_cast<int>(message.data.size()))
        : socket.read(message.data.data(), static_cast<int>(message.data.size()), *reinterpret_cast<unsigned int*>(message.remoteIp.data()));
    if (size <= 0)
    {
      messages.pop_back();
      continue;
    }

    // Check if message is bigger than expected
    if (size > static_cast<int>(TeamCommDataReceived::maximumSize))
    {
      OUTPUT_WARNING("Received packet with invalid size from " << message.remoteIp[3] << "." << message.remoteIp[2] << "." << message.remoteIp[1] << "." << message.remoteIp[0]);
      messages.pop_back();
      continue;
    }

    // Adjust buffer to actual size
    message.data.resize(size);

    // This could be much later than the package was received by the hardware.
    // For our time measurements, this should be enough.
    message.receiveTimestamp = SystemCall::getCurrentSystemTime();

    if (messages.size() >= maxNumOfTcMessages)
    {
      OUTPUT_WARNING("Packet buffer is full, ignoring further ones (last from "
          << message.remoteIp[3] << "." << message.remoteIp[2] << "." << message.remoteIp[1] << "." << message.remoteIp[0] << ")");
      return messages;
    }
  } while (size > 0);

  return messages;
}
