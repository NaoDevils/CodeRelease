#include "TeamCommLocalSocketProvider.h"

#include "Tools/Build.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"


MAKE_MODULE(TeamCommLocalSocketProvider, cognitionInfrastructure);

std::array<std::array<std::optional<std::tuple<unsigned int, TeamCommData>>, MAX_NUM_PLAYERS>, 2> TeamCommLocalSocketProvider::messages;
std::array<std::array<std::shared_mutex, MAX_NUM_PLAYERS>, 2> TeamCommLocalSocketProvider::mutex;

TeamCommLocalSocketProvider::~TeamCommLocalSocketProvider()
{
  ASSERT(theOwnTeamInfo.teamNumber == 1 || theOwnTeamInfo.teamNumber == 2);

  std::unique_lock l(mutex[theOwnTeamInfo.teamNumber - 1][theRobotInfo.number - 1]);
  messages[theOwnTeamInfo.teamNumber - 1][theRobotInfo.number - 1].reset();
}

void TeamCommLocalSocketProvider::update(TeamCommSocket& teamCommSocket)
{
  teamCommSocket.send = [=](const TeamCommData& msg)
  {
    return this->send(msg);
  };

  teamCommSocket.receive = [=]()
  {
    return this->receive();
  };
}

bool TeamCommLocalSocketProvider::send(const TeamCommData& teamCommData)
{
  ASSERT(theOwnTeamInfo.teamNumber == 1 || theOwnTeamInfo.teamNumber == 2);

  std::unique_lock l(mutex[theOwnTeamInfo.teamNumber - 1][theRobotInfo.number - 1]);
  auto& msg = messages[theOwnTeamInfo.teamNumber - 1][theRobotInfo.number - 1];
  if (msg)
  {
    ++std::get<unsigned int>(*msg);
    std::get<TeamCommData>(*msg) = teamCommData;
  }
  else
  {
    msg = std::make_tuple<unsigned int, TeamCommData>(1, TeamCommData(teamCommData));
  }

  return true;
}

std::vector<TeamCommDataReceived> TeamCommLocalSocketProvider::receive()
{
  ASSERT(theOwnTeamInfo.teamNumber == 1 || theOwnTeamInfo.teamNumber == 2);

  std::vector<TeamCommDataReceived> ret;
  auto m = mutex[theOwnTeamInfo.teamNumber - 1].begin();

  const auto& teamMessages = messages[theOwnTeamInfo.teamNumber - 1];
  for (size_t num = 0; num < teamMessages.size(); ++num)
  {
    const auto& robotMessage = teamMessages[num];

    std::shared_lock l(*m++);
    if (robotMessage && std::get<unsigned int>(*robotMessage) != lastMessage[num])
    {
      lastMessage[num] = std::get<unsigned int>(*robotMessage);

      TeamCommDataReceived& received = ret.emplace_back();
      received.data = std::get<TeamCommData>(*robotMessage).data;
      received.receiveTimestamp = SystemCall::getCurrentSystemTime();
    }
  }

  return ret;
}
