/**
 * @file TeamCommLocalSocketProvider.h
 * This file provides a TeamCommSocket without network communication.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/TeamCommSocket.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Platform/SystemCall.h"
#include <array>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <tuple>

MODULE(TeamCommLocalSocketProvider,
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  PROVIDES_WITHOUT_MODIFY(TeamCommSocket)
);

class TeamCommLocalSocketProvider : public TeamCommLocalSocketProviderBase
{
private:
  // This static is intentional allowing communication between different instances of this module.
  // In theory, we have to use a queue here, but since all robots are running synchronously, a single buffer per robot is fine.
  static std::array<std::array<std::optional<std::tuple<unsigned int, TeamCommData>>, MAX_NUM_PLAYERS>, 2> messages;
  static std::array<std::array<std::shared_mutex, MAX_NUM_PLAYERS>, 2> mutex;

  std::array<unsigned int, MAX_NUM_PLAYERS> lastMessage{0};

public:
  ~TeamCommLocalSocketProvider();

  bool send(const TeamCommData& teamCommData);

  std::vector<TeamCommDataReceived> receive();

  void update(TeamCommSocket& teamCommSocket);
};
