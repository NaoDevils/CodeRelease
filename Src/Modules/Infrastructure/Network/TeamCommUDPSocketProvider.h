/**
 * @file TeamCommUDPSocketProvider.h
 * This file provides a TeamCommSocket.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/TeamCommSocket.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Network/UdpComm.h"

MODULE(TeamCommUDPSocketProvider,
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),
  PROVIDES_WITHOUT_MODIFY(TeamCommSocket)
);

class TeamCommUDPSocketProvider : public TeamCommUDPSocketProviderBase
{
private:
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  bool local = false; /**< The id of a local team communication participant or 0 for normal udp communication. */

  static constexpr int maxNumOfTcMessages = MAX_NUM_PLAYERS * 2; // x broadcasting players and substitute + space for delays

public:
  TeamCommUDPSocketProvider();

  /**
   * The method starts the actual communication for local communication.
   * @param port The UDP port this handler is listening to.
   */
  void startLocal(int port);

  /**
   * The method starts the actual communication on the given port.
   * @param port The UDP port this handler is listening to.
   * @param subnet The subnet the handler is broadcasting to.
   */
  void start(int port, const char* subnet);

  void stop();

  /**
   * The method sends the outgoing message queue if possible.
   */
  bool send(const TeamCommData& teamCommData);

  /**
   * The method receives packages if available.
   * @return The number of bytes received.
   */
  std::vector<TeamCommDataReceived> receive();

  void update(TeamCommSocket& teamCommSocket);
};
