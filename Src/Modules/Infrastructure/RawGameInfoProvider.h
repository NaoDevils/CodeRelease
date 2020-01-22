/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file declares a module that provides the game info from the game controller.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#ifdef TARGET_ROBOT
#include "arpa/inet.h"
#include "Tools/Network/UdpComm.h"
#include "Platform/Linux/NaoBody.h"
#include "Platform/Linux/NaoBodyV6.h"
#endif
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/RobotInfo.h"

static const int GAMECONTROLLER_TIMEOUT = 2000; /**< Connected to GameController when packet was received within the last 2000 ms. */

MODULE(RawGameInfoProvider,
{,
  USES(FrameInfo),
  USES(KeyStates),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  LOADS_PARAMETERS(
  {,
    ((RobotInfo) NaoType) naoBodyType, ///< H21, H25 ...
    ((RobotInfo) NaoType) naoHeadType,
  }),
});

/**
* @class RawGameInfoProvider
* A module that provides the game info from the game controller
*/
class RawGameInfoProvider : public RawGameInfoProviderBase
{
private:
#ifdef TARGET_ROBOT
  /** The main function, called every cycle
  * @param rawGameInfo The data struct to be filled
  */
  void update(RawGameInfo& rawGameInfo);
  void update(OpponentTeamInfo& teamInfo);
  void update(OwnTeamInfo& ownTeamInfo);
  void update(RobotInfo& robotInfo);
  void execute();

  bool receive();

  bool send();

  void initialize();

  uint8_t lastGameState = STATE_INITIAL;
  uint8_t lastSecsTillUnpenalized = 0;
  unsigned lastExecutedTimeStamp = 0; // only one execute per frame!
  unsigned lastSentTimeStamp = 0;
#else
  void update(RawGameInfo& rawGameInfo)
  { 
    if(Blackboard::getInstance().exists("GameInfo"))
      (GameInfo&) rawGameInfo = (GameInfo&) Blackboard::getInstance()["GameInfo"]; 
  }
  void update(OpponentTeamInfo& teamInfo) {}
  void update(OwnTeamInfo& ownTeamInfo) {}
  void update(RobotInfo& robotInfo) {}
  bool receive() { return true; }
  bool send() { return true;  }
  void initialize() { memset(&gameCtrlData, 0, sizeof(gameCtrlData)); initialized = true; }
#endif

  void updateGameState();

  bool initialized = false;
  RoboCup::RoboCupGameControlData gameCtrlData; // local copy of all game control data
  uint8_t ownPenalty = PENALTY_NONE;
  unsigned timeStampGameStateChanged = 0;
  unsigned timeStampPenaltyChanged = 0;
  float transitionToBhuman = 0.f;
  unsigned lastReceivedTimeStamp = 0;
#ifdef TARGET_ROBOT
  UdpComm* udp = 0; /**< The socket used to communicate. */
  in_addr gameControllerAddress; /**< The address of the GameController PC. */
  NaoBody naoBody;
  NaoBodyV6 naoBodyV6;
#endif

public:
  /** Constructor. */
  RawGameInfoProvider()
  {    
#ifdef TARGET_ROBOT
    memset(&gameControllerAddress, 0, sizeof(gameControllerAddress));
  	memset(&gameCtrlData, 0, sizeof(gameCtrlData));
#endif
  }
};
