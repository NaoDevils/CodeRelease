/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file declares a module that provides the game info from the game controller.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include <memory>

#if defined(MACOS) || defined(LINUX)
#include "arpa/inet.h"
#endif

#include "Tools/Module/Module.h"

#include "Tools/Network/UdpComm.h"
#include "Platform/Linux/NaoBodyV6.h"

#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/USBSettings.h"

// debug returns for GC 2022
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(RawGameInfoProvider,
  REQUIRES(FrameInfo),
  USES(BehaviorData),
  USES(FallDownState),
  USES(RobotPose),
  USES(BallModel),
  REQUIRES(KeyStates),
  REQUIRES(USBSettings),
  REQUIRES(RobotInfo),
  HAS_PREEXECUTION,
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  LOADS_PARAMETERS(,
    ((RobotInfo) NaoType) naoBodyType, ///< H21, H25 ...
    ((RobotInfo) NaoType) naoHeadType,
    (std::string)("") robotConfig,
    (unsigned)(TEAM_YELLOW) ownColor,
    (unsigned)(TEAM_BLACK) oppColor,
    (int)(2000) gameControllerTimeout,
    (int)(1000) aliveDelay
  )
);

/**
* @class RawGameInfoProvider
* A module that provides the game info from the game controller
*/
class RawGameInfoProvider : public RawGameInfoProviderBase
{
private:
  /** The main function, called every cycle
  * @param rawGameInfo The data struct to be filled
  */
  void update(RawGameInfo& rawGameInfo);
  void update(OpponentTeamInfo& teamInfo);
  void update(OwnTeamInfo& ownTeamInfo);
  void update(RobotInfo& robotInfo);
  void execute(tf::Subflow&);

  bool receive();

  bool send();

  void initialize();

  uint8_t lastGameState = STATE_INITIAL;
  uint8_t lastSecsTillUnpenalized = 0;
  unsigned lastSentTimeStamp = 0;
  unsigned lastPlayerNumberUpdate = 0;
  unsigned lastTeamNumberUpdate = 0;
  int teamNumber = Global::getSettings().teamNumber;

  void updateGameState();

  bool initialized = false;
  RoboCup::RoboCupGameControlData gameCtrlData; // local copy of all game control data
  uint8_t ownPenalty = PENALTY_NONE;
  unsigned timeStampGameStateChanged = 0;
  unsigned timeStampPenaltyChanged = 0;
  bool chestButtonPressed = false;
  float transitionToFramework = 0.f;
  unsigned lastReceivedTimeStamp = 0;
  std::unique_ptr<UdpComm> udp = nullptr; /**< The socket used to communicate. */
  in_addr gameControllerAddress; /**< The address of the GameController PC. */
#ifdef TARGET_ROBOT
  NaoBodyV6 naoBodyV6;
#endif

public:
  /** Constructor. */
  RawGameInfoProvider()
  {
    memset(&gameControllerAddress, 0, sizeof(gameControllerAddress));
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }
};
