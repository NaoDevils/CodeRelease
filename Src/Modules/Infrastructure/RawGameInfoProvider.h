/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file declares a module that provides the game info from the game controller.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include <memory>
#include <array>

#include "Tools/Module/Module.h"
#include "Tools/Network/UdpComm.h"

#ifdef TARGET_ROBOT
#include "Platform/Linux/NaoBodyV6.h"
#endif

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
  HAS_PREEXECUTION,
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  LOADS_PARAMETERS(,
    (unsigned char)(TEAM_YELLOW) ownColor,
    (unsigned char)(TEAM_RED) ownKeeperColor,
    (unsigned char)(TEAM_BLACK) oppColor,
    (unsigned char)(TEAM_BLUE) oppKeeperColor,
    (unsigned char)(1) keeper,
    (int)(5000) gameControllerTimeout,
    (int)(1000) aliveDelay,
    (uint8_t)(0) teamNumber,
    (uint16_t)(0) teamPort,
    (uint8_t)(0) playerNumber
  )
);

/**
* @class RawGameInfoProvider
* A module that provides the game info from the game controller
*/
class RawGameInfoProvider : public RawGameInfoProviderBase
{
  void initialize();
  void setDefaults();

  void update(RawGameInfo& rawGameInfo);
  void update(OpponentTeamInfo& teamInfo);
  void update(OwnTeamInfo& ownTeamInfo);
  void update(RobotInfo& robotInfo);
  void execute(tf::Subflow&);

  bool receive(RoboCup::RoboCupGameControlData& package, std::array<uint8_t, 4>& ip) const;
  bool send();
  bool sendVRC();

  void updateGameState();

  // init local copy with current blackboard version
  // allows to read player/team number from simulator after module request
  RawGameInfo rawGameInfo = Blackboard::get<RawGameInfo>();
  OpponentTeamInfo opponentTeamInfo = Blackboard::get<OpponentTeamInfo>();
  OwnTeamInfo ownTeamInfo = Blackboard::get<OwnTeamInfo>();
  RobotInfo robotInfo = Blackboard::get<RobotInfo>();

  uint8_t lastGameState = STATE_INITIAL;
  unsigned lastSentTimeStamp = 0;
  unsigned lastUSBSettingsUpdate = 0;

  bool initialized = false;
  std::unique_ptr<UdpComm> udp = nullptr; /**< The socket used to communicate. */

  bool lastChestButtonPressed = false;
  bool firstChestButtonPressed = false;

#ifdef TARGET_ROBOT
  NaoBodyV6 naoBodyV6;
#endif
};
