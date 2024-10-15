/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file implements a module that provides information game info.
*/
#include "RawGameInfoProvider.h"
#include "Tools/Build.h"

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

MAKE_MODULE(RawGameInfoProvider, cognitionInfrastructure)

void RawGameInfoProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  opponentTeamInfo = this->opponentTeamInfo;
}

void RawGameInfoProvider::update(OwnTeamInfo& ownTeamInfo)
{
  ownTeamInfo = this->ownTeamInfo;

  // if (ownTeamInfo.goalkeeper > 0 && ownTeamInfo.goalkeeper != 1)
  //   OUTPUT_WARNING("RawGameInfoProvider: Player number " << ownTeamInfo.goalkeeper << " is supposed to be the goalkeeper, which is not supported!");
}

void RawGameInfoProvider::update(RawGameInfo& rawGameInfo)
{
  rawGameInfo = this->rawGameInfo;
}

void RawGameInfoProvider::update(RobotInfo& robotInfo)
{
  robotInfo = this->robotInfo;
}

void RawGameInfoProvider::setDefaults()
{
  if (ownTeamInfo.teamNumber == 0)
  {
    ownTeamInfo.fieldPlayerColour = ownColor;
    ownTeamInfo.goalkeeperColour = ownKeeperColor;
    ownTeamInfo.goalkeeper = keeper;
  }

  if (opponentTeamInfo.teamNumber == 0)
  {
    opponentTeamInfo.fieldPlayerColour = oppColor;
    opponentTeamInfo.goalkeeperColour = oppKeeperColor;
  }

  const auto apply = [](auto usbVal, auto defaultVal, auto& currentVal)
  {
    if (usbVal > 0)
      currentVal = usbVal;
    else if (currentVal == 0 || Build::targetRobot())
      currentVal = defaultVal;
  };
  apply(theUSBSettings.robotNumber, playerNumber, robotInfo.number);
  apply(theUSBSettings.teamNumber, teamNumber, ownTeamInfo.teamNumber);
  apply(theUSBSettings.teamPort, teamPort, ownTeamInfo.teamPort);

#ifdef TARGET_ROBOT
  ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
  robotInfo.transitionToFramework = naoBodyV6.getTransitionToFramework();
#else
  robotInfo.transitionToFramework = 1.f;
#endif
}

void RawGameInfoProvider::execute(tf::Subflow&)
{
  // Avoid firewall window on Windows when module is loaded during initialization
  bool enable = Build::targetRobot();
  MODIFY("module:RawGameInfoProvider:enable", enable);

  if (enable && !initialized)
    initialize();

  setDefaults();

  if (lastUSBSettingsUpdate != theUSBSettings.updateTimestamp)
  {
    lastUSBSettingsUpdate = theUSBSettings.updateTimestamp;
    OUTPUT_TEXT("RawGameInfoProvider: Set player number " << robotInfo.number << ", team number " << ownTeamInfo.teamNumber << " and team port " << ownTeamInfo.teamPort << "!");
  }

  RoboCup::RoboCupGameControlData gameCtrlData;
  if (receive(gameCtrlData, rawGameInfo.remoteIp))
  {
    memcpy(&static_cast<RoboCup::RoboCupGameControlData&>(rawGameInfo), &gameCtrlData, (char*)gameCtrlData.teams - (char*)&gameCtrlData);
    static_cast<RoboCup::TeamInfo&>(opponentTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber != ownTeamInfo.teamNumber ? 0 : 1];
    static_cast<RoboCup::TeamInfo&>(ownTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == ownTeamInfo.teamNumber ? 0 : 1];
    static_cast<RoboCup::RobotInfo&>(robotInfo) = ownTeamInfo.players[robotInfo.number - 1];

    opponentTeamInfo.onLeftSide = gameCtrlData.teams[0].teamNumber != ownTeamInfo.teamNumber;
    ownTeamInfo.onLeftSide = gameCtrlData.teams[0].teamNumber == ownTeamInfo.teamNumber;
    rawGameInfo.timeLastPackageReceived = theFrameInfo.time;
  }
  rawGameInfo.controllerConnected = theFrameInfo.getTimeSince(rawGameInfo.timeLastPackageReceived) < gameControllerTimeout;
  rawGameInfo.oppTeamNumber = opponentTeamInfo.teamNumber;


  if (lastGameState != rawGameInfo.state)
  {
    if ((lastGameState == STATE_INITIAL || lastGameState == STATE_STANDBY) && rawGameInfo.state == STATE_READY)
      rawGameInfo.timeFirstReadyState = rawGameInfo.timeLastPackageReceived;
    lastGameState = rawGameInfo.state;
  }

  updateGameState();

  if (theFrameInfo.getTimeSince(lastSentTimeStamp) > aliveDelay && robotInfo.transitionToFramework >= 1.f)
    if (send())
      lastSentTimeStamp = theFrameInfo.time;
}

bool RawGameInfoProvider::receive(RoboCup::RoboCupGameControlData& package, std::array<uint8_t, 4>& ip) const
{
  bool received = false;
  int size;
  RoboCup::RoboCupGameControlData buffer;
  sockaddr_in addr;
  while (udp && (size = udp->read(reinterpret_cast<char*>(&buffer), sizeof(buffer), addr)) > 0)
  {
    if (size == sizeof(buffer) && !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) && buffer.version == GAMECONTROLLER_STRUCT_VERSION
        && (buffer.teams[0].teamNumber == ownTeamInfo.teamNumber || buffer.teams[1].teamNumber == ownTeamInfo.teamNumber))
    {
      addr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);
      udp->setTarget(reinterpret_cast<sockaddr&>(addr));

      *reinterpret_cast<unsigned int*>(ip.data()) = ntohl(addr.sin_addr.s_addr);
      package = buffer;
      received = true;
    }
  }
  return received;
}

bool RawGameInfoProvider::send()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.playerNum = static_cast<uint8_t>(robotInfo.number);
  returnPacket.teamNum = static_cast<uint8_t>(ownTeamInfo.teamNumber);
  returnPacket.fallen = theFallDownState.state != FallDownState::State::upright;

  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;

  returnPacket.ballAge = theBallModel.timeWhenLastSeen == 0 ? -1.f : theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;
  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();

  return udp && udp->write(reinterpret_cast<const char*>(&returnPacket), sizeof(returnPacket));
}

void RawGameInfoProvider::initialize()
{
  udp = std::make_unique<UdpComm>();
  if (!udp->setBlocking(false) || !udp->setBroadcast(true) || !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) || !udp->setLoopback(false))
  {
    OUTPUT_ERROR("RawGameInfoProvider: Could not open UDP port\n");
    udp.reset();
    // continue, because button interface will still work
    initialized = false;
  }
  else
  {
    initialized = true;
  }
}

void RawGameInfoProvider::updateGameState()
{
  uint8_t& state = rawGameInfo.state;
  uint8_t& penalty = robotInfo.penalty;

  if (robotInfo.transitionToFramework == 0.f)
  {
    robotInfo = RobotInfo();
    ownTeamInfo = OwnTeamInfo();
    opponentTeamInfo = OpponentTeamInfo();
    rawGameInfo = RawGameInfo();
    setDefaults();

    if (theKeyStates.pressed[KeyStates::chest])
      firstChestButtonPressed = true;
  }
  else if (theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
  {
    state = STATE_INITIAL;
    penalty = PENALTY_NONE;
  }
  else if (!rawGameInfo.controllerConnected && !firstChestButtonPressed && !lastChestButtonPressed && theKeyStates.pressed[KeyStates::chest]
      && !theKeyStates.pressed[KeyStates::headFront] && !theKeyStates.pressed[KeyStates::headMiddle] && !theKeyStates.pressed[KeyStates::headRear])
  {
    switch (state)
    {
    case STATE_INITIAL:
    case STATE_STANDBY:
      state = STATE_PLAYING;
      penalty = PENALTY_MANUAL;
      break;
    case STATE_READY:
    case STATE_SET:
      state = STATE_PLAYING;
      penalty = PENALTY_NONE;
      break;
    case STATE_PLAYING:
      penalty = (penalty != PENALTY_NONE) ? PENALTY_NONE : PENALTY_MANUAL;
      break;
    case STATE_FINISHED:
      state = STATE_INITIAL;
      penalty = PENALTY_NONE;
    }
  }
  if (!theKeyStates.pressed[KeyStates::chest])
    firstChestButtonPressed = false;

  lastChestButtonPressed = theKeyStates.pressed[KeyStates::chest];
}
