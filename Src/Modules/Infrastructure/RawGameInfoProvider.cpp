/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file implements a module that provides information game info.
*/

#include "RawGameInfoProvider.h"
#include "Tools/Settings.h"
#include "Tools/Build.h"

MAKE_MODULE(RawGameInfoProvider, cognitionInfrastructure)

void RawGameInfoProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(opponentTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber != this->teamNumber ? 0 : 1];
}

void RawGameInfoProvider::update(OwnTeamInfo& ownTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(ownTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1];

  if (theUSBSettings.teamNumber > 0)
    ownTeamInfo.teamNumber = static_cast<uint8_t>(theUSBSettings.teamNumber);
  else
    ownTeamInfo.teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  this->teamNumber = ownTeamInfo.teamNumber;

  if (theUSBSettings.teamPort > 0)
    ownTeamInfo.teamPort = theUSBSettings.teamPort;
  else
    ownTeamInfo.teamPort = Global::getSettings().teamPort;

  if (lastTeamNumberUpdate != theUSBSettings.updateTimestamp)
  {
    lastTeamNumberUpdate = theUSBSettings.updateTimestamp;
    OUTPUT_TEXT("RawGameInfoProvider: Set team number " << ownTeamInfo.teamNumber << " and team port " << ownTeamInfo.teamPort << "!");
  }

  if (ownTeamInfo.goalkeeper > 0 && ownTeamInfo.goalkeeper != 1)
    OUTPUT_WARNING("RawGameInfoProvider: Player number " << ownTeamInfo.goalkeeper << " is supposed to be the goalkeeper, which is not supported!");
}

void RawGameInfoProvider::update(RawGameInfo& rawGameInfo)
{
  memcpy(&static_cast<RoboCup::RoboCupGameControlData&>(rawGameInfo), &gameCtrlData, (char*)gameCtrlData.teams - (char*)&gameCtrlData);

  // set team number for Logger
  rawGameInfo.oppTeamNumber = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 1 : 0].teamNumber;

  rawGameInfo.timeLastPackageReceived = lastReceivedTimeStamp;
  rawGameInfo.controllerConnected = theFrameInfo.getTimeSince(rawGameInfo.timeLastPackageReceived) < gameControllerTimeout;
  rawGameInfo.remoteIp = gameCtrlDataSender;
  if (lastGameState != rawGameInfo.state)
  {
    if (lastGameState == STATE_INITIAL && rawGameInfo.state == STATE_READY)
      rawGameInfo.timeFirstReadyState = lastReceivedTimeStamp;
    lastGameState = rawGameInfo.state;
  }
}

void RawGameInfoProvider::update(RobotInfo& robotInfo)
{
  if (theUSBSettings.robotNumber > 0)
    robotInfo.number = theUSBSettings.robotNumber;
  else
    robotInfo.number = Global::getSettings().playerNumber;

  robotInfo.transitionToFramework = transitionToFramework;

  RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1];
  static_cast<RoboCup::RobotInfo&>(robotInfo) = team.players[robotInfo.number - 1];
  robotInfo.naoBodyType = naoBodyType;
  robotInfo.naoHeadType = naoHeadType;

  if (lastPlayerNumberUpdate != theUSBSettings.updateTimestamp)
  {
    lastPlayerNumberUpdate = theUSBSettings.updateTimestamp;
    OUTPUT_TEXT("RawGameInfoProvider: Set player number " << robotInfo.number << "!");
  }
}

void RawGameInfoProvider::execute(tf::Subflow&)
{
  bool enable = Build::targetRobot();
  MODIFY("module:RawGameInfoProvider:enable", enable);

  if (enable && !initialized)
    initialize();

#ifdef TARGET_ROBOT
  ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
  transitionToFramework = naoBodyV6.getTransitionToFramework();
#else
  transitionToFramework = 1.f;
#endif

  uint8_t lastPenalty = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty;
  if (enable && receive())
  {
    lastReceivedTimeStamp = theFrameInfo.time;
    RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1];
    lastSecsTillUnpenalized = team.players[theRobotInfo.number - 1].secsTillUnpenalised;
  }
  else if (theFrameInfo.getTimeSince(lastReceivedTimeStamp) > gameControllerTimeout)
  {
    if (gameCtrlData.teams[0].teamNumber == 0 && gameCtrlData.teams[1].teamNumber == 0)
    {
      gameCtrlData.teams[0].teamNumber = static_cast<uint8_t>(this->teamNumber);
      gameCtrlData.teams[0].fieldPlayerColour = ownColor;
      gameCtrlData.teams[0].goalkeeperColour = ownKeeperColor;
      gameCtrlData.teams[0].goalkeeper = keeper;
      gameCtrlData.teams[1].fieldPlayerColour = oppColor;
      gameCtrlData.teams[1].goalkeeperColour = oppKeeperColor;
    }
    else // already received packages; reset game and robot infos
    {
      // TODO: try to guess better infos here, depending on visual or other cues!
      RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1];
      if (theFrameInfo.getTimeSince(lastReceivedTimeStamp) / 1000 > lastSecsTillUnpenalized && team.players[theRobotInfo.number - 1].penalty != PENALTY_MANUAL)
        team.players[theRobotInfo.number - 1].penalty = PENALTY_NONE;
    }
  }
  if (lastGameState != gameCtrlData.state)
  {
    lastGameState = gameCtrlData.state;
    timeStampGameStateChanged = theFrameInfo.time;
  }
  updateGameState();
  if (lastPenalty != gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty)
  {
    lastPenalty = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty;
    timeStampPenaltyChanged = theFrameInfo.time;
  }

  if (enable && theFrameInfo.getTimeSince(lastSentTimeStamp) > aliveDelay && transitionToFramework >= 1.f)
  {
    if (send())
      lastSentTimeStamp = theFrameInfo.time;
  }
}

bool RawGameInfoProvider::receive()
{
  bool received = false;
  int size;
  RoboCup::RoboCupGameControlData buffer;
  struct sockaddr_in from;
  while (udp && (size = udp->read(reinterpret_cast<char*>(&buffer), sizeof(buffer), from)) > 0)
  {
    if (size == sizeof(buffer) && !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) && buffer.version == GAMECONTROLLER_STRUCT_VERSION
        && (buffer.teams[0].teamNumber == this->teamNumber || buffer.teams[1].teamNumber == this->teamNumber))
    {
      gameCtrlData = buffer;
      if (memcmp(&gameControllerAddress, &from.sin_addr, sizeof(in_addr)))
      {
        memcpy(&gameControllerAddress, &from.sin_addr, sizeof(in_addr));
        udp->setTarget(inet_ntoa(gameControllerAddress), GAMECONTROLLER_RETURN_PORT);
      }

      *reinterpret_cast<unsigned int*>(gameCtrlDataSender.data()) = ntohl(from.sin_addr.s_addr);

      received = true;
    }
  }
  return received;
}

bool RawGameInfoProvider::send()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  returnPacket.teamNum = static_cast<uint8_t>(this->teamNumber);
  returnPacket.fallen = theFallDownState.state != FallDownState::State::upright;

  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;

  returnPacket.ballAge = (theBallModel.timeWhenLastSeen == 0) ? -1.f : theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;
  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();
  return !udp || udp->write(reinterpret_cast<const char*>(&returnPacket), sizeof(returnPacket));
}

void RawGameInfoProvider::initialize()
{
  udp = std::make_unique<UdpComm>();
  if (!udp->setBlocking(false) || !udp->setBroadcast(true) || !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) || !udp->setLoopback(false))
  {
    fprintf(stderr, "RawGameInfoProvider: Could not open UDP port\n");
    udp.reset();
    // continue, because button interface will still work
    initialized = false;
  }
  else
  {
    initialized = true;
  }
  if (timeStampGameStateChanged == 0)
  {
    timeStampPenaltyChanged = theFrameInfo.time;
    timeStampGameStateChanged = theFrameInfo.time;
  }
}

void RawGameInfoProvider::updateGameState()
{
  bool timeOut = theFrameInfo.getTimeSince(lastReceivedTimeStamp) > gameControllerTimeout;
  ownPenalty = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty;

  if (transitionToFramework == 0.f)
  {
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
    if (theKeyStates.pressed[KeyStates::chest])
      firstChestButtonPressed = true;
  }
  else if (timeOut && !firstChestButtonPressed && !lastChestButtonPressed && theKeyStates.pressed[KeyStates::chest] && !theKeyStates.pressed[KeyStates::headFront]
      && !theKeyStates.pressed[KeyStates::headMiddle] && !theKeyStates.pressed[KeyStates::headRear])
  {
    if (theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
    {
      gameCtrlData.state = STATE_INITIAL;
      ownPenalty = PENALTY_NONE;
    }
    else
    {
      switch (gameCtrlData.state)
      {
      case STATE_INITIAL:
      {
        gameCtrlData.state = STATE_PLAYING;
        ownPenalty = PENALTY_MANUAL;
        break;
      }
      case STATE_SET:
      case STATE_FINISHED:
      {
        gameCtrlData.state = STATE_INITIAL;
        ownPenalty = PENALTY_NONE;
        break;
      }
      case STATE_READY:
        gameCtrlData.state = STATE_PLAYING;
      case STATE_PLAYING:
      {
        ownPenalty = (ownPenalty != PENALTY_NONE) ? PENALTY_NONE : PENALTY_MANUAL;
      }
      default:
        break;
      }
    }
  }
  if (!theKeyStates.pressed[KeyStates::chest])
    firstChestButtonPressed = false;

  lastChestButtonPressed = theKeyStates.pressed[KeyStates::chest];

  gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty = ownPenalty;
}
