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
}

void RawGameInfoProvider::update(RawGameInfo& rawGameInfo)
{
  memcpy(&static_cast<RoboCup::RoboCupGameControlData&>(rawGameInfo), &gameCtrlData, (char*)gameCtrlData.teams - (char*)&gameCtrlData);

  // set team number for Logger
  rawGameInfo.oppTeamNumber = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 1 : 0].teamNumber;

  rawGameInfo.timeLastPackageReceived = lastReceivedTimeStamp;
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
      gameCtrlData.teams[0].teamColour = static_cast<uint8_t>(ownColor);
      gameCtrlData.teams[1].teamColour = static_cast<uint8_t>(oppColor);
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
  while (udp && (size = udp->read((char*)&buffer, sizeof(buffer), from)) > 0)
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

      received = true;
    }
  }
  return received;
}

bool RawGameInfoProvider::send()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.playerNum = (uint8_t)theRobotInfo.number;
  returnPacket.teamNum = (uint8_t)this->teamNumber;
  returnPacket.fallen = theFallDownState.state != FallDownState::State::upright;

  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;

  returnPacket.ballAge = (theBallModel.timeWhenLastSeen == 0) ? -1.f : theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();
  return !udp || udp->write((const char*)&returnPacket, sizeof(returnPacket));
}

void RawGameInfoProvider::initialize()
{
  udp = std::make_unique<UdpComm>();
  if (!udp->setBlocking(false) || !udp->setBroadcast(true) || !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) || !udp->setLoopback(false))
  {
    fprintf(stderr, "libgamectrl: Could not open UDP port\n");
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
  int timeInGameState = theFrameInfo.getTimeSince(timeStampGameStateChanged);
  int timeSincePenaltyChange = theFrameInfo.getTimeSince(timeStampPenaltyChanged);
  ownPenalty = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty;

  if (transitionToFramework < 1.f)
  {
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }
  else if (timeOut && timeInGameState > 1000 && timeSincePenaltyChange > 1000)
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
        if (chestButtonPressed && !theKeyStates.pressed[KeyStates::chest])
        {
          gameCtrlData.state = STATE_PLAYING;
          ownPenalty = PENALTY_MANUAL;
        }
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
        if (theKeyStates.pressed[KeyStates::chest])
          ownPenalty = (ownPenalty != PENALTY_NONE) ? PENALTY_NONE : PENALTY_MANUAL;
      }
      default:
        break;
      }
    }
  }
  chestButtonPressed = theKeyStates.pressed[KeyStates::chest] && !theKeyStates.pressed[KeyStates::headFront] && !theKeyStates.pressed[KeyStates::headMiddle]
      && !theKeyStates.pressed[KeyStates::headRear];
  gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == this->teamNumber ? 0 : 1].players[theRobotInfo.number - 1].penalty = ownPenalty;
}
