/**
* @file Modules/Infrastructure/RawGameInfoProvider.h
* This file implements a module that provides information game info.
*/

#include "RawGameInfoProvider.h"
#include "Tools/Settings.h"

MAKE_MODULE(RawGameInfoProvider, cognitionInfrastructure)

#ifdef TARGET_ROBOT
static const int ALIVE_DELAY = 1000; /**< Send an alive signal every 1000 ms. */

void RawGameInfoProvider::update(OpponentTeamInfo& opponentTeamInfo)
{
  if (lastExecutedTimeStamp != theFrameInfo.time)
  {
    lastExecutedTimeStamp = theFrameInfo.time;
    execute();
  }
  (RoboCup::TeamInfo&) opponentTeamInfo = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber != Global::getSettings().teamNumber ? 0 : 1];
}

void RawGameInfoProvider::update(OwnTeamInfo& ownTeamInfo)
{
  if (lastExecutedTimeStamp != theFrameInfo.time)
  {
    lastExecutedTimeStamp = theFrameInfo.time;
    execute();
  }
  (RoboCup::TeamInfo&) ownTeamInfo = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
}

void RawGameInfoProvider::update(RawGameInfo& rawGameInfo)
{
  if (lastExecutedTimeStamp != theFrameInfo.time)
  {
    lastExecutedTimeStamp = theFrameInfo.time;
    execute();
  }
  memcpy(&(RoboCup::RoboCupGameControlData&) rawGameInfo, &gameCtrlData, (char*) gameCtrlData.teams - (char*) &gameCtrlData);

  // set team number for Logger
  rawGameInfo.oppTeamNumber = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0].teamNumber;
}

void RawGameInfoProvider::update(RobotInfo& robotInfo)
{
  if (lastExecutedTimeStamp != theFrameInfo.time)
  {
    lastExecutedTimeStamp = theFrameInfo.time;
    execute();
  }
  RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  (RoboCup::RobotInfo&) robotInfo = team.players[Global::getSettings().playerNumber - 1];
  robotInfo.naoBodyType = naoBodyType;
  robotInfo.naoHeadType = naoHeadType;
  robotInfo.number = Global::getSettings().playerNumber;
  if (Global::getSettings().naoVersion == RobotConfig::V6)
    robotInfo.transitionToBhuman = naoBodyV6.getTransitionToBhuman();
  else
    robotInfo.transitionToBhuman = naoBody.getTransitionToBhuman();
  transitionToBhuman = robotInfo.transitionToBhuman;
}

void RawGameInfoProvider::execute()
{
  if (!initialized)
    initialize();
  uint8_t lastPenalty = 
    gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty;
  if (receive())
  {
    lastReceivedTimeStamp = theFrameInfo.time;
    RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
    lastSecsTillUnpenalized = team.players[Global::getSettings().playerNumber - 1].secsTillUnpenalised; 
  }
  else if (theFrameInfo.getTimeSince(lastReceivedTimeStamp) > GAMECONTROLLER_TIMEOUT)
  {
    if (gameCtrlData.teams[0].teamNumber == 0 && gameCtrlData.teams[1].teamNumber == 0)
    {
      gameCtrlData.teams[0].teamNumber = (uint8_t)Global::getSettings().teamNumber;
      gameCtrlData.teams[0].teamColour = TEAM_YELLOW;
    }
    else // already received packages; reset game and robot infos
    {
      // TODO: try to guess better infos here, depending on visual or other cues!
      RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
      if (theFrameInfo.getTimeSince(lastReceivedTimeStamp) / 1000 > lastSecsTillUnpenalized 
        && team.players[Global::getSettings().playerNumber].penalty != PENALTY_MANUAL)
        team.players[Global::getSettings().playerNumber].penalty = PENALTY_NONE;
    }
  }
  if (lastGameState != gameCtrlData.state)
  {
    lastGameState = gameCtrlData.state;
    timeStampGameStateChanged = theFrameInfo.time;
  }
  updateGameState();
  if (lastPenalty !=
    gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty)
  {
    lastPenalty = 
      gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty;
    timeStampPenaltyChanged = theFrameInfo.time;
  }

  float transitionToBhuman;
  if (Global::getSettings().naoVersion == RobotConfig::V6)
    transitionToBhuman = naoBodyV6.getTransitionToBhuman();
  else
    transitionToBhuman = naoBody.getTransitionToBhuman();

  if (theFrameInfo.getTimeSince(lastSentTimeStamp) > ALIVE_DELAY && transitionToBhuman >= 1.f)
  {
    if (!send())
    {
      // output?
    }
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
		if (size == sizeof(buffer) &&
			!std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
			buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
			(buffer.teams[0].teamNumber == Global::getSettings().teamNumber ||
				buffer.teams[1].teamNumber == Global::getSettings().teamNumber))
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
	returnPacket.team = (uint8_t)Global::getSettings().teamNumber;
	returnPacket.player = (uint8_t)Global::getSettings().playerNumber;
	returnPacket.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
	return !udp || udp->write((const char*)&returnPacket, sizeof(returnPacket));
}

void RawGameInfoProvider::initialize()
{
  if (udp) delete udp;
  udp = new UdpComm();
  if (!udp->setBlocking(false) ||
    !udp->setBroadcast(true) ||
    !udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
    !udp->setLoopback(false))
  {
    fprintf(stderr, "libgamectrl: Could not open UDP port\n");
    delete udp;
    udp = 0;
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
#endif

void RawGameInfoProvider::updateGameState()
{
  bool timeOut = theFrameInfo.getTimeSince(lastReceivedTimeStamp) > GAMECONTROLLER_TIMEOUT;
  int timeInGameState = theFrameInfo.getTimeSince(timeStampGameStateChanged);
  int timeSincePenaltyChange = theFrameInfo.getTimeSince(timeStampPenaltyChanged);
#ifdef TARGET_ROBOT  
  ownPenalty = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty;
#endif
  if (transitionToBhuman != 1.f)
  {
    memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  }
  else if (timeOut && timeInGameState > 1000 &&  timeSincePenaltyChange > 1000)
  {
    switch (gameCtrlData.state)
    {
    case STATE_INITIAL:
    {
      if (theKeyStates.pressed[KeyStates::chest])
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
#ifdef TARGET_ROBOT  
  gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty = ownPenalty;
#endif
}