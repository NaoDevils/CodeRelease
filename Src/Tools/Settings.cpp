/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#ifdef TARGET_ROBOT
#include "Platform/Linux/NaoBody.h"
#include "Platform/Linux/NaoBodyV6.h"
#include <cstdio>
#include <unistd.h>
#endif

#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Utils/dorsh/models/Robot.h"

STREAMABLE(Robots,
{,
  (std::vector<RobotConfig>) robotsIds,
});

bool Settings::recover = false;

Settings Settings::settings(true);
bool Settings::loaded = false;

Settings::Settings(bool master)
{
  ASSERT(master);
}

Settings::Settings()
{
  static_assert(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black, "These macros and enums have to match!");
  if(!loaded)
  {
    VERIFY(settings.load());
    loaded = true;
  }
  *this = settings;

#ifdef TARGET_SIM
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    int index = atoi(RoboCupCtrl::controller->getRobotName().c_str() + 5) - 1;
    teamNumber = index < 6 ? 1 : 2;
    teamPort = 10000 + teamNumber;
    teamColor = index < 6 ? blue : red;
    playerNumber = index % 6 + 1;
  }

  robotName = "Nao";

  ConsoleRoboCupCtrl* ctrl = dynamic_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  if(ctrl)
  {
    std::string logFileName = ctrl->getLogFile();
    if(logFileName != "")
    {
      QRegExp re("[0-9]_[A-Za-z]*_[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]_[0-9][0-9]-[0-9][0-9]-[0-9][0-9].log", Qt::CaseSensitive, QRegExp::RegExp2);
      int pos = re.indexIn(logFileName.c_str());
      if(pos != -1)
      {
        robotName = logFileName.substr(pos + 2);
        robotName = robotName.substr(0, robotName.find("_"));
      }
      else
        robotName = "Default";
    }
  }

  bodyName = robotName.c_str();

#endif

}

bool Settings::load()
{

  if (!Global::theStreamHandler)
  {
    static StreamHandler streamHandler;
    Global::theStreamHandler = &streamHandler;
  }

  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
  {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robotName.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  robotName = SystemCall::getHostName();
  std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
  {
    TRACE("Could not load robots.cfg");
    return false;
  }
  else
  {
    Robots robots;
    robotsStream >> robots;

    std::string bodyId;
    
	  // wait for NaoQi/LoLA
    if (naoVersion == RobotConfig::V6)
    {
      NaoBodyV6 naoBody;
      if(!naoBody.init())
      {
        fprintf(stderr, "B-Human: Waiting for LoLA via ndevilsbase...\n");
        do
        {
          usleep(1000000);
        }
        while(!naoBody.init());
      }
      
      bodyId = std::string(NaoBodyV6().getBodyId());
    }
    else
    {
      NaoBody naoBody;
      if(!naoBody.init())
      {
        fprintf(stderr, "B-Human: Waiting for NaoQi via libbhuman...\n");
        do
        {
          usleep(1000000);
        }
        while(!naoBody.init());
      }

      bodyId = std::string(NaoBody().getBodyId());
    }

    for(const RobotConfig& robot : robots.robotsIds)
    {
      if(robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        naoVersion = robot.naoVersion;
        break;
      }
    }
    if(bodyId.empty())
    {
      TRACE("Could not find bodyId in robots.cfg");
      return false;
    }
  }
#else
  robotName = "Nao";
  bodyName = "Nao";
  naoVersion = RobotConfig::V5;
#endif

#ifdef TARGET_ROBOT
  if(robotName == bodyName)
    printf("Hi, I am %s.\n", robotName.c_str());
  else
    printf("Hi, I am %s (using %ss Body).\n", robotName.c_str(), bodyName.c_str());
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", getName(teamColor));
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
  printf("gameMode %s\n", getName(gameMode));
#endif

  return true;
}
