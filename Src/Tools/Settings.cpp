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
#include "Platform/Linux/NaoBodyV6.h"
#include <iostream>
#include <chrono>
#include <thread>
#endif

#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Streams/StreamHandler.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Utils/dorsh/models/Robot.h"

STREAMABLE(Robots,,
  (std::vector<RobotConfig>) robotsIds
);

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
  if (!loaded)
  {
    VERIFY(settings.load());
    loaded = true;
  }
  *this = settings;

#ifdef TARGET_SIM
  if (SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    int index = atoi(RoboCupCtrl::controller->getRobotName().c_str() + 5) - 1;
    teamNumber = index < MAX_NUM_PLAYERS ? 1 : 2;
    teamPort = 10000 + teamNumber;
    teamColor = index < MAX_NUM_PLAYERS ? blue : red;
    playerNumber = index % MAX_NUM_PLAYERS + 1;
  }

  robotName = "Nao";

  ConsoleRoboCupCtrl* ctrl = dynamic_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  if (ctrl)
  {
    std::string logFileName = ctrl->getLogFile();
    if (logFileName != "")
    {
      QRegExp re("[0-9]_[A-Za-z]*_[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]_[0-9][0-9]-[0-9][0-9]-[0-9][0-9].log", Qt::CaseSensitive, QRegExp::RegExp2);
      int pos = re.indexIn(logFileName.c_str());
      if (pos != -1)
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

  std::string bhdir = File::getBHDir();
#ifdef TARGET_ROBOT
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if (!robotsStream.exists())
  {
    TRACE("Could not load robots.cfg");
    return false;
  }
  else
  {
    Robots robots;
    robotsStream >> robots;

    std::string bodyId, headId;

    // wait for LoLA
    NaoBodyV6 naoBody;
    while (!naoBody.init())
    {
      std::cerr << "Waiting for LoLA via ndevilsbase...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (*NaoBodyV6().getBodyId() == 0)
    {
      std::cerr << "Waiting for body id...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    bodyId = std::string(NaoBodyV6().getBodyId());

    while (*NaoBodyV6().getHeadId() == 0)
    {
      std::cerr << "Waiting for head id...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    headId = std::string(NaoBodyV6().getHeadId());

    for (const RobotConfig& robot : robots.robotsIds)
    {
      if (robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        naoVersion = robot.naoVersion;
      }
      if (robot.headId == headId)
      {
        robotName = robot.name;
      }
    }

    if (bodyName.empty())
    {
      bodyName = bodyId;
      TRACE("Could not find bodyId in robots.cfg.");
    }

    if (robotName.empty())
    {
      robotName = headId;
      TRACE("Could not find headId in robots.cfg.");
    }
  }
#else
  robotName = "Nao";
  bodyName = "Nao";
  naoVersion = RobotConfig::V6;
#endif

  // the default config hierarchy does not work here because
  // the global settings object is not initialized at this point
  const std::list<std::string> names = {bhdir + "/Config/settings.cfg", bhdir + "/Config/Robots/" + robotName + "/Head/settings.cfg", bhdir + "/Config/Robots/" + bodyName + "/Body/settings.cfg"};
  InMapFile stream(names);
  if (stream.exists())
    stream >> *this;
  else
  {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robotName.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  std::cout << "Hi, I am " << robotName;
  if (robotName == bodyName)
    std::cout << ".\n";
  else
    std::cout << " (using body of " << bodyName << ").\n";

  std::cout << "teamNumber: " << teamNumber << "\n";
  std::cout << "teamPort: " << teamPort << "\n";
  std::cout << "teamColor: " << getName(teamColor) << "\n";
  std::cout << "playerNumber: " << playerNumber << "\n";
  std::cout << "gameMode: " << getName(gameMode) << "\n";
  std::cout << "overlays:";
  for (const std::string& overlay : overlays)
    std::cout << " " << overlay;
  std::cout << "\n";
#endif

  return true;
}
