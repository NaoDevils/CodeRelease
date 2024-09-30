#include "models/Robot.h"
#include "Session.h"
#include "tools/Platform.h"
#include <iostream>

#include "Tools/Streams/InStreams.h"
#include "Tools/Configuration/RobotConfig.h"
#include <iostream>

#if defined(LINUX) || defined(MACOS)
#include <cstdlib>
#include <sys/types.h>
#include <cerrno>
#endif

STREAMABLE(RobotConfigs,,
  (std::vector<RobotConfigDorsh>) robotsIds
);

std::string RobotConfigDorsh::getBestIP(const Context& context) const
{
  return Session::getInstance().getBestIP(context, this);
}

void RobotConfigDorsh::initRobotsByName(std::map<std::string, std::unique_ptr<RobotConfigDorsh>>& robotsByName)
{
  RobotConfigs robots;
  std::string robotsDir = "Robots";
  InMapFile stream(linuxToPlatformPath(robotsDir + "/robots.cfg"));
  if (stream.exists())
  {
    stream >> robots;
  }
  else
    perror(("Cannot open " + robotsDir).c_str());
  for (size_t i = 0; i < robots.robotsIds.size(); ++i)
  {
    robotsByName[robots.robotsIds[i].name] = std::make_unique<RobotConfigDorsh>(robots.robotsIds[i]);
  }
}
