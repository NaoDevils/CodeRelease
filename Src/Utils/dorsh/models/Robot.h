#pragma once

#include "Tools/Configuration/RobotConfig.h"

class Context;
class Session;
class Team;

STREAMABLE_WITH_BASE(RobotConfigDorsh, RobotConfig,
{
  std::string getBestIP(const Context& context) const;
  static void initRobotsByName(std::map<std::string, RobotConfigDorsh*> &robotsByName)
  ,
});
