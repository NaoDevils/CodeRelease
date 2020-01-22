#include "Platform/BHAssert.h"
#include "Utils/dorsh/Session.h"
#include "Utils/dorsh/models/Robot.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/agents/DataAgent.h"
#include "Utils/dorsh/cmdlib/IConsole.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include <iostream>

Session::Session()
  : console(0),
    logLevel(ALL),
    dataAgent(0),
    robotsByName()
{
}

void Session::setTeamNumber(Team* team)
{
  this->teamNumber = &team->number;
}

std::string Session::getBestIP(const Context& context, const RobotConfigDorsh* robot)
{
  std::string ip;
  Team* team = context.getSelectedTeam();

  if (team->deployDevice == "wlan")
    ip = robot->wlan;
  else if (team->deployDevice == "lan")
    ip = robot->lan;
  else if (team->deployDevice == "auto")
    ip = dataAgent->isLANReachable(robot) ? robot->lan : robot->wlan;
  else
    ASSERT(false);

  return ip;
}

Session& Session::getInstance()
{
  static Session theInstance;
  return theInstance;
}

void Session::registerConsole(IConsole* console)
{
  this->console = console;
  log(TRACE, "Session: Console registered.");
}

IConsole* Session::getConsole()
{
  return console;
}

void Session::log(LogLevel logLevel, const std::string& error)
{
  if(logLevel >= this->logLevel)
  {
    if(console)
      console->errorLine(error);
    else
      std::cerr << error << std::endl;
  }
}

bool Session::isReachable(const Context& context, const RobotConfigDorsh* robot)
{
  Team* team = context.getSelectedTeam();

  if (team->deployDevice == "lan")
  {
    return true;
  }
  else if (team->deployDevice == "wlan")
  {
    return true;
  }
  else if (team->deployDevice == "auto")
  {
    return dataAgent->isLANReachable(robot) || dataAgent->isWLANReachable(robot);
  }
  else
  {
    ASSERT(false);
  }
  return false;
}

ENetwork Session::getBestNetwork(const RobotConfigDorsh* robot)
{
  if (dataAgent->isLANReachable(robot))
  {
    return ENetwork::LAN;
  }
  else if (dataAgent->isWLANReachable(robot))
  {
    return ENetwork::WLAN;
  }
  else
  {
    return ENetwork::NONE;
  }
}


void Session::registerDataListener(QObject* qObject, RobotConfigDorsh* robot)
{
  if (dataAgent)
  {
    log(TRACE, "Session: Registered data listener.");
    QObject::connect(dataAgent, SIGNAL(newSensorData(std::map<std::string, std::string>)),
      qObject, SLOT(changeData(std::map<std::string, std::string>)));
    dataAgent->reset(robot);
  }
  else
  {
    log(WARN, "Session: Could not register data listener. No dataAgent initialized.");
  }
}

void Session::removeDataListener(QObject* qObject, RobotConfigDorsh* robot)
{
  log(TRACE, "Session: Removed data listener.");
  QObject::disconnect(dataAgent, SIGNAL(newSensorData(std::map<std::string, std::string>)),
    qObject, SLOT(changeData(std::map<std::string, std::string>)));
  dataAgent->reset(robot);
}

void Session::registerWifiListener(QObject* qObject, RobotConfigDorsh* robot)
{
  if (dataAgent)
  {
    log(TRACE, "Session: Registered wifi listener.");
    QObject::connect(dataAgent, SIGNAL(wifiConnection(std::string, bool)),
                     qObject, SLOT(changeWifi(std::string, bool)));
    dataAgent->reset(robot);
  }
  else
  {
    log(WARN, "Session: Could not register wifi listener. No dataAgent initialized.");
  }
}

void Session::removeWifiListener(QObject* qObject, RobotConfigDorsh* robot)
{
  log(TRACE, "Session: Removed wifi listener.");
  QObject::disconnect(dataAgent, SIGNAL(wifiConnection(std::string, bool)),
                      qObject, SLOT(changeWifi(std::string, bool)));
  dataAgent->reset(robot);
}