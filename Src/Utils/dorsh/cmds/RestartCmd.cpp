#include <cstdlib>
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/cmds/RestartCmd.h"
#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/tools/Sleeper.h"
#include "Utils/dorsh/tools/StringTools.h"
#include <iostream>
RestartCmd RestartCmd::theRestartCmd;

RestartCmd::RestartCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string RestartCmd::getName() const
{
  return "restart";
}

std::string RestartCmd::getDescription() const
{
  return std::string("[ naodevils | naodevilsbase | full | robot ]\n") + "Restarts naodevils, naodevilsbase, both or the robot.";
}

bool RestartCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  type = NO_RESTART;
  if (params.size() > 1)
  {
    context.errorLine("Too many parameters.");
    return false;
  }
  else if (params.size() == 0 || params[0] == "naodevils")
    type = NAODEVILS;
  else if (params[0] == "naodevilsbase")
    type = NAODEVILSBASE;
  else if (params[0] == "robot")
    type = ROBOT;
  else if (params[0] == "full")
    type = NAODEVILS_AND_NAODEVILSBASE;

  return true;
}

Task* RestartCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new RestartTask(context, &robot, type);
}

std::vector<std::string> RestartCmd::complete(const std::string& cmdLine) const
{
  const size_t ARGS_SIZE = 4;
  std::string arguments[ARGS_SIZE] = {"naodevils", "naodevilsbase", "full", "robot"};

  std::vector<std::string> result;
  std::vector<std::string> commandWithArgs = split(cmdLine);
  if (commandWithArgs.size() == 2)
  {
    for (size_t i = 0; i < ARGS_SIZE; i++)
    {
      if (arguments[i].find(commandWithArgs[1]) == 0)
        result.push_back(arguments[i].substr(commandWithArgs[1].size(), arguments[i].size()));
    }
  }
  return result;
}

RestartCmd::RestartTask::RestartTask(Context& context, RobotConfigDorsh* robot, RestartType type) : RobotTask(context, robot), type(type) {}

void RestartCmd::RestartTask::reportStatus(const ProcessRunner& r)
{
  if (r.error())
    context().errorLine("Robot \"" + robot->name + "\" is unreachable");
  else
    context().printLine("Robot \"" + robot->name + "\" restarted");
}

bool RestartCmd::RestartTask::execute()
{
  std::string ip = robot->getBestIP(context());
  QString naoVersion = QString::fromStdString(RobotConfigDorsh::getName(robot->naoVersion));


  context().printLine("restart: using ip " + ip + " for " + robot->name + ".");

  if (type < SINGLE_COMMANDS)
  {
    if (type == NAODEVILS)
    {
      const auto [cmd, params] = remoteCommand("systemctl --user restart naodevils", ip);
      ProcessRunner r(context(), cmd, params);
      r.run();
      reportStatus(r);
    }
    else if (type == NAODEVILSBASE)
    {
      const auto [cmd, params] = remoteCommand("systemctl --user restart naodevilsbase", ip);
      ProcessRunner r(context(), cmd, params);
      r.run();
      if (r.error())
      {
        context().errorLine(robot->name + ": Could not restart naodevilsbase.");
        return false;
      }
      else
        context().printLine(robot->name + ": restarted naodevilsbase.");
    }
    else if (type == ROBOT)
    {
      const auto [cmd, params] = remoteCommand("sudo reboot", ip);
      ProcessRunner r(context(), cmd, params);
      r.run();
      reportStatus(r);
    }
    else
    {
      context().errorLine("Unkown restart command.");
      return false;
    }
  }
  else if (type == NAODEVILS_AND_NAODEVILSBASE)
  {
    auto [cmd, params] = remoteCommand("systemctl --user stop naodevils", ip);
    ProcessRunner r(context(), cmd, params);
    r.run();
    if (r.error())
    {
      context().errorLine(robot->name + ": Failed to stop naodevils.");
      return false;
    }
    else
    {
      context().printLine(robot->name + ": naodevils stopped");
    }
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);

    std::tie(cmd, params) = remoteCommand("systemctl --user restart naodevilsbase", ip);
    r = ProcessRunner(context(), cmd, params);
    r.run();
    if (r.error())
    {
      context().errorLine(robot->name + ": Failed to restart naodevilsbase.");
      return false;
    }
    else
      context().printLine(robot->name + ": naodevilsbase restarted");
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);

    std::tie(cmd, params) = remoteCommand("systemctl --user start naodevils", ip);
    r = ProcessRunner(context(), cmd, params);
    r.run();
    if (r.error())
    {
      context().errorLine(robot->name + ": Failed to start naodevils.");
      return false;
    }
    else
      context().printLine(robot->name + ": naodevils started");
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);
    context().printLine(robot->name + ": Done");
  }
  else
  {
    context().errorLine("Unkown restart command.");
    return false;
  }

  return true;
}
