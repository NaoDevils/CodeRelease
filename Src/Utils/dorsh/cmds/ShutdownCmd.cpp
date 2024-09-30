#include "cmds/ShutdownCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include <cstdlib>

ShutdownCmd ShutdownCmd::theShutdownCmd;

ShutdownCmd::ShutdownCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ShutdownCmd::getName() const
{
  return "shutdown";
}

std::string ShutdownCmd::getDescription() const
{
  return "shutdown robot";
}

bool ShutdownCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if (!params.empty() && params[0] != "-s" && params[0] != "all")
  {
    context.errorLine("Unrecognized parameters.");
    return false;
  }
  return true;
}

Task* ShutdownCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new ShutdownTask(context, &robot);
}

ShutdownCmd::ShutdownTask::ShutdownTask(Context& context, RobotConfigDorsh* robot) : RobotTask(context, robot) {}

bool ShutdownCmd::ShutdownTask::execute()
{
  std::string ip = robot->getBestIP(context());

  context().printLine(robot->name + ": Shutting down...");
  const auto [cmd, params] = remoteCommand("sudo poweroff", ip);
  ProcessRunner r(context(), cmd, params);
  r.run();
  if (r.error())
  {
    context().errorLine(robot->name + ": Shutdown failed.");
    return false;
  }
  context().printLine(robot->name + ": Shutdown finished.");
  return true;
}
