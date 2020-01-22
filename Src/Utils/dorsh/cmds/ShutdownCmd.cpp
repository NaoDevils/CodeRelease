#include "Utils/dorsh/cmds/ShutdownCmd.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/tools/StringTools.h"
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
  return "bhumand stop && naoqid stop && halt";
}

bool ShutdownCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(!params.empty() && params[0] != "-s")
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

ShutdownCmd::ShutdownTask::ShutdownTask(Context& context, RobotConfigDorsh* robot)
  : RobotTask(context, robot)
{}

bool ShutdownCmd::ShutdownTask::execute()
{
  std::string ip = robot->getBestIP(context());
  QString naoVersion = fromString(RobotConfigDorsh::getName(robot->naoVersion));

  context().printLine(robot->name + ": Shutting down...");
  std::string command = "";
  if (naoVersion == "V6") {
    command = remoteCommand("systemctl --user stop bhumand; sleep 5s; sudo halt", ip);
  } else {
    command = remoteCommand("halt", ip);
  }
  ProcessRunner r(context(), fromString(command));
  r = ProcessRunner(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Shutdown failed.");
    return false;
  }
  context().printLine(robot->name + ": Shutdown finished.");
  return true;
}
