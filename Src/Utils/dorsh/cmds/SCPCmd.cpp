#include <cstdlib>
#include "cmdlib/Commands.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/SCPCmd.h"
#include "Session.h"
#include "tools/ShellTools.h"

SCPCmd SCPCmd::theSCPCmd;

SCPCmd::SCPCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SCPCmd::getName() const
{
  return "scp";
}

std::string SCPCmd::getDescription() const
{
  return "copies one file via SCP to robot or from robot (when first param @prefixed)";
}

bool SCPCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if (params.size() < 2)
  {
    context.errorLine("too few parameters");
    return false;
  }

  fromRobot = params[0][0] == '@';
  fromFile = fromRobot ? std::string(params[0]).replace(0, 1, "") : params[0];
  size_t i = 1;
  if (fromFile[0] == '"')
  {
    fromFile = fromFile.replace(0, 1, "");
    while (i < params.size() && fromFile[fromFile.size() - 1] != '"')
      fromFile += " " + params[i++];

    if (i == params.size())
    {
      context.errorLine("too few parameters (or missing \")");
      return false;
    }

    fromFile = fromFile.replace(fromFile.size() - 1, 1, "");
  }

  toFile = params[i++];
  if (toFile[0] == '"')
  {
    toFile = toFile.replace(0, 1, "");
    while (i < params.size() && toFile[toFile.size() - 1] != '"')
      toFile += " " + params[i++];

    if (toFile[toFile.size() - 1] != '"')
    {
      context.errorLine("missing \"");
      return false;
    }

    toFile = toFile.replace(toFile.size() - 1, 1, "");
  }

  if (i != params.size())
  {
    context.errorLine("too many parameters");
    return false;
  }

  return true;
}

Task* SCPCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  if (fromRobot)
    return new SCPTask(context, &robot, scpCommandFromRobot(fromFile, robot.getBestIP(context), toFile));
  else
    return new SCPTask(context, &robot, scpCommandToRobot(fromFile, robot.getBestIP(context), toFile));
}

SCPCmd::SCPTask::SCPTask(Context& context, RobotConfigDorsh* robot, std::tuple<QString, QStringList> command) : RobotTask(context, robot), command(std::move(command)) {}

bool SCPCmd::SCPTask::execute()
{
  const auto& [cmd, params] = command;
  ProcessRunner r(context(), cmd, params);
  r.run();

  if (r.error())
  {
    context().errorLine(robot->name + ": scp command failed.");
    return false;
  }
  return true;
}
