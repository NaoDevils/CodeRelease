#include "Platform/File.h"
#include "Utils/dorsh/cmds/SSHCmd.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/Session.h"
#include <cstdlib>

SSHCmd SSHCmd::theSSHCmd;

SSHCmd::SSHCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SSHCmd::getName() const
{
  return "ssh";
}

std::string SSHCmd::getDescription() const
{
  return "executes an command via ssh or opens a ssh session";
}

bool SSHCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  command = "";
  for (std::vector<std::string>::const_iterator param = params.begin(); param != params.end(); ++param)
    command += " " + *param;

  return true;
}

Task* SSHCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new SSHTask(context, &robot, command);
}

SSHCmd::SSHTask::SSHTask(Context& context, RobotConfigDorsh* robot, const std::string& command) : RobotTask(context, robot), command(command) {}

bool SSHCmd::SSHTask::execute()
{
  QString runCommand = "";
  QStringList runParams;
  if (command == "")
  {
    std::tie(runCommand, runParams) = connectCommand(robot->getBestIP(context()));
  }
  else
  {
    std::tie(runCommand, runParams) = remoteCommandForQProcess(command, robot->getBestIP(context()));
  }

  ProcessRunner r(context(), runCommand, runParams);
  r.run();

  if (r.error())
  {
    context().errorLine(robot->name + ": ssh command" + command + " failed.");
    return false;
  }
  return true;
}
