#include "Platform/File.h"
#include "Utils/dorsh/cmds/SSHCmd.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/ShellTools.h"
#include "Utils/dorsh/tools/StringTools.h"
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
  for(std::vector<std::string>::const_iterator param = params.begin(); param != params.end(); ++param)
    command += " " + *param;

  return true;
}

Task* SSHCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new SSHTask(context, &robot, command);
}

SSHCmd::SSHTask::SSHTask(Context& context,
                         RobotConfigDorsh* robot,
                         const std::string& command)
  : RobotTask(context, robot),
    command(command)
{}

bool SSHCmd::SSHTask::execute()
{
  std::string commandToRun = "";
  if(command == "")
  {
#ifdef WINDOWS
    commandToRun = "cmd /c start " + connectCommand(robot->getBestIP(context()));
#elif defined LINUX
    commandToRun = "xterm -hold -e " + connectCommand(robot->getBestIP(context()));
#elif defined MACOS
    commandToRun = std::string(File::getBHDir()) + "/Make/macOS/loginFromBush " + robot->getBestIP(context());
#endif // WINDOWS
  }
  else
  {
    QString naoVersion = fromString(RobotConfigDorsh::getName(robot->naoVersion));

    if (naoVersion == "V6"){
      if ( command == " bhumand start" ) {
        command = " systemctl --user start bhumand";
      }
      else if ( command == " bhumand stop" )
      {
        command = " systemctl --user stop bhumand";
      }
      else if ( command == " sudo /etc/init.d/naoqi start" )
      {
        command = " systemctl --user start ndevild";
      }
      else if ( command == " sudo /etc/init.d/naoqi stop" )
      {
        command = " systemctl --user stop ndevild";
      }
    }
    commandToRun = remoteCommandForQProcess(command, robot->getBestIP(context()));
  }

  ProcessRunner r(context(), commandToRun);
  r.run();

  if(r.error())
  {
    context().errorLine(robot->name + ": ssh command" + command + " failed.");
    return false;
  }
  return true;
}
