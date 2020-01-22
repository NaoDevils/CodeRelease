#include <iostream>
#include <QString>
#include <QStringList>
#include <QInputDialog>
#include <QDir>
#include "Platform/File.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/cmds/AddRobotCmd.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/models/Robot.h"
#include "Utils/dorsh/Session.h"
#include "Utils/dorsh/tools/Filesystem.h"

#include <fstream>
#include <sstream>
#include <iomanip>

AddRobotCmd AddRobotCmd::theAddRobotCmd;

AddRobotCmd::AddRobotCmd()
{
  Commands::getInstance().addCommand(this);
}

bool AddRobotCmd::execute(Context& context, const std::vector<std::string>& params)
{
  QString command = AddRobotCmd::getCommand();

  QStringList args = QStringList();

  if (params.size() != 2)
  {
    context.errorLine("addRobot [IP] [NAME]");
    context.errorLine("Add Robot failed!");
    return false;
  }

  args.push_back(QString("--ip"));
  if (!validateAddress(params[0]))
  {
    context.errorLine(params[0] + ": is not a valid IP address");
    context.errorLine("Add Robot failed!");
    return false;
  }
  args.push_back(QString(params[0].c_str()));

  args.push_back(QString("--add"));
  args.push_back(QString(params[1].c_str()));
  
  ProcessRunner r(context, command, args);
  r.run();
  if (r.error())
  {
    context.errorLine("Add Robot failed!");
    return false;
  }
  else
  {
    context.printLine("Success: Please restart Dorsh to make changes visible!");
    return true;
  }
}

bool AddRobotCmd::validateAddress(std::string ip)
{
  std::vector<std::string> list = split(ip, '.');

  if (list.size() != 4)
    return false;

  for (std::string octet : list)
  {
    try
    {
      if (std::stoi(octet) > 255 || std::stoi(octet) < 0)
        return false;
    }
    catch (const std::invalid_argument&)
    {
      return false;
    }
  }

  return true;
}

std::string AddRobotCmd::getName() const
{
  return "addRobot";
}

std::string AddRobotCmd::getDescription() const
{
  return "[IP] [NAME]\nAdd a new robot (V6 only)";
}

std::vector<std::string> AddRobotCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return commandWithArgs;
  else
    return commandWithArgs;
}

#ifdef WINDOWS
QString AddRobotCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/installRobot.cmd");
}
#else
QString AddRobotCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/installRobot");
}
#endif
