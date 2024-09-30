#include <iostream>
#include <QString>
#include <QStringList>
#include <QInputDialog>
#include <QDir>
#include "Platform/File.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/AddRobotCmd.h"
#include "tools/StringTools.h"
#include "tools/Platform.h"
#include "models/Team.h"
#include "models/Robot.h"
#include "Session.h"
#include "tools/Filesystem.h"

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
QString AddRobotCmd::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/installRobot.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/installRobot");
#endif
}
