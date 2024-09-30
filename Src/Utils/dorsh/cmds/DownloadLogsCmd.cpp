/*
 * DownloadLogsCmd.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "DownloadLogsCmd.h"
#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include "Session.h"
#include "models/Team.h"
#include "models/Robot.h"

DownloadLogsCmd DownloadLogsCmd::theDownloadLogsCmd;

DownloadLogsCmd::DownloadLogsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DownloadLogsCmd::getName() const
{
  return "downloadLogs";
}

std::string DownloadLogsCmd::getDescription() const
{
  return "Downloads all *.log files from the robot. Afterwards the logs are deleted from the robot.";
}

bool DownloadLogsCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if (params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }
  return true;
}

Task* DownloadLogsCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new DownloadLogsCmd::DownloadLogsTask(context, &robot);
}

bool DownloadLogsCmd::postExecution(Context& context, const std::vector<std::string>& params)
{
  return true;
}

bool DownloadLogsCmd::DownloadLogsTask::execute()
{
  QString command = getCommand();
  QStringList args = QStringList();
  args.push_back("-d"); //delete files after download.

  args.push_back(QString::fromStdString(robot->name));
  args.push_back(QString::fromStdString(robot->getBestIP(context())));

  ProcessRunner r(context(), command, args);
  r.run();

  if (r.error())
  {
    context().errorLine("Download failed!");
  }
  else
  {
    context().printLine("Success! (" + robot->name + ")");
  }

  return true;
}

DownloadLogsCmd::DownloadLogsTask::DownloadLogsTask(Context& context, RobotConfigDorsh* robot) : RobotTask(context, robot) {}

QString DownloadLogsCmd::DownloadLogsTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadLogs.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadLogs");
#endif
}
