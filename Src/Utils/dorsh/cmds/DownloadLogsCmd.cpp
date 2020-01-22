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
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/Session.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/models/Robot.h"

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

bool DownloadLogsCmd::preExecution(Context & context, const std::vector<std::string> & params)
{
  if(params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }
  return true;
}

Task *DownloadLogsCmd::perRobotExecution(Context & context, RobotConfigDorsh & robot)
{
  return new DownloadLogsCmd::DownloadLogsTask(context, &robot);
}

bool DownloadLogsCmd::postExecution(Context & context, const std::vector<std::string> & params)
{
  return true;
}

bool DownloadLogsCmd::DownloadLogsTask::execute()
{
  QString command = getCommand();
  QStringList args = QStringList();
  args.push_back("-d"); //delete files after download.

  args.push_back(fromString(robot->name));
  args.push_back(fromString(robot->getBestIP(context())));

  ProcessRunner r(context(), command, args);
  r.run();

  if(r.error())
  {
    context().errorLine("Download failed!");
  }
  else
  {
    context().printLine("Success! (" + robot->name + ")");
  }

  return true;
}

DownloadLogsCmd::DownloadLogsTask::DownloadLogsTask(Context &context,
    RobotConfigDorsh *robot)
: RobotTask(context, robot)
{}

QString DownloadLogsCmd::DownloadLogsTask::getCommand()
{
#ifdef WINDOWS
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs.cmd");
#else
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs");
#endif
}
