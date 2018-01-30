/*
 * DownloadLogsCmd.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#include "DownloadCameraCalibrationsCmd.h"
#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/models/Robot.h"

DownloadCameraCalibrationsCmd DownloadCameraCalibrationsCmd::theDownloadLogsCmd;

DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DownloadCameraCalibrationsCmd::getName() const
{
  return "downloadCameraCalibrations";
}

std::string DownloadCameraCalibrationsCmd::getDescription() const
{
  return "Downloads cameraCalibrations.cfg from the robot.";
}

bool DownloadCameraCalibrationsCmd::preExecution(Context & context, const std::vector<std::string> & params)
{
  if(params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }
  return true;
}

Task *DownloadCameraCalibrationsCmd::perRobotExecution(Context & context, Robot & robot)
{
  return new DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask(context, &robot);
}

bool DownloadCameraCalibrationsCmd::postExecution(Context & context, const std::vector<std::string> & params)
{
  return true;
}

bool DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::execute()
{
  QString command = getCommand();
  QStringList args = QStringList();

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

DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::DownloadCameraCalibrationsTask(Context &context,
    Robot *robot)
: RobotTask(context, robot)
{}

QString DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::getCommand()
{
#ifdef WINDOWS
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadCameraCalibrations.cmd");
#else
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadCameraCalibrations");
#endif
}
