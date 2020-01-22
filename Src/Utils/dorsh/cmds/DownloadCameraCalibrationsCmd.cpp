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
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/Session.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/models/Robot.h"

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

Task *DownloadCameraCalibrationsCmd::perRobotExecution(Context & context, RobotConfigDorsh & robot)
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
    RobotConfigDorsh *robot)
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
