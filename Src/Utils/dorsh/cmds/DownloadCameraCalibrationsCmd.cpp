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
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/Platform.h"
#include "Session.h"
#include "models/Team.h"
#include "models/Robot.h"

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

bool DownloadCameraCalibrationsCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if (params.size() > 0)
  {
    context.printLine("This command does not have any parameters!");
    return false;
  }
  return true;
}

Task* DownloadCameraCalibrationsCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask(context, &robot);
}

bool DownloadCameraCalibrationsCmd::postExecution(Context& context, const std::vector<std::string>& params)
{
  return true;
}

bool DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::execute()
{
  QString command = getCommand();
  QStringList args = QStringList();

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

DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::DownloadCameraCalibrationsTask(Context& context, RobotConfigDorsh* robot) : RobotTask(context, robot) {}

QString DownloadCameraCalibrationsCmd::DownloadCameraCalibrationsTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadCameraCalibrations.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/downloadCameraCalibrations");
#endif
}
