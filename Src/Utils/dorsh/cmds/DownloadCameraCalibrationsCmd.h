/*
 * DownloadLogsCmd.h
 *
 *  Created on: Mar 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#ifndef DOWNLOADCAMERACALIBRATIONSCMD_H_
#define DOWNLOADCAMERACALIBRATIONSCMD_H_

#include "Utils/dorsh/cmdlib/RobotCommand.h"
/**
 * Downloads the log files from the robot.
 */
class DownloadCameraCalibrationsCmd : public RobotCommand
{
  class DownloadCameraCalibrationsTask : public RobotTask
  {
  public:
    DownloadCameraCalibrationsTask(Context &context, RobotConfigDorsh *robot);
    bool execute();
    QString getCommand();
  };

  public:
    DownloadCameraCalibrationsCmd();
    virtual std::string getName() const;
    virtual std::string getDescription() const;
    virtual bool preExecution(Context &context, const std::vector<std::string> &params);
    virtual Task* perRobotExecution(Context &context, RobotConfigDorsh &robot);
    virtual bool postExecution(Context &context, const std::vector<std::string> &params);

    QString getCommand();
  public:
    static DownloadCameraCalibrationsCmd theDownloadLogsCmd;
};

#endif /* DOWNLOADLOGSCMD_H_ */
