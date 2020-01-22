#pragma once

#include "Utils/dorsh/cmdlib/RobotCommand.h"

class QString;

class DeployCmd : public RobotCommand
{
  class DeployTask : public RobotTask
  {
    QString buildConfig;
    Team* team;

  public:
    DeployTask(Context &context, const QString& buildConfig, Team* team, RobotConfigDorsh* robot);
    virtual bool execute();
  };

  Team* team;
  QString buildConfig;

  DeployCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, RobotConfigDorsh &robot);

  static QString getCommand();
  public:
    static DeployCmd theDeployCmd;
  private:
    //static std::string mapColorToConf(std::string color);
    static std::string mapColorToConf(int color);
};
