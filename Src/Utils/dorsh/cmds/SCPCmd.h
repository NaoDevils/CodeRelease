#pragma once

#include "Utils/dorsh/cmdlib/RobotCommand.h"

class SCPCmd : public RobotCommand
{
  class SCPTask : public RobotTask
  {
    std::string command;
  public:
    SCPTask(Context &context, RobotConfigDorsh *robot, const std::string &command);
    bool execute();
  };

  bool fromRobot;
  std::string fromFile;
  std::string toFile;

  SCPCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, RobotConfigDorsh &robot);
public:
  static SCPCmd theSCPCmd;
};
