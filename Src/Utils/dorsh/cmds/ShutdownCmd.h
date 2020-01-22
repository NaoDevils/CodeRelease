#pragma once

#include "Utils/dorsh/cmdlib/RobotCommand.h"

class ShutdownCmd : public RobotCommand
{
  class ShutdownTask : public RobotTask
  {
  public:
    ShutdownTask(Context &context, RobotConfigDorsh *robot);
    bool execute();
  };

  ShutdownCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, RobotConfigDorsh &robot);
public:
  static ShutdownCmd theShutdownCmd;
};
