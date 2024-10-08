#pragma once

#include "cmdlib/RobotCommand.h"

class RestartCmd;
class ProcessRunner;

enum RestartType
{
  NAODEVILS,
  NAODEVILSBASE,
  ROBOT,
  SINGLE_COMMANDS,
  NAODEVILS_AND_NAODEVILSBASE,
  COMBINED_COMMANDS,
  NO_RESTART
};

class RestartCmd : public RobotCommand
{
  class RestartTask : public RobotTask
  {
    RestartType type;
    void reportStatus(const ProcessRunner& r);

  public:
    RestartTask(Context& context, RobotConfigDorsh* robot, RestartType type);
    virtual bool execute();
  };

  RestartType type;

  RestartCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, RobotConfigDorsh& robot);

public:
  static RestartCmd theRestartCmd;
};
