#pragma once

#include "Utils/dorsh/cmdlib/RobotCommand.h"

class ShowCmd : public RobotCommand
{
  std::vector<std::string> files;

  ShowCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, RobotConfigDorsh &robot);

  static ShowCmd theShowCmd;
};
