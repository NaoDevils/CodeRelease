#pragma once

#include "cmdlib/CommandAdapter.h"

class SimCmd : public CommandAdapter
{
  SimCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
  std::string getSimulatorExecutable(std::string buildConfig);

public:
  static SimCmd theSimCmd;
};
