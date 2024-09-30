#pragma once

#include "cmdlib/Command.h"

class CommandAdapter : public Command
{
public:
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
};
