#pragma once

#include "Utils/dorsh/cmdlib/CommandAdapter.h"

class QString;
class AddRobotCmd;

class AddRobotCmd : public CommandAdapter
{
  AddRobotCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;

  static QString getCommand();
public:
  static AddRobotCmd theAddRobotCmd;
private:
  bool validateAddress(std::string ip);
};
