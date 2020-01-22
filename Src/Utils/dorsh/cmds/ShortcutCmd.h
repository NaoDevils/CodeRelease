#pragma once

#include "Utils/dorsh/cmdlib/CommandAdapter.h"

class ShortcutCmd;

class ShortcutCmd : public CommandAdapter
{
  ShortcutCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
public:
  static ShortcutCmd theShortcutCmd;
};
