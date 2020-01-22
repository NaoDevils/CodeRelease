#include "Utils/dorsh/cmds/ShortcutCmd.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/tools/StringTools.h"

ShortcutCmd ShortcutCmd::theShortcutCmd;

ShortcutCmd::ShortcutCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ShortcutCmd::getName() const
{
  return "shortcuts";
}

std::string ShortcutCmd::getDescription() const
{
  std::string description = "Ctrl + S: save \n";
  description += "Ctrl + R: reload team \n";
  description += "Ctrl + PageUp: select previous team (only available for multiple team tabs)\n";
  description += "Ctrl + PageDown: select next team (only available for multiple team tabs)\n";
  description += "Ctrl + U: select/unselect upper robot row \n";
  description += "Ctrl + L: select/unselect lower robot row";
  return description;
}

bool ShortcutCmd::execute(Context& context, const std::vector<std::string>& params)
{
  return true;
}
