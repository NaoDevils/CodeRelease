#include "cmds/ShortcutCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"

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
