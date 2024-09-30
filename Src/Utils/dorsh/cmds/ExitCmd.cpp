#include "cmds/ExitCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"

ExitCmd ExitCmd::theExitCmd;

ExitCmd::ExitCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ExitCmd::getName() const
{
  return "exit";
}

std::string ExitCmd::getDescription() const
{
  return "Exit dorsh.";
}

bool ExitCmd::execute(Context& context, const std::vector<std::string>& params)
{
  context.shutdown();
  return true;
}
