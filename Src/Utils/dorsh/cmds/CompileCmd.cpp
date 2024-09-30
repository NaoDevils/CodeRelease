#include "cmds/CompileCmd.h"
#include "cmdlib/Context.h"
#include "tools/Filesystem.h"
#include "tools/StringTools.h"
#include "models/Team.h"
#include "Platform/File.h"
#include "tools/Platform.h"
#include "Tools/Build.h"

CompileCmd CompileCmd::theCompileCmd;

CompileCmd::CompileCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string CompileCmd::getName() const
{
  return "compile";
}

std::string CompileCmd::getDescription() const
{
  return "[ <config> [ <project> ] ]\nCompiles a project with a specified build configuration. [Default: Develop and Nao]";
}

std::vector<std::string> CompileCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return getBuildConfigs();
  else if (commandWithArgs.size() == 2 && *--cmdLine.end() != ' ')
    return getBuildConfigs(commandWithArgs[1]);
  else
    return {};
}

CompileCmd::CompileTask::CompileTask(Context& context, const std::string& label, const QString& command, const QStringList& args)
    : Task(context), r(context, command, args), label(label)
{
}

bool CompileCmd::CompileTask::execute()
{
  r.run();
  if (context().isCanceled())
  {
    context().cleanupFinished();
    return true;
  }
  bool status = true;
  if (r.error())
  {
    context().errorLine("Failed to compile.");
    status = false;
  }
  return status;
}

void CompileCmd::CompileTask::cancel()
{
  r.stop();
}

void CompileCmd::CompileTask::setContext(Context* context)
{
  r.setContext(*context);
  Task::setContext(context);
}

std::string CompileCmd::CompileTask::getLabel()
{
  return label;
}

bool CompileCmd::execute(Context& context, const std::vector<std::string>& params)
{
  QString command;
  QStringList args;

  if (params.size() > 2)
  {
    context.errorLine("Too many parameters specified.");
    return false;
  }
  else if (params.empty())
  {
    Team* team = context.getSelectedTeam();
    if (team && team->buildConfig.length() > 0)
    {
      command = getCommand("Nao");
      args = getParams(QString::fromStdString(team->buildConfig), "Nao");
    }
    else
    {
      command = getCommand("Nao");
      args = getParams("Develop", "Nao");
    }
  }
  else if (params.size() == 1)
  {
    command = getCommand("Nao");
    args = getParams(QString::fromStdString(params[0]), "Nao");
  }
  else
  {
    command = getCommand(QString::fromStdString(params[1]));
    args = getParams(QString::fromStdString(params[0]), QString::fromStdString(params[1]));
  }

  context.executeDetached(new CompileTask(context, "Build", command, args));
  return context.waitForChildren();
}

QString CompileCmd::getCommand(const QString& project)
{
  if constexpr (Build::platformWindows())
  {
    if (project == "Nao")
      return "bash";
    else
      return QString(File::getBHDir()) + "/Make/Windows/compile.bat";
  }
  else
  {
    return QString(File::getBHDir()) + "/Make/Linux/compile.sh";
  }
}

QStringList CompileCmd::getParams(const QString& config, const QString& project)
{
  QStringList args;

  if (project == "Nao")
  {
    if constexpr (Build::platformWindows())
      args << "-l" << wslpath(QString(File::getBHDir()) + "/Make/Linux/compile.sh");

    args << "nao-" + config.toLower() << "Nao";
  }
  else
  {
    const bool isMultiConfig = Filesystem::isMultiConfig();
    args << (isMultiConfig ? "simulator-multiconfig-" : "simulator-") + config.toLower() << "SimRobot";

    // Compile using the same VS version that Dorsh was compiled with.
#ifdef _MSC_VER
#if (_MSC_VER < 1930)
    args << "16";
#else
    args << "17";
#endif
#endif
  }

  return args;
}

QString CompileCmd::wslpath(const QString& path)
{
  QString tempPath = "";
  for (int i = 0; i < path.length(); i++)
  {
    if (i == 0)
    {
      tempPath = QString("/mnt/") + path.at(i);
      continue;
    }
    if (i == 1)
      continue;
    tempPath = tempPath + path.at(i);
  }

  return tempPath;
}
