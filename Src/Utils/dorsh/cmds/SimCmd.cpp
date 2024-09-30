#include <cstdlib>
#include <algorithm>

#include "Platform/File.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/SimCmd.h"
#include "models/Team.h"
#include "tools/Filesystem.h"
#include "tools/Platform.h"
#include "Tools/Build.h"

SimCmd SimCmd::theSimCmd;

SimCmd::SimCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string SimCmd::getName() const
{
  return "sim";
}

std::string SimCmd::getDescription() const
{
  return "Connects to robots via simulator. Requires a Simulator built with Develop configuration.";
}

bool SimCmd::execute(Context& context, const std::vector<std::string>& params)
{
  //const std::string buildConfig = "Develop";
  const std::string buildConfig = context.getSelectedTeam()->buildConfig;
  const std::string simulatorExecutable = getSimulatorExecutable(buildConfig);
  const std::string remoteRobotScene = std::string(File::getBHDir()) + "/Config/Scenes/RemoteRobot.ros2";

  File simFile(simulatorExecutable, "r");
  if (!simFile.exists())
  {
    bool compileStatus = Commands::getInstance().execute(&context, "compile " + buildConfig + " SimRobot");
    if (!compileStatus)
      return false;
  }
  ProcessRunner r(context, simulatorExecutable, {QString::fromStdString(remoteRobotScene)});
  r.run();
  if (r.error())
  {
    context.errorLine("Failed.");
    return false;
  }
  return true;
}

std::string SimCmd::getSimulatorExecutable(std::string buildConfig)
{
  std::string simulatorExecutable = std::string(File::getBHDir()) + "/Build";
  if (Filesystem::isMultiConfig())
  {
    if (buildConfig == "Develop")
      buildConfig = "RelWithDebInfo";

    simulatorExecutable += "/simulator-multiconfig/" + buildConfig + "/";
  }
  else
  {
    std::transform(buildConfig.begin(),
        buildConfig.end(),
        buildConfig.begin(),
        [](unsigned char c)
        {
          return std::tolower(c);
        });

    simulatorExecutable += "/simulator-" + buildConfig + "/";
  }

  simulatorExecutable += Build::platformWindows() ? "SimRobot.exe" : Build::platformMacOS() ? ".app/Contents/MacOS/SimRobot" : "SimRobot";

  return simulatorExecutable;
}
