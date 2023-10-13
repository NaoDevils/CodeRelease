#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QColor>
#include <QStringList>
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/cmds/DeployCmd.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/Session.h"
#include "Utils/dorsh/models/Team.h"
#include "Tools/ColorModelConversions.h"
#include "Utils/dorsh/tools/Filesystem.h"

#include <fstream>
#include <sstream>
#include <iomanip>

DeployCmd DeployCmd::theDeployCmd;

DeployCmd::DeployTask::DeployTask(Context& context, const QString& buildConfig, Team* team, RobotConfigDorsh* robot)
    : RobotTask(context, robot), buildConfig(buildConfig), team(team)
{
}

bool DeployCmd::DeployTask::execute()
{
  // Change TargetCoM in modules.cfg


  /* Since the PingAgent knows the roundtrip time for all robots, maybe we can
   * adjust the timeout of rsync to determine faster if the connection is
   * lost.
   */
  QString command = DeployCmd::getCommand();

  QStringList args = QStringList();
  args.push_back(QString("-d")); //Arne 27.04.17 - delete Logs on every deploy
  //args.push_back(QString("-nr")); reachability check on every deploy
  args.push_back(buildConfig);
  args.push_back(QString::fromStdString(robot->getBestIP(context())));
  args.push_back(QString("-r"));
  args.push_back(QString("-n"));
  args.push_back(QString::number(team->number));
  args.push_back(QString("-o"));
  args.push_back(QString::number(team->port));
  args.push_back(QString("-own"));
  args.push_back(QString::number(mapColorToConf(team->colorOwn)));
  args.push_back(QString("-opp"));
  args.push_back(QString::number(mapColorToConf(team->colorOpp)));
  args.push_back(QString("-p"));
  args.push_back(QString::number(team->getPlayerNumber(*robot)));
  for (const std::string& overlay : team->overlays)
  {
    args.push_back(QString("-eo"));
    args.push_back(overlay.c_str());
  }
  args.push_back(QString("-g"));
  args.push_back(QString::fromStdString(team->gameMode));
  args.push_back(QString("-w"));
  args.push_back(QString::fromStdString(team->wlanConfig));
  args.push_back(QString("-v"));
  args.push_back(QString::number(team->volume));
  args.push_back(QString("-mv"));
  args.push_back(QString::number(team->micVolume));
  args.push_back(QString("-nv"));
  args.push_back(QString::fromStdString(RobotConfigDorsh::getName(robot->naoVersion)));

  ProcessRunner r(context(), command, args);
  r.run();
  if (r.error())
  {
    context().errorLine("Deploy of \"" + robot->name + "\" failed!");
    return false;
  }
  else
  {
    context().printLine("Success! (" + robot->name + ")");
    return true;
  }
}

int DeployCmd::DeployTask::mapColorToConf(int color)
{
  std::map<int, int> teamColorMap{
      {65535, 0}, //cyan
      {16711680, 1}, //red
      {16776960, 2}, //yellow
      {0, 3}, //black
      {16777215, 4}, //white
      {891904, 5}, //darkgreen
      {16744960, 6}, //orange
      {14221567, 7}, //purple
      {7024663, 8}, //brown
      {8421504, 9}, //grey
  };

  int gcColor;
  if (teamColorMap.find(color) == teamColorMap.end())
  {
    context().errorLine("Selected Color is unknown! Please use only the custom colors!");
    gcColor = 2; //yellow
  }
  else
  {
    gcColor = teamColorMap[color];
  }
  return gcColor;
}

DeployCmd::DeployCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DeployCmd::getName() const
{
  return "deploy";
}

std::string DeployCmd::getDescription() const
{
  return "Deploys code to selected robots. (Uses the copyfiles script)";
}

std::vector<std::string> DeployCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return getBuildConfigs();
  else
    return getBuildConfigs(commandWithArgs[1]);
}

bool DeployCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  team = context.getSelectedTeam();
  if (!team)
  {
    context.errorLine("No team selected!");
    return false;
  }

  buildConfig = QString::fromStdString(team->buildConfig);
  if (params.size() > 0)
    buildConfig = QString::fromStdString(params[0]);

  // compile and deploy if compiling was successful
  return context.execute("compile " + buildConfig.toStdString());
}

Task* DeployCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new DeployTask(context, buildConfig, team, &robot);
}

#ifdef WINDOWS
QString DeployCmd::getCommand()
{
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/copyfiles.cmd");
}
#else
QString DeployCmd::getCommand()
{
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/copyfiles");
}
#endif
