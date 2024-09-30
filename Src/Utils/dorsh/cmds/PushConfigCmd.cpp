#include <iostream>
#include <QString>
#include <QStringList>
#include "Platform/File.h"
#include "cmdlib/Context.h"
#include "cmdlib/Commands.h"
#include "cmdlib/ProcessRunner.h"
#include "cmds/PushConfigCmd.h"
#include "tools/StringTools.h"
#include "tools/Platform.h"
#include "models/Team.h"
#include "tools/Filesystem.h"
#include "Tools/ColorModelConversions.h"

#include <fstream>
#include <sstream>
#include <iomanip>

PushConfigCmd PushConfigCmd::thePushConfigCmd;

PushConfigCmd::PushConfigTask::PushConfigTask(Context& context, const QString& buildConfig, Team* team, RobotConfigDorsh* robot)
    : RobotTask(context, robot), buildConfig(buildConfig), team(team)
{
}

bool PushConfigCmd::PushConfigTask::execute()
{
  QString command = PushConfigCmd::getCommand();

  QStringList args = QStringList();
  args.push_back(QString("-d")); //Arne 27.04.17 - delete Logs on every deploy
  //args.push_back(QString("-nr")); reachability check on every deploy
  args.push_back(QString("-sn"));
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
    context().errorLine("PushConfig of \"" + robot->name + "\" failed!");
    return false;
  }
  else
  {
    context().printLine("Success! (" + robot->name + ")");
    return true;
  }
}

int PushConfigCmd::PushConfigTask::mapColorToConf(int color)
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

bool PushConfigCmd::preExecution(Context& context, const std::vector<std::string>& params)
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

  return true;
}

PushConfigCmd::PushConfigCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string PushConfigCmd::getName() const
{
  return "pushConfig";
}

std::string PushConfigCmd::getDescription() const
{
  return "Push config files to selected robots. (Uses the copyfiles script)";
}

std::vector<std::string> PushConfigCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  if (commandWithArgs.size() == 1)
    return getBuildConfigs();
  else
    return getBuildConfigs(commandWithArgs[1]);
}

Task* PushConfigCmd::perRobotExecution(Context& context, RobotConfigDorsh& robot)
{
  return new PushConfigTask(context, buildConfig, team, &robot);
}

#ifdef WINDOWS
QString PushConfigCmd::getCommand()
{
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/copyfiles.cmd");
}
#else
QString PushConfigCmd::getCommand()
{
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + platformDirectory() + "/copyfiles");
}
#endif
