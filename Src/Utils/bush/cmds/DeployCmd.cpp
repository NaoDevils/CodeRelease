#include <iostream>
#include "Platform/File.h"
#include <QString>
#include <QStringList>
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/DeployCmd.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"

#include <fstream>

DeployCmd DeployCmd::theDeployCmd;

DeployCmd::DeployTask::DeployTask(Context &context, const QString& buildConfig, Team* team, Robot *robot)
: RobotTask(context, robot),
  buildConfig(buildConfig),
  team(team)
{}

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
  args.push_back(QString("-nc"));
  args.push_back(QString("-nr"));
  args.push_back(buildConfig);
  args.push_back(fromString(robot->getBestIP(context())));
  args.push_back(QString("-r"));
  args.push_back(QString("-n"));
  args.push_back(QString::number(team->number));
  args.push_back(QString("-o"));
  args.push_back(QString::number(team->port));
  args.push_back(QString("-t"));
  args.push_back(team->colorOwn.c_str());
  args.push_back(QString("-own"));
  args.push_back(mapColorToConf(team->colorOwn).c_str());
  args.push_back(QString("-opp"));
  args.push_back(mapColorToConf(team->colorOpp).c_str());
  args.push_back(QString("-p"));
  args.push_back(QString::number(team->getPlayerNumber(*robot)));
  args.push_back(QString("-l"));
  args.push_back(team->location.c_str());
  args.push_back(QString("-g"));
  args.push_back(team->gameMode.c_str());
  args.push_back(QString("-w"));
  args.push_back(team->wlanConfig.c_str());
  args.push_back(QString("-v"));
  args.push_back(QString::number(team->volume));
  args.push_back(QString("-mv"));
  args.push_back(QString::number(team->micVolume));
  args.push_back(QString("-sp"));
  args.push_back(QString("TargetCoM"));
  args.push_back(QString((team->walkConfig == "LIPM") ? "ZMPIPController2012Module" : "FLIPMController"));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapData"));
  args.push_back(QString((team->mocapConfig) ? "MocapDataProvider" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapRobotPose"));
  args.push_back(QString((team->mocapConfig) ? "MocapRobotPoseProvider" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapBallModel"));
  args.push_back(QString((team->mocapConfig) ? "MocapRobotPoseProvider" : "default"));

  ProcessRunner r(context(), command, args);
  r.run();
  if(r.error())
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

std::string DeployCmd::mapColorToConf(std::string color)
{
  if (color == "black")
    return "x = 50; y = 128; z = 128;";
  else if (color == "yellow")
    return "x = 128; y = 80; z = 150;";
  else if (color == "red")
    return "x = 128; y = 150; z = 150;";
  else if (color == "blue")
    return "x = 128; y = 150; z = 80;";
  else if (color == "green")
    return "x = 128; y = 100; z = 100;";
  else // if (color == "gray")
    return "x = 150; y = 128; z = 128;";
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

  if(commandWithArgs.size() == 1)
    return getBuildConfigs();
  else
    return getBuildConfigs(commandWithArgs[1]);
}

bool DeployCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  team = context.getSelectedTeam();
  if(!team)
  {
    context.errorLine("No team selected!");
    return false;
  }

  buildConfig = fromString(team->buildConfig);
  if(params.size() > 0)
    buildConfig = fromString(params[0]);

  // compile and deploy if compiling was successful
  return context.execute("compile " + toString(buildConfig));
}

Task* DeployCmd::perRobotExecution(Context &context, Robot &robot)
{
  return new DeployTask(context, buildConfig, team, &robot);
}

#ifdef WINDOWS
QString DeployCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles.cmd");
}
#else
QString DeployCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles");
}
#endif
