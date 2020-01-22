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

DeployCmd::DeployTask::DeployTask(Context &context, const QString& buildConfig, Team* team, RobotConfigDorsh *robot)
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
  //args.push_back(QString("-nr")); reachability check on every deploy
  args.push_back(buildConfig);
  args.push_back(fromString(robot->getBestIP(context())));
  args.push_back(QString("-r"));
  args.push_back(QString("-n"));
  args.push_back(QString::number(team->number));
  args.push_back(QString("-o"));
  args.push_back(QString::number(team->port));
  //args.push_back(QString("-t"));
  //args.push_back(team->colorOwn.c_str());
  args.push_back(QString("-own"));
  //args.push_back(mapColorToConf(team->colorOwn).c_str());
  args.push_back(mapColorToConf(team->colorOwn).c_str());
  args.push_back(QString("-opp"));
  //args.push_back(mapColorToConf(team->colorOpp).c_str());
  args.push_back(mapColorToConf(team->colorOpp).c_str());
  args.push_back(QString("-p"));
  args.push_back(QString::number(team->getPlayerNumber(*robot)));
  args.push_back(QString("-l"));
  args.push_back(team->location.c_str());
  args.push_back(QString("-g"));
  args.push_back(team->gameMode.c_str());
  args.push_back(QString("-w"));

  QString wifi = team->wlanConfig.c_str();
  bool fiveGConfigExists = false;
  bool configExists = false;

  std::vector<std::string> configs = Filesystem::getWlanConfigs();
  for (size_t i = 0; i < configs.size(); ++i) 
  {
    QString tmpConfig = fromString(configs[i]);
    if (tmpConfig.contains(wifi + "_5GHz")) 
    {
      fiveGConfigExists = true;
    }
    if (tmpConfig.startsWith(wifi) && tmpConfig.endsWith(wifi))
    {
      configExists = true;
    }
  }

  if (team->wlanFrequency == "auto")
  {
    if (robot->naoVersion == RobotConfig::V6)
    {
      if (fiveGConfigExists)
      {
        args.push_back(wifi + "_5GHz");
      }
      else
      {
        args.push_back(wifi);
      }
    }
    else
    {
      if (configExists)
      {
        args.push_back(wifi);
      }
      else
      {
        context().errorLine("2.4GHz config of network " + team->wlanConfig + " does not exist!");
        return false;
      }
    }
  }
  else if (team->wlanFrequency == "2.4 GHz")
  {
    if (configExists)
    {
      args.push_back(wifi);
    }
    else
    {
      context().errorLine("2.4GHz config of network " + team->wlanConfig + " does not exist!");
      return false;
    }
  }
  else
  {
    if (fiveGConfigExists)
    {
      args.push_back(wifi + "_5GHz");
    }
    else
    {
      context().errorLine("5GHz config of network " + team->wlanConfig + " does not exist!");
      return false;
    }
  }
  args.push_back(QString("-v"));
  args.push_back(QString::number(team->volume));
  args.push_back(QString("-mv"));
  args.push_back(QString::number(team->micVolume));
  args.push_back(QString("-log"));
  args.push_back(QString(team->logConfig.c_str()));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapData"));
  args.push_back(QString((team->mocapConfig) ? "MocapDataProvider" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapRobotPose"));
  args.push_back(QString((team->mocapConfig) ? "MocapRobotPoseProvider" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("MocapBallModel"));
  args.push_back(QString((team->mocapConfig) ? "MocapRobotPoseProvider" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("CameraSettings"));
  args.push_back(QString((robot->naoVersion == RobotConfig::V6) ? "default" : "CameraProvider"));
  args.push_back(QString("-sp"));
  args.push_back(QString("CameraSettingsUpper"));
  args.push_back(QString((robot->naoVersion == RobotConfig::V6) ? "default" : "CameraProvider"));
  args.push_back(QString("-sp"));
  args.push_back(QString("CameraSettingsV6"));
  args.push_back(QString((robot->naoVersion == RobotConfig::V6) ? "CameraProviderV6" : "default"));
  args.push_back(QString("-sp"));
  args.push_back(QString("CameraSettingsUpperV6"));
  args.push_back(QString((robot->naoVersion == RobotConfig::V6) ? "CameraProviderV6" : "default"));
  args.push_back(QString("-nv"));
  args.push_back(fromString(RobotConfigDorsh::getName(robot->naoVersion)));

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

/*std::string DeployCmd::mapColorToConf(std::string color)
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
}*/

std::string DeployCmd::mapColorToConf(int color)
{
  unsigned char y;
  unsigned char cb;
  unsigned char cr;

  unsigned char r = static_cast<unsigned char>(color / (256 * 256));
  unsigned char g = static_cast<unsigned char>(color % (256 * 256) / 256);
  unsigned char b = static_cast<unsigned char>(color % 256);

  ColorModelConversions::fromRGBToYCbCr(r, g, b, y, cb, cr);

  std::stringstream config;
  config << "y = " << static_cast<int>(y)
    << "; cb = " << static_cast<int>(cb)
    << "; cr = " << static_cast<int>(cr) << ";" ;
  std::cout << config.str() << "\n";
  std::cout << "rgb( " <<  color << ")"
    << static_cast<unsigned>(r) << " "
    << static_cast<unsigned>(g) << " "
    << static_cast<unsigned>(b) << " " << "\n";
  return config.str();
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

Task* DeployCmd::perRobotExecution(Context &context, RobotConfigDorsh &robot)
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
