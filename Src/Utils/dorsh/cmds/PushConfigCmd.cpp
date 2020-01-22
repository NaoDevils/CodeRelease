#include <iostream>
#include <QString>
#include <QStringList>
#include "Platform/File.h"
#include "Utils/dorsh/cmdlib/Context.h"
#include "Utils/dorsh/cmdlib/Commands.h"
#include "Utils/dorsh/cmdlib/ProcessRunner.h"
#include "Utils/dorsh/cmds/PushConfigCmd.h"
#include "Utils/dorsh/tools/StringTools.h"
#include "Utils/dorsh/tools/Platform.h"
#include "Utils/dorsh/models/Team.h"
#include "Utils/dorsh/tools/Filesystem.h"
#include "Tools/ColorModelConversions.h"

#include <fstream>
#include <sstream>
#include <iomanip>

PushConfigCmd PushConfigCmd::thePushConfigCmd;

PushConfigCmd::PushConfigTask::PushConfigTask(Context &context, const QString& buildConfig, Team* team, RobotConfigDorsh *robot)
  : RobotTask(context, robot),
  buildConfig(buildConfig),
  team(team)
{}

bool PushConfigCmd::PushConfigTask::execute()
{
  QString command = PushConfigCmd::getCommand();

  QStringList args = QStringList();
  args.push_back(QString("-d")); //Arne 27.04.17 - delete Logs on every deploy
  args.push_back(QString("-nc"));
  //args.push_back(QString("-nr")); reachability check on every deploy
  args.push_back(QString("-sn"));
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

std::string PushConfigCmd::mapColorToConf(int color)
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
    << "; cr = " << static_cast<int>(cr) << ";";
  std::cout << config.str() << "\n";
  std::cout << "rgb( " << color << ")"
    << static_cast<unsigned>(r) << " "
    << static_cast<unsigned>(g) << " "
    << static_cast<unsigned>(b) << " " << "\n";
  return config.str();
}

bool PushConfigCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  team = context.getSelectedTeam();
  if (!team)
  {
    context.errorLine("No team selected!");
    return false;
  }

  buildConfig = fromString(team->buildConfig);
  if (params.size() > 0)
    buildConfig = fromString(params[0]);

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

Task* PushConfigCmd::perRobotExecution(Context &context, RobotConfigDorsh &robot)
{
  return new PushConfigTask(context, buildConfig, team, &robot);
}

#ifdef WINDOWS
QString PushConfigCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles.cmd");
}
#else
QString PushConfigCmd::getCommand()
{
  return fromString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/copyfiles");
}
#endif
