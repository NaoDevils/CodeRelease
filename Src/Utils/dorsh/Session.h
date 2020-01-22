#pragma once

#include <map>
#include <string>
#include <vector>
#include <QObject>
#include <Utils/dorsh/models/Team.h>

class Context;
class DataAgent;
struct RobotConfigDorsh;
class IConsole;
class CommandContext;

enum ENetwork
{
  WLAN = 0,
  LAN,
  ENetworkSize,
  NONE // This is invalid (do not create an entry in the maps for it)
};

enum LogLevel
{
  ALL = 0,
  TRACE = 0,
  WARN = 1,
  CRITICAL = 2,
  FATAL = 3,
  OFF = 4
};

class Session : public QObject
{
  Q_OBJECT

  friend class Initializer;

  IConsole* console;
  LogLevel logLevel;

  DataAgent* dataAgent;

  Session();

  ENetwork getBestNetwork(const RobotConfigDorsh* robot);

public:
  unsigned short* teamNumber;
  std::map<std::string, RobotConfigDorsh*> robotsByName;

  static Session& getInstance();

  void registerConsole(IConsole* console);
  IConsole* getConsole();

  void log(LogLevel logLevel, const std::string& message);

  std::string getBestIP(const Context& context, const RobotConfigDorsh* robot);
  void setTeamNumber(Team* team);
  bool isReachable(const Context& context, const RobotConfigDorsh* robot);

  void registerDataListener(QObject* qObject, RobotConfigDorsh* robot = nullptr);
  void removeDataListener(QObject* qObject, RobotConfigDorsh* robot = nullptr);

  void registerWifiListener(QObject* qObject, RobotConfigDorsh* robot = nullptr);
  void removeWifiListener(QObject* qObject, RobotConfigDorsh* robot = nullptr);

  std::vector<std::string> sendDebugRequest(const Context& context, const RobotConfigDorsh* robot, const std::string& command);

signals:
  void robotsChanged();
};
