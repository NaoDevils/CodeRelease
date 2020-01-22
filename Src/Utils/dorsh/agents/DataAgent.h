#pragma once

#include <QProcess>
#include <QTimer>
#include <QUdpSocket>

#include <map>
#include <string>
#include "Tools/Network/UdpComm.h"

#include "Utils/dorsh/models/Power.h"
#include "Utils/dorsh/Session.h"

struct RobotConfigDorsh;

class DataAgent : public QObject
{
  Q_OBJECT
  std::map<std::string, std::string> receivedMap;
  std::map<std::string, int> timeOfLastUpdate;
  std::map<std::string, int> timeOfLastUpdateWifi;
  QUdpSocket udpSocket;
  QUdpSocket wifiSocket;
  QTimer timer;
  unsigned short teamNumber = 12;

public:
  DataAgent();
  ~DataAgent();
  void initialize(std::map<std::string, RobotConfigDorsh*>& robotsByName);
  void reset(RobotConfigDorsh* robot);
  void updateRobot();
  bool isLANReachable(const RobotConfigDorsh* robot);
  bool isWLANReachable(const RobotConfigDorsh* robot);

private slots:
  std::map<std::string, std::string> stringToMap(char mapCharArray[]);
  void updateData();
  void updateWifiConnection();
  void teamObserver();

signals:
  void newSensorData(std::map<std::string, std::string>);
  void wifiConnection(std::string, bool);
};
