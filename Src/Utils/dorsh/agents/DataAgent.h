#pragma once

#include <QProcess>
#include <QTimer>
#include <QUdpSocket>

#include <map>
#include <string>
#include <memory>
#include "Tools/Network/UdpComm.h"

#include "Utils/dorsh/models/Power.h"
#include "Utils/dorsh/Session.h"
#include <nlohmann/json_fwd.hpp>

#include <QFloat16> // see https://github.com/nlohmann/json/issues/2650

struct RobotConfigDorsh;

class DataAgent : public QObject
{
  Q_OBJECT
  std::map<std::string, nlohmann::json> lastData;
  QUdpSocket dataSocket;
  QUdpSocket tcSocket;
  QUdpSocket gcSocket;
  QTimer timer;
  unsigned short teamNumber = 12;

  static constexpr std::string_view keepAliveData = "sendData";
  static constexpr unsigned updateTime = 5000;
  static constexpr unsigned dataTimeout = 10000;
  static constexpr unsigned gcTimeout = 60000;
  unsigned lastGCMessage = 0;

public:
  DataAgent();
  ~DataAgent();
  bool isLANReachable(const RobotConfigDorsh* robot);
  bool isWLANReachable(const RobotConfigDorsh* robot);

private slots:
  void updateData();
  void updateTC();
  void updateGC();
  void sendKeepAlive();

signals:
  void newData(const nlohmann::json&);
  void updateGCStatus(bool);
};
