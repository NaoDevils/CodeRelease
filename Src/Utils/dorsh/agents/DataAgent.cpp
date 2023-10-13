/**
 * @file DataAgent.cpp
 * manages the SensorReader, Powerbar and Pingbar for dorsh. LAN connection is now determined by existence of SensorReader Packages and Wifi connection by existence of TeamData Packages
 */

#include <chrono>
#include <sstream>
#include <thread>
#include <QTimer>
#include <QUdpSocket>
#include <Utils/dorsh/models/Team.h>
#include "Platform/SystemCall.h"
#include "Utils/dorsh/agents/DataAgent.h"
#include "Utils/dorsh/models/Robot.h"
#include "Tools/Network/UdpComm.h"
#include "Utils/dorsh/ui/MainWindow.h"
#include <nlohmann/json.hpp>

DataAgent::DataAgent()
{
  //binds to Port and listens for new Data
  dataSocket.bind(QHostAddress::AnyIPv4, 55555, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(&dataSocket, SIGNAL(readyRead()), this, SLOT(updateData()));
  tcSocket.bind(QHostAddress::AnyIPv4, 10000 + teamNumber, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(&tcSocket, SIGNAL(readyRead()), this, SLOT(updateTC()));
  gcSocket.bind(QHostAddress::Any, 3838, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(&gcSocket, SIGNAL(readyRead()), this, SLOT(updateGC()));

  // start timer to send keep-alives
  timer.setSingleShot(false);
  connect(&timer, SIGNAL(timeout()), this, SLOT(sendKeepAlive()));
  timer.start(updateTime);
}

DataAgent::~DataAgent() {}

bool DataAgent::isLANReachable(const RobotConfigDorsh* robot)
{
  const auto data = lastData.find(robot->name);
  if (data == lastData.end())
    return false;

  if (!data->second.contains("lan"))
    return false;

  const unsigned currentTime = SystemCall::getRealSystemTime();
  return currentTime - data->second["lan"].get<unsigned>() < dataTimeout;
}

bool DataAgent::isWLANReachable(const RobotConfigDorsh* robot)
{
  const auto data = lastData.find(robot->name);
  if (data == lastData.end())
    return false;

  const unsigned currentTime = SystemCall::getRealSystemTime();
  if (data->second.contains("wlan") && currentTime - data->second["wlan"].get<unsigned>() < dataTimeout)
    return true;

  if (data->second.contains("tc") && currentTime - data->second["tc"].get<unsigned>() < dataTimeout)
    return true;

  return false;
}

void DataAgent::sendKeepAlive()
{
  const unsigned short newTeamNumber = *Session::getInstance().teamNumber;
  if (10000 + newTeamNumber != tcSocket.localPort())
  {
    tcSocket.flush();
    tcSocket.close();
    if (tcSocket.bind(QHostAddress::AnyIPv4, 10000 + teamNumber, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
      teamNumber = newTeamNumber;
  }

  if (lastGCMessage == 0 || SystemCall::getRealSystemTime() - lastGCMessage > gcTimeout)
  {
    emit updateGCStatus(false);
    dataSocket.writeDatagram(keepAliveData.data(), keepAliveData.size(), QHostAddress("10.0.255.255"), 55555);
  }
}

//updates the Map with received SensorData package
void DataAgent::updateData()
{
  if (dataSocket.hasPendingDatagrams())
  {
    QByteArray datagram;
    datagram.resize(dataSocket.pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;

    dataSocket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    // skip keep-alive packages
    if (static_cast<size_t>(datagram.size()) >= keepAliveData.size() && strncmp(datagram.data(), keepAliveData.data(), keepAliveData.size()) == 0)
      return;

    try
    {
      nlohmann::json j = nlohmann::json::from_msgpack(datagram.begin(), datagram.end());

      const unsigned currentTime = SystemCall::getRealSystemTime();

      if ((sender.toIPv4Address() & 0xFFFF0000) == 0x0A000000) // ip == 10.0.*.*
        j["wlan"] = currentTime;
      else
        j["lan"] = currentTime;

      nlohmann::json& data = lastData[j.at("name").get<std::string>()];
      data.merge_patch(j);

      emit newData(data);
    }
    catch (...)
    {
      return;
    }
  }
}

//updates the Wifi conncetion with received TeamData package
void DataAgent::updateTC()
{
  if (tcSocket.hasPendingDatagrams())
  {
    QByteArray datagram;
    datagram.resize(tcSocket.pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;

    tcSocket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    const auto sameIP = [&](const auto& elem)
    {
      const auto& [first, second] = elem;
      return second->wlan == sender.toString().toStdString();
    };

    const auto& robots = Session::getInstance().robotsByName;
    const auto robot = std::find_if(robots.begin(), robots.end(), sameIP);

    if (robot != robots.end())
    {
      auto& data = lastData[robot->second->name];
      data["name"] = robot->second->name;
      data["tc"] = SystemCall::getRealSystemTime();
      emit newData(data);
    }
  }
}

void DataAgent::updateGC()
{
  if (gcSocket.hasPendingDatagrams())
  {
    QByteArray datagram;
    datagram.resize(gcSocket.pendingDatagramSize());

    gcSocket.readDatagram(datagram.data(), datagram.size());

    static constexpr std::string_view expBuf = "RGme";
    if (static_cast<size_t>(datagram.size()) >= expBuf.size() && strncmp(datagram.data(), expBuf.data(), expBuf.size()) == 0)
      lastGCMessage = SystemCall::getRealSystemTime();

    emit updateGCStatus(true);
  }
}
