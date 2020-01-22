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

#define UPDATE_TIME 5000

DataAgent::DataAgent()
{
  //binds to Port and listens for new Data
  udpSocket.bind(QHostAddress::Any, 55555, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(&udpSocket, SIGNAL(readyRead()), this, SLOT(updateData()));
  wifiSocket.bind(QHostAddress::Any, 10000 + teamNumber, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(&wifiSocket, SIGNAL(readyRead()), this, SLOT(updateWifiConnection()));
  //starts timer for disabeling powerbar/pingbar
  timer.setSingleShot(false);
  connect(&timer, SIGNAL(timeout()), this, SLOT(teamObserver()));
  timer.start(UPDATE_TIME);
}

DataAgent::~DataAgent()
{
}

//initialize every robot with an empty Map, no power and no Wifi connection
void DataAgent::initialize(std::map<std::string, RobotConfigDorsh*>& robotsByName)
{
  for(auto it = robotsByName.cbegin(), end = robotsByName.cend(); it != end; ++it)
  {
    timeOfLastUpdate[it->first] = -UPDATE_TIME;
    timeOfLastUpdateWifi[it->first] = -UPDATE_TIME;
    std::map<std::string, std::string> emptyMap;
    emptyMap[it->first] = 1;
    emptyMap["battery_charge"] = "0.000000";
    emptyMap["battery_status"] = "-32708.000000";
    emptyMap["state"] = "0";
    emit newSensorData(emptyMap);
    emit wifiConnection(it->first, false);
  }
}

bool DataAgent::isLANReachable(const RobotConfigDorsh* robot)
{
  const unsigned currentTime = SystemCall::getRealSystemTime();
  auto iter = timeOfLastUpdate.find(robot->name);
  if (iter != timeOfLastUpdate.cend())
  {
    if (currentTime - iter->second <= UPDATE_TIME)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool DataAgent::isWLANReachable(const RobotConfigDorsh* robot)
{
  const unsigned currentTime = SystemCall::getRealSystemTime();
  auto iter = timeOfLastUpdateWifi.find(robot->name);
  if (iter != timeOfLastUpdateWifi.cend())
  {
    if (currentTime - iter->second <= UPDATE_TIME)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

void DataAgent::teamObserver()
{
  unsigned  short newTeamNumber = *Session::getInstance().teamNumber;
  if ( 10000 + newTeamNumber != wifiSocket.localPort() )
  {
    wifiSocket.flush();
    wifiSocket.close();
    bool succ = wifiSocket.bind(QHostAddress::Any, 10000 + teamNumber);
    if ( succ )
    {
      teamNumber = newTeamNumber;
    }
  }

  updateRobot();
}

//updates the Map with received SensorData package
void DataAgent::updateData()
{
  if (udpSocket.hasPendingDatagrams())
  {
    QByteArray datagram;
    datagram.resize(udpSocket.pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;

    udpSocket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    receivedMap = stringToMap(datagram.data());

    //Print map for debug
    /*for ( auto it = receivedMap.cbegin(), end = receivedMap.cend(); it != end; ++it)
    {
      printf("%s %s\n", it->first.c_str(), it->second.c_str());
    }*/

    const unsigned currentTime = SystemCall::getRealSystemTime();

    if (!receivedMap.empty())
    {
      std::string robotName;
      for (auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end; ++it)
      {
        auto iter = receivedMap.find(it->second->name);
        if (iter != receivedMap.cend())
        {
          robotName = it->second->name;
          timeOfLastUpdate[it->second->name] = currentTime;
          break;
        }
      }
      receivedMap["lan_connection"] = "1.000000";
    }
    emit newSensorData(receivedMap);

    //Disable powerbar if no package is received
    updateRobot();
  }
}

//updates the Wifi conncetion with received TeamData package
void DataAgent::updateWifiConnection()
{
  if (wifiSocket.hasPendingDatagrams())
  {
    QByteArray datagram;
    datagram.resize(wifiSocket.pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;

    wifiSocket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    const unsigned currentTime = SystemCall::getRealSystemTime();

    std::string robotName;
    for (auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end; ++it)
    {
      if (it->second->wlan == sender.toString().toLatin1().data())
      {
        robotName = it->second->name;
        timeOfLastUpdateWifi[robotName] = currentTime;
        break;
      }
    }
    emit wifiConnection(robotName, true);

    //Disable powerbar if no package is received
    updateRobot();
  } 
}

//Disable sensorReader map, powerbar and pingbar if packages are expired
void DataAgent::updateRobot()
{
  const unsigned currentTime = SystemCall::getRealSystemTime();
  for (auto it = timeOfLastUpdate.cbegin(), end = timeOfLastUpdate.cend(); it != end; ++it)
  {
    if (currentTime - it->second > UPDATE_TIME)
    {
      std::map<std::string, std::string> emptyMap;
      emptyMap[it->first] = 1;
      emptyMap["battery_charge"] = "0.000000";
      emptyMap["battery_status"] = "-32708.000000";
      emptyMap["lan_connection"] = "0.000000";
      emptyMap["state"] = "0";
      emptyMap["wifi_state"] = "";
      emptyMap["wifi_name"] = "";
      emit newSensorData(emptyMap);
    }
  }

  for (auto it = timeOfLastUpdateWifi.cbegin(), end = timeOfLastUpdateWifi.cend(); it != end; ++it)
  {
    if (currentTime - it->second > UPDATE_TIME)
    {
      emit wifiConnection(it->first, false);
    }
  }
}

//resets the robot with an empty Map, no power and no Wifi connection
void DataAgent::reset(RobotConfigDorsh* robot)
{
  if(robot)
  {
    std::map<std::string, std::string> emptyMap;
    emptyMap[robot->name] = 1;
    emptyMap["battery_charge"] = "0.000000";
    emptyMap["battery_status"] = "-32708.000000";
    emptyMap["lan_connection"] = "0.000000";
    emptyMap["state"] = "0";
    emit newSensorData(emptyMap);

    std::string empty;
    emit wifiConnection(empty, false);
  }
}

//converts the sensorReader package String to Map
std::map<std::string, std::string> DataAgent::stringToMap(char mapCharArray[])
{
  std::map<std::string, std::string> receivedMap;
  std::string name;
  std::string val;
  bool secondVal = false;

  for (int i = 0; i < 4096; i++)
  {
    if (mapCharArray[i] == '@')
    {
      break;
    }
    if (mapCharArray[i] == ':')
    {
      secondVal = true;
    }
    else if (mapCharArray[i] == ',')
    {
      secondVal = false;
      receivedMap[name] = val;
      name = "";
      val = "";
    }
    else
    {
      if (secondVal)
      {
        val += mapCharArray[i];
      }
      else
      {
        name += mapCharArray[i];
      }
    }
  }
  return receivedMap;
}