#include "Utils/bush/agents/PowerAgent.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Sleeper.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/models/Robot.h"
#include "Platform/SystemCall.h"


#define UPDATE_TIME 30000

PowerAgent::PowerAgent(PingAgent* pingAgent) : pingAgent(pingAgent)
{}

PowerAgent::~PowerAgent()
{
  for(auto it = processes.cbegin(), end = processes.cend(); it != end; ++it)
    if(it->second)
      delete it->second;
  for (auto it = processesP.cbegin(), end = processesP.cend(); it != end; ++it)
	  if (it->second)
		  delete it->second;
}

void PowerAgent::initialize(std::map<std::string, Robot*>& robotsByName)
{
  Session::getInstance().registerPingListener(this);
  for(auto it = robotsByName.cbegin(), end = robotsByName.cend(); it != end; ++it)
  {
    power[it->first] = Power();
    timeOfLastUpdate[it->first] = -UPDATE_TIME;
	//processes for charge in %
    processes[it->first] = new QProcess(this);
    connect(processes[it->first], SIGNAL(readyReadStandardOutput()), this, SLOT(batteryReadable()));
  }
  emit powerChanged(&this->power);
}

void PowerAgent::setPings(ENetwork, std::map<std::string, double>*)
{
  const unsigned currentTime = SystemCall::getRealSystemTime();

  for(auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end; ++it)
  {
    if (pingAgent->getBestNetwork(it->second) == ENetwork::LAN)
    {
      if (currentTime - timeOfLastUpdate[it->first] > UPDATE_TIME)
      {
        const std::string ip = it->second->lan;
        const std::string python =
          "import naoqi\n"
          "import sys\n"
          "from naoqi import ALProxy\n"
          "try:\n"
          "  memory = ALProxy(\"ALMemory\",\"127.0.0.1\",9559)\n"
          "  value =  memory.getData(\"Device/SubDeviceList/Battery/Charge/Sensor/Value\")\n"
          "  status =  memory.getData(\"Device/SubDeviceList/Battery/Charge/Sensor/Status\")\n"
          "  print str(value) + \" \" + str(status)\n"
          "except:\n"
          "  print \"NoNAOqi 0.0\"\n";

        const std::string cmd = "python -";

        // ensure process is not running anymore
        if (processes[it->first]->state() != QProcess::ProcessState::NotRunning)
        {
          processes[it->first]->kill();
        }

        std::string c = remoteCommandForQProcess(cmd, ip);
        processes[it->first]->start(fromString(c));
        processes[it->first]->write(QByteArray(python.c_str()));
        processes[it->first]->closeWriteChannel();
        timeOfLastUpdate[it->first] = currentTime;
      }
    }
    else
    {
      if (power[it->first].value != 101 || power[it->first].powerPlugged == true)
      {
        power[it->first].value = 101;
        power[it->first].powerPlugged = false;
        power[it->first].naoqi = true;
        emit powerChanged(&this->power);
      }
    }
  }

}

void PowerAgent::batteryReadable()
{
  QProcess* process = dynamic_cast<QProcess*>(sender());

  QByteArray data = process->readAllStandardOutput();
  QString output(data);

  QRegExp rx("(?:^|\\n)([\\d.]+|NoNAOqi) ([-\\d.]+)\\n");
  if (rx.indexIn(output) < 0) return;
  QStringList list = rx.capturedTexts();
  if (list.size() < 3) return;

  std::string name = "";
  for (const auto &p : processes)
  {
    if (p.second == process)
    {
      name = p.first;
      break;
    }
  }

  if (name == "") return;

  if (list[1] != "NoNAOqi")
  {
    float value = list[1].toFloat();
    power[name].value = static_cast<int>(value * 100.f);
    power[name].powerPlugged = !(static_cast<short>(list[2].toFloat()) & 0b100000);
    power[name].naoqi = true;
  }
  else
  {
    power[name].value = 101;
    power[name].powerPlugged = false;
    power[name].naoqi = false;
  }

	emit powerChanged(&this->power);
}


void PowerAgent::reset(Robot* robot)
{
  if(robot)
  {
    power[robot->name].value = 101; // Invalid Value
    power[robot->name].powerPlugged = false;
    power[robot->name].naoqi = true;
    timeOfLastUpdate[robot->name] = -UPDATE_TIME;
    emit powerChanged(&this->power);
  }
}
