/**
* @file Modules/Infrastructure/RobotHealthProvider.h
* This file implements a module that provides information about the robot's health.
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/


#include "RobotHealthProvider.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

#include <algorithm>
#include <iterator>
#include <numeric>

RobotHealthProvider::RobotHealthProvider()
    : lastExecutionTime(0), lastRelaxedHealthComputation(0), startBatteryLow(0), lastBatteryLevel(1), batteryVoltageFalling(false), highTemperatureSince(0)
#ifdef TARGET_ROBOT
      ,
      lastWlanCheckedTime(0)
#endif
{
}

void RobotHealthProvider::update(RobotHealth& robotHealth)
{
  // count percepts
  if (theBallPercept.status == BallPercept::seen || !theMultipleBallPercept.balls.empty())
    ++robotHealth.ballPercepts;
  robotHealth.linePercepts += static_cast<unsigned>(theCLIPFieldLinesPercept.lines.size());
  robotHealth.goalPercepts += static_cast<unsigned>(theCLIPGoalPercept.goalPosts.size());

  // Transfer information from other process:
  robotHealth = theMotionRobotHealth;
  // Compute frame rate of cognition process:
  unsigned now = SystemCall::getCurrentSystemTime();
  if (lastExecutionTime != 0)
    timeBuffer.push_front(now - lastExecutionTime);
  robotHealth.cognitionFrameRate = timeBuffer.sum() ? 1000.0f / timeBuffer.averagef() : 0.f;
  lastExecutionTime = now;

  std::string robotName = enableName ? Global::getSettings().robotName + "! " : "";

  // read cpu and mainboard temperature
  robotHealth.cpuTemperature = (unsigned char)theSystemSensorData.cpuTemperature;
  if (robotHealth.cpuTemperature > cpuHeat && theFrameInfo.getTimeSince(highCPUTemperatureSince) / 1000 > 20)
  {
    highCPUTemperatureSince = theFrameInfo.time;
    SystemCall::text2Speech(robotName + "CPU temperature at exclamation mark");
  }
#ifdef TARGET_ROBOT
  if (theFrameInfo.getTimeSince(lastWlanCheckedTime) > 10 * 1000)
  {
    lastWlanCheckedTime = theFrameInfo.time;
    ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
    robotHealth.wlan = NaoBodyV6::getWlanStatus();
  }
#endif

  if (theFrameInfo.getTimeSince(lastRelaxedHealthComputation) > 5000)
  {
    lastRelaxedHealthComputation = theFrameInfo.time;

    // transfer maximal temperature, battery level and total current from SensorData:
    robotHealth.batteryLevel = (unsigned char)((theSystemSensorData.batteryLevel == SensorData::off ? 1.f : theSystemSensorData.batteryLevel) * 100.f);
    const auto maxTemperature = std::max_element(theJointSensorData.temperatures.begin() + 14, theJointSensorData.temperatures.end());
    robotHealth.maxJointTemperature = *maxTemperature;
    robotHealth.jointWithMaxTemperature = static_cast<Joints::Joint>(std::distance(theJointSensorData.temperatures.begin() + 14, maxTemperature));
    robotHealth.totalCurrent = std::accumulate(theJointSensorData.currents.begin(), theJointSensorData.currents.end(), 0.0f);

    // Add cpu load, memory load and robot name:
    float memoryUsage, load[3];
    SystemCall::getLoad(memoryUsage, load);
    robotHealth.load[0] = (unsigned char)(load[0] * 10.f);
    robotHealth.load[1] = (unsigned char)(load[1] * 10.f);
    robotHealth.load[2] = (unsigned char)(load[2] * 10.f);
    robotHealth.memoryUsage = (unsigned char)(memoryUsage * 100.f);
    robotHealth.robotName = Global::getSettings().robotName;

    //battery warning
    if (lastBatteryLevel < robotHealth.batteryLevel)
      batteryVoltageFalling = false;
    else if (lastBatteryLevel > robotHealth.batteryLevel)
      batteryVoltageFalling = true;
    if (robotHealth.batteryLevel < batteryLow)
    {
      if (batteryVoltageFalling && theFrameInfo.getTimeSince(startBatteryLow) > 1000)
      {
        SystemCall::text2Speech(robotName + "low battery");
        //next warning in 90 seconds
        startBatteryLow = theFrameInfo.time + 30000;
        batteryVoltageFalling = false;
      }
    }
    else if (startBatteryLow < theFrameInfo.time)
      startBatteryLow = theFrameInfo.time;
    lastBatteryLevel = robotHealth.batteryLevel;

    //temperature warning
    if (robotHealth.maxJointTemperature > temperatureHeat)
    {
      if (theFrameInfo.getTimeSince(highTemperatureSince) > 1000)
      {
        if (robotHealth.maxJointTemperature > temperatureFireExclamationMark)
          SystemCall::text2Speech(robotName + "fire exclamation mark");
        else if (robotHealth.maxJointTemperature > temperatureFire)
          SystemCall::text2Speech(robotName + "fire");
        else
          SystemCall::text2Speech(robotName + "heat");
        highTemperatureSince = theFrameInfo.time + 20000;
      }
    }
    else if (highTemperatureSince < theFrameInfo.time)
      highTemperatureSince = theFrameInfo.time;

    robotHealth.configuration = RobotHealth::Configuration::CONFIGURATION;
    robotHealth.overlays = Global::getSettings().overlays;
  }
}

MAKE_MODULE(RobotHealthProvider, cognitionInfrastructure)
