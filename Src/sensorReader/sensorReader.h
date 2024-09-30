/**
* @file sensorReader.h
* Declaration of shared data provided by sensorReader.
* @author Dominik & Fried
*/

#pragma once

#include <map>
#include <sys/mman.h>
#include <nlohmann/json_fwd.hpp>
#include <chrono>
#include "../Tools/Network/UdpComm.h"
#include <filesystem>

struct NDData;

class SensorReader
{
public:
  int main();
  void cleanUp();

private:
  static bool shutdown;

  const NDData* dataV6 = reinterpret_cast<const NDData*>(MAP_FAILED); /**< The shared memory. */
  UdpComm socket;
  UdpComm gcSocket;

  std::chrono::time_point<std::chrono::steady_clock> wlanRequest;
  std::chrono::time_point<std::chrono::steady_clock> gcData;
  std::filesystem::file_time_type walkCalibrationTimestamp;
  float qualityOfRobotHardware = 1.f;
  bool walkCalibrated = false;

  nlohmann::json getJson() const;

  bool mapSharedMemory();
  std::string runCommand(std::string cmd);

  static void sigHandler(int signal);
};
