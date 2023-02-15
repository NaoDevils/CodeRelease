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

  nlohmann::json getJson() const;

  bool mapSharedMemory();
  std::string runCommand(std::string cmd);

  static void sigHandler(int signal);
};
