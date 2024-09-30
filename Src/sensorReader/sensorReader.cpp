/**
 * @file sensorReader.cpp
 * provider of Sensordata
 */

#include "sensorReader.h"

#include "stdlib.h"

#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <regex>
#include <cassert>
#include <iostream>

#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <naodevilsbase.h>

bool SensorReader::shutdown = false;

int main()
{
  std::cout << "Starting sensor reader." << std::endl;
  SensorReader theInstance;
  sched_param param;
  param.sched_priority = 1; //Set low thread priority (1-99)
  pthread_t handle = pthread_self();
  if (pthread_setschedparam(handle, SCHED_FIFO, &param))
    return 12;
  return theInstance.main();
}

void SensorReader::sigHandler(int signal)
{
  if (!shutdown)
    printf("Caught signal %i\nShutting down...", signal);

  shutdown = true;
}

int SensorReader::main()
{
  signal(SIGINT, SensorReader::sigHandler);
  signal(SIGTERM, SensorReader::sigHandler);

  socket.setBlocking(false);
  socket.setBroadcast(true);
  socket.setLoopback(false);
  socket.bind("0.0.0.0", 55555);

  gcSocket.setBlocking(false);
  gcSocket.setBroadcast(true);
  gcSocket.setLoopback(false);
  gcSocket.bind("0.0.0.0", 3838);

  // wait for start up of naodevilsbase
  while (!shutdown && (!mapSharedMemory() || dataV6->bodyId[0] == 0))
    sleep(2);


  // load body name
  std::string robotConfig("/home/nao/Config/Robots/robots.cfg");
  std::string bodyName = "";

  std::ifstream cfg(robotConfig.c_str());
  if (cfg.is_open())
  {
    std::string line;
    const std::regex re(".*name\\s*=\\s*(\\S+);.*(" + std::string(dataV6->bodyId) + ").+");
    while (!cfg.eof())
    {
      std::getline(cfg, line);
      std::smatch match;
      if (std::regex_search(line, match, re) && match.size() == 3)
      {
        bodyName = match[1].str();
        break;
      }
    }
    cfg.close();
  }

  std::cout << "Start sending data." << std::endl;

  while (!shutdown)
  {
    nlohmann::json j = getJson();

    j["body_name"] = bodyName;

    std::array<char, 1024> name{0};
    gethostname(name.data(), name.size());
    j["name"] = std::string(name.data());

    const std::regex regex("State: .+\\((.+)\\)");
    const std::string output = runCommand("/usr/bin/networkctl status wlan0 -n 0");

    if (std::smatch match; std::regex_search(output, match, regex) && match.size() == 2)
      j["wifi_state"] = match[1].str();

    //load walk calibration quality
    try
    {
      std::string walkCalibration("/home/nao/Persistent/" + bodyName + "/walkCalibration.cfg");
      if (walkCalibrationTimestamp != std::filesystem::last_write_time(walkCalibration))
      {
        std::ifstream wc(walkCalibration.c_str());
        if (wc.is_open())
        {
          std::string line;
          const std::regex quality("qualityOfRobotHardware\\s*=\\s*(.*);");
          const std::regex calibrated("walkCalibrated\\s*=\\s*(.*);");

          while (!wc.eof())
          {
            std::getline(wc, line);
            std::smatch match;
            if (std::regex_search(line, match, quality) && match.size() == 2)
            {
              qualityOfRobotHardware = std::stof(match[1].str());
            }
            if (std::regex_search(line, match, calibrated) && match.size() == 2)
            {
              walkCalibrated = strcasecmp("true", match[1].str().c_str()) == 0;
              walkCalibrationTimestamp = std::filesystem::last_write_time(walkCalibration);
            }
          }
          // for debug
          //std::cout << j["body_name"] << ": walkCalibrated = " << walkCalibrated << " | qualityOfRobotHardware = " << qualityOfRobotHardware << std::endl;

          wc.close();
        }
      }
    }
    catch (const std::filesystem::filesystem_error& e)
    {
      walkCalibrated = false;
      // for debug
      //std::cout << "Catch: " << e.what() << std::endl;
    }

    j["hardware_quality"] = walkCalibrated ? qualityOfRobotHardware : -1.f;

    const std::vector<uint8_t> data = nlohmann::json::to_msgpack(j);

    // for debug
    //std::cout << j.dump(4) << std::endl;

    socket.setTarget("10.1.255.255", 55555);
    if (socket.write(reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size())))
      std::cout << "LAN package sent" << std::endl;

    const auto now = std::chrono::steady_clock::now();

    // check dorsh package
    {
      static constexpr std::string_view expBuf = "sendData";
      std::array<char, expBuf.size()> buf;
      unsigned int ip;
      while (socket.read(buf.data(), buf.size(), ip) == buf.size())
      {
        if ((ip & 0xFFFF0000) == 0x0A000000 // ip == 10.0.*.*
            && strncmp(buf.data(), expBuf.data(), expBuf.size()) == 0)
        {
          std::cout << "Dorsh package received" << std::endl;
          wlanRequest = now;
        }
      }
    }

    // check GC package
    {
      static constexpr std::string_view expBuf = "RGme";
      std::array<char, expBuf.size()> buf;
      while (gcSocket.read(buf.data(), buf.size()) == buf.size())
      {
        if (gcData != now && strncmp(buf.data(), expBuf.data(), expBuf.size()) == 0)
        {
          std::cout << "GC package received" << std::endl;
          gcData = now;
        }
      }
    }

    using namespace std::chrono_literals;
    if (now - wlanRequest < 15s && now - gcData > 10s)
    {
      socket.setTarget("10.0.255.255", 55555);
      if (socket.write(reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size())))
        std::cout << "WLAN package sent" << std::endl;
    }

    sleep(5);
  }

  cleanUp();

  std::cout << "Bye!" << std::endl;

  return 0;
}

std::string SensorReader::runCommand(std::string cmd)
{
  std::string data;
  FILE* stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL)
        data.append(buffer);
    pclose(stream);
  }
  return data;
}

bool SensorReader::mapSharedMemory()
{
  std::ifstream f("/home/nao/robocup.conf");
  //Check for existence of shared memory file
  assert(f.good());

  std::cout << "Try to open v6 shared memory" << std::endl;
  int memoryHandle = shm_open(NDData::mem_name, O_RDONLY, S_IRUSR | S_IWUSR);
  if (memoryHandle == -1)
  {
    perror("shm_open failed");
    return false;
  }

  void* mem = mmap(nullptr, sizeof(NDData), PROT_READ, MAP_SHARED, memoryHandle, 0);
  if (mem == MAP_FAILED)
  {
    perror("mmap failed");
    close(memoryHandle);
    return false;
  }

  // after mmap, memory handle can be closed
  close(memoryHandle);

  dataV6 = reinterpret_cast<const NDData*>(mem);

  return true;
}

void SensorReader::cleanUp()
{
  std::cout << "Stopping" << std::endl;

  if (dataV6 != MAP_FAILED)
    munmap(const_cast<void*>(reinterpret_cast<const void*>(dataV6)), sizeof(NDData));
}

nlohmann::json SensorReader::getJson() const
{
  // We have to read the write buffer here. Otherwise, when the framework is stopped,
  // the read buffer does not change and data is not updated.
  // Since data consistency is not so important here, this is fine.
  const auto& sensordata = dataV6->sensors.writeBuffer();

  return {{"sensors", sensordata}, {"headId", dataV6->headId}, {"bodyId", dataV6->bodyId}, {"state", dataV6->state}};
}
