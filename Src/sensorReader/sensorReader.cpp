/**
 * @file sensorReader.cpp
 * provider of Sensordata
 */

#include "sensorReader.h"

#include <libbhuman/bhuman.h>
#include <ndevilsbase/ndevils.h>

#include "stdlib.h"

#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <regex>

#include <sys/mman.h>
#include <arpa/inet.h>
#include "Tools/Network/UdpComm.h"

/*
  @class SensorReader
  provide sensor data
  V6:
  has about 2% variance doing nothing
  has about 20% variance during play
*/
class SensorReader
{
public:
  int main();
  bool interrupt;
  void cleanUp();
private:
#ifdef NDEBUG
  static const int allowedFrameDrops = 3; /**< Maximum number of frame drops allowed before Nao sits down. */
#else
  static const int allowedFrameDrops = 6; /**< Maximum number of frame drops allowed before Nao sits down. */
#endif
  int memoryHandle; /**< The file handle of the shared memory. */
  NDData* dataV6 = (NDData*)MAP_FAILED; /**< The shared memory. */
  LBHData* dataV5 = (LBHData*)MAP_FAILED; /**< The shared memory. */
  std::map<std::string, float> repackData; /**< The repacked data to be sent */
  bool naoV6 = true;
  UdpComm socket;
  
  void packageV6();
  void packageV5();
  std::string mapToString(std::map<std::string, float> &map);

  int mapData();
  int createMemoryhandle();
  std::string runCommand(std::string cmd);
};

SensorReader theDevilsInstance;

void sigHandler(int signal)
{
  theDevilsInstance.interrupt = true;
  theDevilsInstance.cleanUp();
  exit(0);
}

int main()
{
  signal(SIGINT, sigHandler);//Create signalhandler for cleanup after interrupt
  signal(SIGTERM, sigHandler);//Create signalhandler for cleanup after terminate
  SensorReader theDevilsInstance;
  sched_param param;
  param.sched_priority = 1; //Set low thread priority (1-99)
  pthread_t handle = pthread_self();
  if (pthread_setschedparam(handle, SCHED_FIFO, &param))
    return 12;
  return theDevilsInstance.main();
}

int SensorReader::main()
{
  interrupt = false;
  createMemoryhandle(); //Create handle for shared memory and detect nao version
  mapData(); //Map the shared memory

  socket.setBlocking(false);
  socket.setBroadcast(true);
  socket.setLoopback(false);
  socket.bind("0.0.0.0", 55555);
  

  while (!interrupt)
  {
    sleep(5);
    if (naoV6)
    {
      packageV6(); //repack data for naoV6 (repackData)
    }
    else
    {
      packageV5(); //repack data for naoV5 (repackData)
    }

    char name[1024];
    gethostname(name, 1024);
    repackData[name] = 1.0f;
    std::string dataToSend = mapToString(repackData);
    std::regex r("\\s+");
    if (naoV6)
    {
      dataToSend = dataToSend + "bodyId:" + dataV6->bodyId + ",";
      int state = static_cast<int>(dataV6->state);
      std::stringstream stream;
      std::string stateValue;
      stream << state;
      stream >> stateValue;
      dataToSend = dataToSend + "state:" + stateValue + ",";
      std::ifstream f("/home/nao/scripts/wifiState.sh");
      if (f.good()) // check for wifiState.sh existence
      {
        dataToSend = dataToSend + "wifi_name:" + std::regex_replace(runCommand("/home/nao/scripts/wifiState.sh -n"), r , "") + ",";
        dataToSend = dataToSend + "wifi_state:" + std::regex_replace(runCommand("/home/nao/scripts/wifiState.sh -s"), r, "") + ",";
      }
    }
    else
    {
      dataToSend = dataToSend + "bodyId:" + dataV5->bodyId + ",";
      int state = static_cast<int>(dataV5->state);
      std::stringstream stream;
      std::string stateValue;
      stream << state;
      stream >> stateValue;
      dataToSend = dataToSend + "state:" + stateValue + ",";
      std::ifstream f("/home/nao/bin/wifiState.sh");
      if (f.good()) // check for wifiState.sh existence
      {
        dataToSend = dataToSend + "wifi_name:" + std::regex_replace(runCommand("/home/nao/bin/wifiState.sh -n"), r, "") + ",";
        dataToSend = dataToSend + "wifi_state:" + std::regex_replace(runCommand("/home/nao/bin/wifiState.sh -s"), r, "") + ",";
      }
    }

    int sizeOfDataToSend = static_cast<int>(dataToSend.size());
    while (sizeOfDataToSend < 4096) // fill up buffer for tcpComm
    {
      dataToSend += "@";
      sizeOfDataToSend++;
    }

    //Print dataToSend to check values for debug
    //std::cout << dataToSend << std::endl;
    const char* data = dataToSend.c_str();
    socket.setTarget("192.168.101.255", 55555);
    int size = socket.write(data, 4096);
    printf("data package sent: %d\n", size);
  }
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
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
  }
  return data;
}

int SensorReader::createMemoryhandle()
{
  std::ifstream f("/home/nao/robocup.conf");
  //Check for existence of shared memory file
  if ( f.good() )
  {
    naoV6 = true;
    std::cout << "try to open v6 shared memory" << std::endl;
    memoryHandle = shm_open(ND_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (memoryHandle == -1)
    {
      perror("sensorReader: shm_open");
    }
    else if (ftruncate(memoryHandle, sizeof(NDData)) == -1)
    {
      perror("sensorReader: ftruncate");
    }
  }
  else
  {
    naoV6 = false;
    std::cout << "try to open v5 shared memory" << std::endl;
    memoryHandle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (memoryHandle == -1)
    {
      perror("sensorReader: shm_open");
    }
    else if (ftruncate(memoryHandle, sizeof(NDData)) == -1)
    {
      perror("sensorReader: ftruncate");
    }
  }

  return memoryHandle;
}

int SensorReader::mapData()
{
  if (naoV6)
  {
    // map the shared memory
    std::cout << "mmap V6" << std::endl;
    dataV6 = (NDData*)mmap(nullptr, sizeof(NDData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
    if (dataV6 == MAP_FAILED)
    {
      perror("sensorReader: mmap");
      return 1;
    }
  }
  else
  {
    // map the shared memory
    std::cout << "mmap V5" << std::endl;
    dataV5 = (LBHData*)mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
    if (dataV5 == MAP_FAILED)
    {
      perror("sensorReader: mmap");
      return 1;
    }
  }
  return 0;
}


void SensorReader::cleanUp()
{
  fprintf(stderr, "\nsensorReader: Stopping.\n");
  
  if(dataV6 != MAP_FAILED)
    munmap(dataV6, sizeof(NDData));


  if(dataV5 != MAP_FAILED)
    munmap(dataV5, sizeof(LBHData));

  fprintf(stderr, "sensorReader: Stopped.\n");
}

std::string SensorReader::mapToString(std::map<std::string, float> &map)
{
  std::string output = "";
  std::string conv = "";

  for (std::map<std::string, float>::iterator it = map.begin(); it != map.end(); it++)
  {
    conv = std::to_string(it->second);
    output += (it->first) + ":" + (conv)+",";
  }

  return output;
}

void SensorReader::packageV6() //no status or timestamp
{
  repackData["acc_x"] = dataV6->sensors[dataV6->newestSensors].acc[0];
  repackData["acc_y"] = dataV6->sensors[dataV6->newestSensors].acc[1];
  repackData["acc_z"] = dataV6->sensors[dataV6->newestSensors].acc[2];
  repackData["angle_x"] = dataV6->sensors[dataV6->newestSensors].angle[0];
  repackData["angle_y"] = dataV6->sensors[dataV6->newestSensors].angle[1];
  repackData["battery_charge"] = dataV6->sensors[dataV6->newestSensors].battery[NDBattery::charge];
  repackData["battery_status"] = dataV6->sensors[dataV6->newestSensors].battery[NDBattery::status];
  repackData["battery_current"] = dataV6->sensors[dataV6->newestSensors].battery[NDBattery::current];
  repackData["battery_temperature"] = dataV6->sensors[dataV6->newestSensors].battery[NDBattery::temperature];
  repackData["headYaw_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::headYaw];
  repackData["headPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::headPitch];
  repackData["lShoulderPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lShoulderPitch];
  repackData["lShoulderRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lShoulderRoll];
  repackData["lElbowYaw_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lElbowYaw];
  repackData["lElbowRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lElbowRoll];
  repackData["lWristYaw_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lWristYaw];
  repackData["lHipYawPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lHipYawPitch];
  repackData["lHipRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lHipRoll];
  repackData["lHipPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lHipPitch];
  repackData["lKneePitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lKneePitch];
  repackData["lAnklePitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lAnklePitch];
  repackData["lAnkleRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lAnkleRoll];
  repackData["rHipRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rHipRoll];
  repackData["rHipPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rHipPitch];
  repackData["rKneePitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rKneePitch];
  repackData["rAnklePitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rAnklePitch];
  repackData["rAnkleRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rAnkleRoll];
  repackData["rShoulderPitch_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rShoulderPitch];
  repackData["rShoulderRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rShoulderRoll];
  repackData["rElbowYaw_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rElbowYaw];
  repackData["rElbowRoll_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rElbowRoll];
  repackData["rWristYaw_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rWristYaw];
  repackData["lHand_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::lHand];
  repackData["rHand_current"] = dataV6->sensors[dataV6->newestSensors].current[NDJoints::rHand];
  repackData["lFrontLeft_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::lFrontLeft];
  repackData["lFrontRight_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::lFrontRight];
  repackData["lRearLeft_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::lRearLeft];
  repackData["lRearRight_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::lRearRight];
  repackData["rFrontLeft_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::rFrontLeft];
  repackData["rFrontRight_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::rFrontRight];
  repackData["rRearLeft_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::rRearLeft];
  repackData["rRearRight_fsr"] = dataV6->sensors[dataV6->newestSensors].fsr[NDFsrs::rRearRight];
  repackData["gyro_x"] = dataV6->sensors[dataV6->newestSensors].gyro[0];
  repackData["gyro_y"] = dataV6->sensors[dataV6->newestSensors].gyro[1];
  repackData["gyro_z"] = dataV6->sensors[dataV6->newestSensors].gyro[2];
  repackData["headYaw_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::headYaw];
  repackData["headPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::headPitch];
  repackData["lShoulderPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lShoulderPitch];
  repackData["lShoulderRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lShoulderRoll];
  repackData["lElbowYaw_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lElbowYaw];
  repackData["lElbowRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lElbowRoll];
  repackData["lWristYaw_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lWristYaw];
  repackData["lHipYawPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lHipYawPitch];
  repackData["lHipRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lHipRoll];
  repackData["lHipPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lHipPitch];
  repackData["lKneePitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lKneePitch];
  repackData["lAnklePitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lAnklePitch];
  repackData["lAnkleRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lAnkleRoll];
  repackData["rHipRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rHipRoll];
  repackData["rHipPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rHipPitch];
  repackData["rKneePitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rKneePitch];
  repackData["rAnklePitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rAnklePitch];
  repackData["rAnkleRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rAnkleRoll];
  repackData["rShoulderPitch_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rShoulderPitch];
  repackData["rShoulderRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rShoulderRoll];
  repackData["rElbowYaw_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rElbowYaw];
  repackData["rElbowRoll_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rElbowRoll];
  repackData["rWristYaw_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rWristYaw];
  repackData["lHand_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::lHand];
  repackData["rHand_position"] = dataV6->sensors[dataV6->newestSensors].position[NDJoints::rHand];
  repackData["sonar_left"] = dataV6->sensors[dataV6->newestSensors].sonar[NDSonars::left];
  repackData["sonar_right"] = dataV6->sensors[dataV6->newestSensors].sonar[NDSonars::right];
  /*repackData["headYaw_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::headYaw];
  repackData["headPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::headPitch];
  repackData["lShoulderPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lShoulderPitch];
  repackData["lShoulderRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lShoulderRoll];
  repackData["lElbowYaw_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lElbowYaw];
  repackData["lElbowRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lElbowRoll];
  repackData["lWristYaw_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lWristYaw];
  repackData["lHipYawPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lHipYawPitch];
  repackData["lHipRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lHipRoll];
  repackData["lHipPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lHipPitch];
  repackData["lKneePitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lKneePitch];
  repackData["lAnklePitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lAnklePitch];
  repackData["lAnkleRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lAnkleRoll];
  repackData["rHipRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rHipRoll];
  repackData["rHipPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rHipPitch];
  repackData["rKneePitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rKneePitch];
  repackData["rAnklePitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rAnklePitch];
  repackData["rAnkleRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rAnkleRoll];
  repackData["rShoulderPitch_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rShoulderPitch];
  repackData["rShoulderRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rShoulderRoll];
  repackData["rElbowYaw_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rElbowYaw];
  repackData["rElbowRoll_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rElbowRoll];
  repackData["rWristYaw_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rWristYaw];
  repackData["lHand_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::lHand];
  repackData["rHand_stiffness"] = dataV6->sensors[dataV6->newestSensors].stiffness[NDJoints::rHand];*/
  repackData["headYaw_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::headYaw];
  repackData["headPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::headPitch];
  repackData["lShoulderPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lShoulderPitch];
  repackData["lShoulderRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lShoulderRoll];
  repackData["lElbowYaw_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lElbowYaw];
  repackData["lElbowRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lElbowRoll];
  repackData["lWristYaw_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lWristYaw];
  repackData["lHipYawPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lHipYawPitch];
  repackData["lHipRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lHipRoll];
  repackData["lHipPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lHipPitch];
  repackData["lKneePitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lKneePitch];
  repackData["lAnklePitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lAnklePitch];
  repackData["lAnkleRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lAnkleRoll];
  repackData["rHipRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rHipRoll];
  repackData["rHipPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rHipPitch];
  repackData["rKneePitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rKneePitch];
  repackData["rAnklePitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rAnklePitch];
  repackData["rAnkleRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rAnkleRoll];
  repackData["rShoulderPitch_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rShoulderPitch];
  repackData["rShoulderRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rShoulderRoll];
  repackData["rElbowYaw_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rElbowYaw];
  repackData["rElbowRoll_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rElbowRoll];
  repackData["rWristYaw_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rWristYaw];
  repackData["lHand_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::lHand];
  repackData["rHand_stiffness_ac"] = dataV6->sensors[dataV6->newestActuators].stiffness[NDJoints::rHand];
  repackData["headYaw_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::headYaw];
  repackData["headPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::headPitch];
  repackData["lShoulderPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lShoulderPitch];
  repackData["lShoulderRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lShoulderRoll];
  repackData["lElbowYaw_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lElbowYaw];
  repackData["lElbowRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lElbowRoll];
  repackData["lWristYaw_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lWristYaw];
  repackData["lHipYawPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lHipYawPitch];
  repackData["lHipRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lHipRoll];
  repackData["lHipPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lHipPitch];
  repackData["lKneePitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lKneePitch];
  repackData["lAnklePitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lAnklePitch];
  repackData["lAnkleRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lAnkleRoll];
  repackData["rHipRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rHipRoll];
  repackData["rHipPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rHipPitch];
  repackData["rKneePitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rKneePitch];
  repackData["rAnklePitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rAnklePitch];
  repackData["rAnkleRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rAnkleRoll];
  repackData["rShoulderPitch_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rShoulderPitch];
  repackData["rShoulderRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rShoulderRoll];
  repackData["rElbowYaw_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rElbowYaw];
  repackData["rElbowRoll_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rElbowRoll];
  repackData["rWristYaw_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rWristYaw];
  repackData["lHand_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::lHand];
  repackData["rHand_temperature"] = dataV6->sensors[dataV6->newestSensors].temperature[NDJoints::rHand];
  repackData["chestButton_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::chestButton];
  repackData["headFront_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::headFront];
  repackData["headMiddle_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::headMiddle];
  repackData["headRear_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::headRear];
  repackData["lBumperLeft_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::lBumperLeft];
  repackData["lBumperRight_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::lBumperRight];
  repackData["lHandBack_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::lHandBack];
  repackData["lHandLeft_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::lHandLeft];
  repackData["lHandRight_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::lHandRight];
  repackData["rBumperLeft_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::rBumperLeft];
  repackData["rBumperRight"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::rBumperRight];
  repackData["rHandBack_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::rHandBack];
  repackData["rHandLeft_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::rHandLeft];
  repackData["rHandRight_touch"] = dataV6->sensors[dataV6->newestSensors].touch[NDTouchs::rHandRight];
}

void SensorReader::packageV5()
{
  float* sensors = dataV5->sensors[dataV5->readingSensors];
  float* actuators = dataV5->actuators[dataV5->readingActuators];
  
  repackData["acc_x"] = sensors[LBHSensorIds::accXSensor];
  repackData["acc_y"] = sensors[LBHSensorIds::accYSensor];
  repackData["acc_z"] = sensors[LBHSensorIds::accZSensor];
  repackData["angle_x"] = sensors[LBHSensorIds::angleXSensor];
  repackData["angle_y"] = sensors[LBHSensorIds::angleYSensor];
  repackData["battery_charge"] = sensors[LBHSensorIds::batteryChargeSensor];
  repackData["battery_status"] = sensors[LBHSensorIds::batteryStatusSensor];
  repackData["battery_current"] = sensors[LBHSensorIds::batteryCurrentSensor];
  repackData["battery_temperature"] = sensors[LBHSensorIds::batteryTemperatureSensor];
  //repackData["battery_temperaturestatus"] = sensors[LBHSensorIds::batteryTemperatureStatusSensor];
  repackData["headYaw_current"] = sensors[LBHSensorIds::headYawCurrentSensor];
  repackData["headPitch_current"] = sensors[LBHSensorIds::headPitchCurrentSensor];
  repackData["lShoulderPitch_current"] = sensors[LBHSensorIds::lShoulderPitchCurrentSensor];
  repackData["lShoulderRoll_current"] = sensors[LBHSensorIds::lShoulderRollCurrentSensor];
  repackData["lElbowYaw_current"] = sensors[LBHSensorIds::lElbowYawCurrentSensor];
  repackData["lElbowRoll_current"] = sensors[LBHSensorIds::lElbowRollCurrentSensor];
  repackData["lWristYaw_current"] = sensors[LBHSensorIds::lWristYawCurrentSensor];
  repackData["lHipYawPitch_current"] = sensors[LBHSensorIds::lHipYawPitchCurrentSensor];
  repackData["lHipRoll_current"] = sensors[LBHSensorIds::lHipRollCurrentSensor];
  repackData["lHipPitch_current"] = sensors[LBHSensorIds::lHipPitchCurrentSensor];
  repackData["lKneePitch_current"] = sensors[LBHSensorIds::lKneePitchCurrentSensor];
  repackData["lAnklePitch_current"] = sensors[LBHSensorIds::lAnklePitchCurrentSensor];
  repackData["lAnkleRoll_current"] = sensors[LBHSensorIds::lAnkleRollCurrentSensor];
  repackData["rHipRoll_current"] = sensors[LBHSensorIds::rHipRollCurrentSensor];
  repackData["rHipPitch_current"] = sensors[LBHSensorIds::rHipPitchCurrentSensor];
  repackData["rKneePitch_current"] = sensors[LBHSensorIds::rKneePitchCurrentSensor];
  repackData["rAnklePitch_current"] = sensors[LBHSensorIds::rAnklePitchCurrentSensor];
  repackData["rAnkleRoll_current"] = sensors[LBHSensorIds::rAnkleRollCurrentSensor];
  repackData["rShoulderPitch_current"] = sensors[LBHSensorIds::rShoulderPitchCurrentSensor];
  repackData["rShoulderRoll_current"] = sensors[LBHSensorIds::rShoulderRollCurrentSensor];
  repackData["rElbowYaw_current"] = sensors[LBHSensorIds::rElbowYawCurrentSensor];
  repackData["rElbowRoll_current"] = sensors[LBHSensorIds::rElbowRollCurrentSensor];
  repackData["rWristYaw_current"] = sensors[LBHSensorIds::rWristYawCurrentSensor];
  repackData["lHand_current"] = sensors[LBHSensorIds::lHandCurrentSensor];
  repackData["rHand_current"] = sensors[LBHSensorIds::rHandCurrentSensor];
  repackData["lFrontLeft_fsr"] = sensors[LBHSensorIds::lFSRFrontLeftSensor];
  repackData["lFrontRight_fsr"] = sensors[LBHSensorIds::lFSRFrontRightSensor];
  repackData["lRearLeft_fsr"] = sensors[LBHSensorIds::lFSRRearLeftSensor];
  repackData["lRearRight_fsr"] = sensors[LBHSensorIds::lFSRRearRightSensor];
  repackData["rFrontLeft_fsr"] = sensors[LBHSensorIds::rFSRFrontLeftSensor];
  repackData["rFrontRight_fsr"] = sensors[LBHSensorIds::rFSRFrontRightSensor];
  repackData["rRearLeft_fsr"] = sensors[LBHSensorIds::rFSRRearLeftSensor];
  repackData["rRearRight_fsr"] = sensors[LBHSensorIds::rFSRRearRightSensor];
  //repackData["rTotal_fsr"] = sensors[LBHSensorIds::lFSRTotalSensor];
  //repackData["rTotal_fsr"] = sensors[LBHSensorIds::rFSRTotalSensor];
  repackData["gyro_x"] = sensors[LBHSensorIds::gyroXSensor];
  repackData["gyro_y"] = sensors[LBHSensorIds::gyroYSensor];
  repackData["gyro_z"] = sensors[LBHSensorIds::gyroZSensor];
  repackData["headYaw_position"] = sensors[LBHSensorIds::headYawPositionSensor];
  repackData["headPitch_position"] = sensors[LBHSensorIds::headPitchPositionSensor];
  repackData["lShoulderPitch_position"] = sensors[LBHSensorIds::lShoulderPitchPositionSensor];
  repackData["lShoulderRoll_position"] = sensors[LBHSensorIds::lShoulderRollPositionSensor];
  repackData["lElbowYaw_position"] = sensors[LBHSensorIds::lElbowYawPositionSensor];
  repackData["lElbowRoll_position"] = sensors[LBHSensorIds::lElbowRollPositionSensor];
  repackData["lWristYaw_position"] = sensors[LBHSensorIds::lWristYawPositionSensor];
  repackData["lHipYawPitch_position"] = sensors[LBHSensorIds::lHipYawPitchPositionSensor];
  repackData["lHipRoll_position"] = sensors[LBHSensorIds::lHipRollPositionSensor];
  repackData["lHipPitch_position"] = sensors[LBHSensorIds::lHipPitchPositionSensor];
  repackData["lKneePitch_position"] = sensors[LBHSensorIds::lKneePitchPositionSensor];
  repackData["lAnklePitch_position"] = sensors[LBHSensorIds::lAnklePitchPositionSensor];
  repackData["lAnkleRoll_position"] = sensors[LBHSensorIds::lAnkleRollPositionSensor];
  repackData["rHipRoll_position"] = sensors[LBHSensorIds::rHipRollPositionSensor];
  repackData["rHipPitch_position"] = sensors[LBHSensorIds::rHipPitchPositionSensor];
  repackData["rKneePitch_position"] = sensors[LBHSensorIds::rKneePitchPositionSensor];
  repackData["rAnklePitch_position"] = sensors[LBHSensorIds::rAnklePitchPositionSensor];
  repackData["rAnkleRoll_position"] = sensors[LBHSensorIds::rAnkleRollPositionSensor];
  repackData["rShoulderPitch_position"] = sensors[LBHSensorIds::rShoulderPitchPositionSensor];
  repackData["rShoulderRoll_position"] = sensors[LBHSensorIds::rShoulderRollPositionSensor];
  repackData["rElbowYaw_position"] = sensors[LBHSensorIds::rElbowYawPositionSensor];
  repackData["rElbowRoll_position"] = sensors[LBHSensorIds::rElbowRollPositionSensor];
  repackData["rWristYaw_position"] = sensors[LBHSensorIds::rWristYawPositionSensor];
  repackData["lHand_position"] = sensors[LBHSensorIds::lHandPositionSensor];
  repackData["rHand_position"] = sensors[LBHSensorIds::rHandPositionSensor];
  repackData["sonar_left"] = sensors[LBHSensorIds::lUsSensor];
  //repackData["sonar_left1"] = sensors[LBHSensorIds::lUs1Sensor];
  //repackData["sonar_left2"] = sensors[LBHSensorIds::lUs2Sensor];
  //repackData["sonar_left3"] = sensors[LBHSensorIds::lUs3Sensor];
  //repackData["sonar_left4"] = sensors[LBHSensorIds::lUs4Sensor];
  //repackData["sonar_left5"] = sensors[LBHSensorIds::lUs5Sensor];
  //repackData["sonar_left6"] = sensors[LBHSensorIds::lUs6Sensor];
  //repackData["sonar_left7"] = sensors[LBHSensorIds::lUs7Sensor];
  //repackData["sonar_left8"] = sensors[LBHSensorIds::lUs8Sensor];
  //repackData["sonar_left9"] = sensors[LBHSensorIds::lUs9Sensor];
  repackData["sonar_right"] = sensors[LBHSensorIds::rUsSensor];
  //repackData["sonar_right1"] = sensors[LBHSensorIds::rUs1Sensor];
  //repackData["sonar_right2"] = sensors[LBHSensorIds::rUs2Sensor];
  //repackData["sonar_right3"] = sensors[LBHSensorIds::rUs3Sensor];
  // repackData["sonar_right4"] = sensors[LBHSensorIds::rUs4Sensor];
  // repackData["sonar_right5"] = sensors[LBHSensorIds::rUs5Sensor];
  //repackData["sonar_right6"] = sensors[LBHSensorIds::rUs6Sensor];
  //repackData["sonar_right7"] = sensors[LBHSensorIds::rUs7Sensor];
  //repackData["sonar_right8"] = sensors[LBHSensorIds::rUs8Sensor];
  //repackData["sonar_right9"] = sensors[LBHSensorIds::rUs9Sensor];
  repackData["headYaw_stiffness_ac"] = actuators[LBHActuatorIds::headYawStiffnessActuator];
  repackData["headPitch_stiffness_ac"] = actuators[LBHActuatorIds::headPitchStiffnessActuator];
  repackData["lShoulderPitch_stiffness_ac"] = actuators[LBHActuatorIds::lShoulderPitchStiffnessActuator];
  repackData["lShoulderRoll_stiffness_ac"] = actuators[LBHActuatorIds::lShoulderRollStiffnessActuator];
  repackData["lElbowYaw_stiffness_ac"] = actuators[LBHActuatorIds::lElbowYawStiffnessActuator];
  repackData["lElbowRoll_stiffness_ac"] = actuators[LBHActuatorIds::lElbowRollStiffnessActuator];
  repackData["lWristYaw_stiffness_ac"] = actuators[LBHActuatorIds::lWristYawStiffnessActuator];
  repackData["lHipYawPitch_stiffness_ac"] = actuators[LBHActuatorIds::lHipYawPitchStiffnessActuator];
  repackData["lHipRoll_stiffness_ac"] = actuators[LBHActuatorIds::lHipRollStiffnessActuator];
  repackData["lHipPitch_stiffness_ac"] = actuators[LBHActuatorIds::lHipPitchStiffnessActuator];
  repackData["lKneePitch_stiffness_ac"] = actuators[LBHActuatorIds::lKneePitchStiffnessActuator];
  repackData["lAnklePitch_stiffness_ac"] = actuators[LBHActuatorIds::lAnklePitchStiffnessActuator];
  repackData["lAnkleRoll_stiffness_ac"] = actuators[LBHActuatorIds::lAnkleRollStiffnessActuator];
  repackData["rHipRoll_stiffness_ac"] = actuators[LBHActuatorIds::rHipRollStiffnessActuator];
  repackData["rHipPitch_stiffness_ac"] = actuators[LBHActuatorIds::rHipPitchStiffnessActuator];
  repackData["rKneePitch_stiffness_ac"] = actuators[LBHActuatorIds::rKneePitchStiffnessActuator];
  repackData["rAnklePitch_stiffness_ac"] = actuators[LBHActuatorIds::rAnklePitchStiffnessActuator];
  repackData["rAnkleRoll_stiffness_ac"] = actuators[LBHActuatorIds::rAnkleRollStiffnessActuator];
  repackData["rShoulderPitch_stiffness_ac"] = actuators[LBHActuatorIds::rShoulderPitchStiffnessActuator];
  repackData["rShoulderRoll_stiffness_ac"] = actuators[LBHActuatorIds::rShoulderRollStiffnessActuator];
  repackData["rElbowYaw_stiffness_ac"] = actuators[LBHActuatorIds::rElbowYawStiffnessActuator];
  repackData["rElbowRoll_stiffness_ac"] = actuators[LBHActuatorIds::rElbowRollStiffnessActuator];
  repackData["rWristYaw_stiffness_ac"] = actuators[LBHActuatorIds::rWristYawStiffnessActuator];
  repackData["lHand_stiffness_ac"] = actuators[LBHActuatorIds::lHandStiffnessActuator];
  repackData["rHand_stiffness_ac"] = actuators[LBHActuatorIds::rHandStiffnessActuator];
  repackData["headYaw_temperature"] = sensors[LBHSensorIds::headYawTemperatureSensor];
  repackData["headPitch_temperature"] = sensors[LBHSensorIds::headPitchTemperatureSensor];
  repackData["lShoulderPitch_temperature"] = sensors[LBHSensorIds::lShoulderPitchTemperatureSensor];
  repackData["lShoulderRoll_temperature"] = sensors[LBHSensorIds::lShoulderRollTemperatureSensor];
  repackData["lElbowYaw_temperature"] = sensors[LBHSensorIds::lElbowYawTemperatureSensor];
  repackData["lElbowRoll_temperature"] = sensors[LBHSensorIds::lElbowRollTemperatureSensor];
  repackData["lWristYaw_temperature"] = sensors[LBHSensorIds::lWristYawTemperaturSensor];
  repackData["lHipYawPitch_temperature"] = sensors[LBHSensorIds::lHipYawPitchTemperatureSensor];
  repackData["lHipRoll_temperature"] = sensors[LBHSensorIds::lHipRollTemperatureSensor];
  repackData["lHipPitch_temperature"] = sensors[LBHSensorIds::lHipPitchTemperatureSensor];
  repackData["lKneePitch_temperature"] = sensors[LBHSensorIds::lKneePitchTemperatureSensor];
  repackData["lAnklePitch_temperature"] = sensors[LBHSensorIds::lAnklePitchTemperatureSensor];
  repackData["lAnkleRoll_temperature"] = sensors[LBHSensorIds::lAnkleRollTemperatureSensor];
  repackData["rHipRoll_temperature"] = sensors[LBHSensorIds::rHipRollTemperatureSensor];
  repackData["rHipPitch_temperature"] = sensors[LBHSensorIds::rHipPitchTemperatureSensor];
  repackData["rKneePitch_temperature"] = sensors[LBHSensorIds::rKneePitchTemperatureSensor];
  repackData["rAnklePitch_temperature"] = sensors[LBHSensorIds::rAnklePitchTemperatureSensor];
  repackData["rAnkleRoll_temperature"] = sensors[LBHSensorIds::rAnkleRollTemperatureSensor];
  repackData["rShoulderPitch_temperature"] = sensors[LBHSensorIds::rShoulderPitchTemperatureSensor];
  repackData["rShoulderRoll_temperature"] = sensors[LBHSensorIds::rShoulderRollTemperatureSensor];
  repackData["rElbowYaw_temperature"] = sensors[LBHSensorIds::rElbowYawTemperatureSensor];
  repackData["rElbowRoll_temperature"] = sensors[LBHSensorIds::rElbowRollTemperatureSensor];
  repackData["rWristYaw_temperature"] = sensors[LBHSensorIds::rWristYawTemperaturSensor];
  repackData["lHand_temperature"] = sensors[LBHSensorIds::lHandTemperaturSensor];
  repackData["rHand_temperature"] = sensors[LBHSensorIds::rHandTemperaturSensor];
  repackData["chestButton_touch"] = sensors[LBHSensorIds::chestButtonSensor];
  repackData["headFront_touch"] = sensors[LBHSensorIds::headTouchFrontSensor];
  repackData["headMiddle_touch"] = sensors[LBHSensorIds::headTouchMiddleSensor];
  repackData["headRear_touch"] = sensors[LBHSensorIds::headTouchRearSensor];
  repackData["lBumperLeft_touch"] = sensors[LBHSensorIds::lBumperLeftSensor];
  repackData["lBumperRight_touch"] = sensors[LBHSensorIds::lBumperRightSensor];
  repackData["lHandBack_touch"] = sensors[LBHSensorIds::lHandTouchBackSensor];
  repackData["lHandLeft_touch"] = sensors[LBHSensorIds::lHandTouchLeftSensor];
  repackData["lHandRight_touch"] = sensors[LBHSensorIds::lHandTouchRightSensor];
  repackData["rBumperLeft_touch"] = sensors[LBHSensorIds::rBumperLeftSensor];
  repackData["rBumperRight"] = sensors[LBHSensorIds::rBumperRightSensor];
  repackData["rHandBack_touch"] = sensors[LBHSensorIds::rHandTouchBackSensor];
  repackData["rHandLeft_touch"] = sensors[LBHSensorIds::rHandTouchLeftSensor];
  repackData["rHandRight_touch"] = sensors[LBHSensorIds::rHandTouchRightSensor];
}