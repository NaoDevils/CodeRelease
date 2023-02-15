/**
* @file Platform/linux/NaoBody.cpp
* Declaration of a class for accessing the body of the nao via NaoQi/ndevilsbase.
* @author TheCanadianGuy & schwingmar
*/

#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>

#include "NaoBodyV6.h"
#include "BHAssert.h"

class NaoBodyAccessV6
{
public:
  int fd = -1; /**< The file descriptor for the shared memory block. */
  sem_t* sem_sensors = SEM_FAILED; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  sem_t* sem_actuators = SEM_FAILED; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  NDData* ndData = reinterpret_cast<NDData*>(MAP_FAILED); /**< The pointer to the mapped shared memory block. */
  NDData::SteadyTimePoint startTime;


  ~NaoBodyAccessV6() { cleanup(); }

  bool init()
  {
    if (ndData != MAP_FAILED)
      return true;

    fd = shm_open(NDData::mem_name, O_RDWR, S_IRUSR | S_IWUSR);
    if (fd == -1)
      return false;

    sem_sensors = sem_open(NDData::sem_name_sensors, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if (sem_sensors == SEM_FAILED)
    {
      close(fd);
      fd = -1;
      return false;
    }
    sem_actuators = sem_open(NDData::sem_name_actuators, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if (sem_actuators == SEM_FAILED)
    {
      close(fd);
      fd = -1;
      return false;
    }

    VERIFY((ndData = reinterpret_cast<NDData*>(mmap(nullptr, sizeof(NDData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0))) != MAP_FAILED);
    ndData->state = NDData::okState;
    startTime = std::chrono::steady_clock::now();
    return true;
  }

  void cleanup()
  {
    if (ndData != MAP_FAILED)
    {
      munmap(ndData, sizeof(NDData));
      ndData = reinterpret_cast<NDData*>(MAP_FAILED);
    }
    if (fd != -1)
    {
      close(fd);
      fd = -1;
    }
    if (sem_sensors != SEM_FAILED)
    {
      sem_close(sem_sensors);
      sem_sensors = SEM_FAILED;
    }
    if (sem_actuators != SEM_FAILED)
    {
      sem_close(sem_actuators);
      sem_actuators = SEM_FAILED;
    }
  }
} naoBodyAccessV6;

NaoBodyV6::~NaoBodyV6()
{
  //if(fdCpuTemp)
  //  fclose(fdCpuTemp);
}

bool NaoBodyV6::init()
{
  return naoBodyAccessV6.init();
}

void NaoBodyV6::cleanup()
{
  naoBodyAccessV6.cleanup();
}

void NaoBodyV6::setCrashed(int termSignal)
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  naoBodyAccessV6.ndData->state = NDData::State(termSignal);
}

bool NaoBodyV6::wait()
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  ASSERT(naoBodyAccessV6.sem_sensors != SEM_FAILED);
  do
  {
    if (sem_wait(naoBodyAccessV6.sem_sensors) == -1)
    {
      bool success = false;
      while (errno == 516)
      {
        if (sem_wait(naoBodyAccessV6.sem_sensors) == -1)
        {
          ASSERT(false);
          continue;
        }
        else
        {
          success = true;
          break;
        }
      }
      if (!success)
      {
        ASSERT(false);
        return false;
      }
    }
  } while (!naoBodyAccessV6.ndData->sensors.beginRead());

  return true;
}

const char* NaoBodyV6::getHeadId() const
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  return naoBodyAccessV6.ndData->headId;
}

const char* NaoBodyV6::getBodyId() const
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  return naoBodyAccessV6.ndData->bodyId;
}

const NDData::SensorData& NaoBodyV6::getSensors()
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  return naoBodyAccessV6.ndData->sensors.readBuffer();
}

float NaoBodyV6::getCPUTemperature()
{
  float cpu = 0;

  if (!fdCpuTemp)
  {
    // hwmon1 for Linux 4.4.86-rt99
    // hwmon2 for Linux 5.4.70-rt40
    fdCpuTemp = fopen("/sys/class/hwmon/hwmon1/temp2_input", "r");
    if (!fdCpuTemp)
      fdCpuTemp = fopen("/sys/class/hwmon/hwmon2/temp2_input", "r");
    ASSERT(fdCpuTemp);
  }

  if (fdCpuTemp)
  {
    VERIFY(fscanf(fdCpuTemp, "%f", &cpu) == 1);
    VERIFY(fclose(fdCpuTemp) == 0);
    fdCpuTemp = 0;
  }

  return cpu / 1000.f;
}

float NaoBodyV6::getTransitionToFramework()
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  return naoBodyAccessV6.ndData->transitionToFramework.load(std::memory_order_relaxed);
}

bool NaoBodyV6::getWlanStatus()
{
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

NDData::ActuatorData& NaoBodyV6::openActuators()
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  return naoBodyAccessV6.ndData->actuators.writeBuffer();
}

void NaoBodyV6::closeActuators()
{
  ASSERT(naoBodyAccessV6.ndData != reinterpret_cast<NDData*>(MAP_FAILED));
  ASSERT(naoBodyAccessV6.sem_actuators != SEM_FAILED);

  naoBodyAccessV6.ndData->actuators.finishWrite();

  int sval;
  if (sem_getvalue(naoBodyAccessV6.sem_actuators, &sval) == 0)
  {
    if (sval < 1)
    {
      sem_post(naoBodyAccessV6.sem_actuators);
    }
  }
}
