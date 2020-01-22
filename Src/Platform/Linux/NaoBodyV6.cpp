/**
* @file Platform/linux/NaoBody.cpp
* Declaration of a class for accessing the body of the nao via NaoQi/libbhuman.
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
  sem_t* sem = SEM_FAILED; /**< The semaphore used for synchronizing to the NaoQi DCM. */
  NDData* ndData = (NDData*)MAP_FAILED; /**< The pointer to the mapped shared memory block. */
  SteadyTimePoint startTime;


  ~NaoBodyAccessV6()
  {
    cleanup();
  }

  bool init()
  {
    if(ndData != MAP_FAILED)
      return true;

    fd = shm_open(ND_MEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
    if(fd == -1)
      return false;

    sem = sem_open(ND_SEM_NAME, O_RDWR, S_IRUSR | S_IWUSR, 0);
    if(sem == SEM_FAILED)
    {
      close(fd);
      fd = -1;
      return false;
    }

    VERIFY((ndData = (NDData*)mmap(nullptr, sizeof(NDData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) != MAP_FAILED);
    ndData->state = okState;
    startTime = std::chrono::steady_clock::now();
    return true;
  }

  void cleanup()
  {
    if(ndData != MAP_FAILED)
    {
      munmap(ndData, sizeof(NDData));
      ndData = (NDData*)MAP_FAILED;
    }
    if(fd != -1)
    {
      close(fd);
      fd = -1;
    }
    if(sem != SEM_FAILED)
    {
      sem_close(sem);
      sem = SEM_FAILED;
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
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  naoBodyAccessV6.ndData->state = NDState(termSignal);
}

bool NaoBodyV6::wait()
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  ASSERT(naoBodyAccessV6.sem != SEM_FAILED);
  do
  {
    if(sem_wait(naoBodyAccessV6.sem) == -1)
    {
      bool success = false;
      while(errno == 516)
      {
        if(sem_wait(naoBodyAccessV6.sem) == -1)
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
      if(!success)
      {
        ASSERT(false);
        return false;
      }
    }
  }
  while(naoBodyAccessV6.ndData->readingSensors == naoBodyAccessV6.ndData->newestSensors);
  naoBodyAccessV6.ndData->readingSensors = naoBodyAccessV6.ndData->newestSensors;

  static bool shout = true;
  if(shout)
  {
    shout = false;
    printf("LoLA is working\n");
  }

  return true;
}

const char* NaoBodyV6::getHeadId() const
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  return naoBodyAccessV6.ndData->headId;
}

const char* NaoBodyV6::getBodyId() const
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  return naoBodyAccessV6.ndData->bodyId;
}

NDSensorData* NaoBodyV6::getSensors()
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  return &naoBodyAccessV6.ndData->sensors[naoBodyAccessV6.ndData->readingSensors];
}

float NaoBodyV6::getCPUTemperature()
{
  float cpu = 0;

  if(!fdCpuTemp)
  {
    fdCpuTemp = fopen("/sys/class/hwmon/hwmon1/temp2_input", "r");
    ASSERT(fdCpuTemp);
  }

  if(fdCpuTemp)
  {
    VERIFY(fscanf(fdCpuTemp, "%f", &cpu) == 1);
    VERIFY(fclose(fdCpuTemp) == 0);
    fdCpuTemp = 0;
  }

  return cpu/1000.f;
}

float NaoBodyV6::getTransitionToBhuman()
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  return naoBodyAccessV6.ndData->transitionToBhuman;
}

bool NaoBodyV6::getWlanStatus()
{
  return access("/sys/class/net/wlan0", F_OK) == 0;
}

void NaoBodyV6::openActuators(NDActuatorData*& actuators)
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  ASSERT(writingActuators == -1);
  writingActuators = 0;
  if(writingActuators == naoBodyAccessV6.ndData->newestActuators)
    ++writingActuators;
  if(writingActuators == naoBodyAccessV6.ndData->readingActuators)
    if(++writingActuators == naoBodyAccessV6.ndData->newestActuators)
      ++writingActuators;
  ASSERT(writingActuators != naoBodyAccessV6.ndData->newestActuators);
  ASSERT(writingActuators != naoBodyAccessV6.ndData->readingActuators);
  actuators = &naoBodyAccessV6.ndData->actuators[writingActuators];
}

void NaoBodyV6::closeActuators()
{
  ASSERT(naoBodyAccessV6.ndData != (NDData*)MAP_FAILED);
  ASSERT(writingActuators >= 0);
  naoBodyAccessV6.ndData->newestActuators = writingActuators;
  writingActuators = -1;
}
