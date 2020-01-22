/**
 * @file ndevils.cpp
 * Implementation of a process that provides basic LoLA socket access and shares its data via semaphore and shared memory with the framework.
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <cmath>
#include <ctime>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include "stdlib.h"

#include "ndevils.h"

#include "msgpack.hpp"
#include <chrono>

const char *socket_path = "/tmp/robocup";
#define LOLA_SENSOR_SIZE 896

// helper function for msgpack str type
std::string to_string(const msgpack::object_str& str) 
{
  return std::string(str.ptr, str.size);
}

/*
  @class NDevils
  Handles connection between LoLA and our shared memory, starts in stand by state (chest button blinks blue).
  If chest button is pressed once, framework data is accepted.
  If chest button is then pressed thrice, this exe goes to standby state.
  If in standby state and sitting upright, hip pitch and yaw have stiffness for stabilization.
  If chest button is held for >3 seconds, a controlled shut down is initiated.
*/
class NDevils
{
public:
  int main();
private:
  static const int allowedFrameDrops = 10; /**< Maximum number of frame drops allowed before Nao sits down. */
  /*
    Takes @buffer from socket stream and unpacks msgpack (hard coded order!) into data pointer.
  */
  void unpack_data(const char* buffer);

  /*
    Packs data into a @sbuffer with hard coded array element order within category.
  */
  void pack_data(msgpack::sbuffer &sbuffer);
  
  // helper function for pack_data
  void pack_map(msgpack::packer<msgpack::sbuffer> &pk, const std::string &name, const int size, float* &data_ptr);

  void pack_map_bool(msgpack::packer<msgpack::sbuffer> &pk, const std::string &name, const int size, bool* &data_ptr);

  int fd = 0;
  int memoryHandle = -1; /**< The file handle of the shared memory. */
  NDData* data = (NDData*)MAP_FAILED; /**< The shared memory. */
  sem_t* sem = SEM_FAILED; /**< The semaphore used to notify bhuman about new data. */

  int lastReadingActuators = -1; /**< The previous actuators read. For detecting frames without seemingly new data from bhuman. */
  int actuatorDrops = 0; /**< The number of frames without seemingly new data from bhuman. */
  int frameDrops = allowedFrameDrops + 1; /**< The number frames without a reaction from bhuman. */
  int lolaConnectionAttempts = 5; /**< After this number of attempts, we quit. TODO: keep on trying! */

  SteadyTimePoint ndevilsbaseStartTime;
  
  //TODO: state names are misleading, since standing means framework active but can mean the robot is sitting.
  enum State {sitting, standingUp, standing, sittingDown} state = sitting; 
  bool shutDownRequested = false; /**< True, if chest button was pressed for 3 seconds. */
  bool chestButtonPressedAndReleased = false; // /**< Chest button was pressed previously and now released. */
  float phase = 0.f; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */

  float startAngles[numOfJoints]; /**< Start angles for standing up or sitting down. */

  bool stabilizationStatus = false; // If the robots is in sitDown position and stabilizing.
  bool chargingStatus = false; // If the robot is currently charging or not.

  int startPressedTime = 0; /**< The last time the chest button was not pressed. */
  SteadyTimePoint lastNDevilsStartTime; /**< The last time the framework was started. */

  int triplePressStartTime = 0; /**< After first chest button press within 2 secs, this timestamp is set. */
  int triplePressCount = 0; /**< Counter for transition between framework and base. On count 3, control changes */
  bool triplePressLastButtonState = false; /**< Remember last chest button state. */
  bool triplePressSitDownRequested = false; /**< After 3 presses, this is set and state transition to/from sitting starts */
  static bool run;

  void setLEyeLeds(float* actuators);
  void setREyeLeds(float* actuators);
  void setSkullLeds(float* actuators);

  void setChestLeds(float* chestLEDs);

  void stabilize(float* controlledPositions, float* controlledStiffnesses); /**< If robot is in sitting position, slight stabilization is applied. */

  void handleState(); /**< State transitions and actions for the bases State is handled. */
  void disableFrameworkWhenCharging();
 
  bool hasStiffness(); /**< Checks for stiffness. Helper function for safe transitions. */
  
  bool isStandHigh(); /**< Since our high stand uses no stiffness, check this for safe sitdown. */

  bool isPlayDead();
  
  static void sighandlerShutdown(int sig);
  void cleanUp();
};

bool NDevils::run = true;

void NDevils::unpack_data(const char* buffer)
{
  int writingSensors = 0;
  if(writingSensors == data->newestSensors)
    ++writingSensors;
  if(writingSensors == data->readingSensors)
    if(++writingSensors == data->newestSensors)
      ++writingSensors;
  assert(writingSensors != data->newestSensors);
  assert(writingSensors != data->readingSensors);

  float* data_ptr = (float*)&data->sensors[writingSensors]; // get pointer to start of sensor data
  
  msgpack::object_handle oh = 
    msgpack::unpack(&buffer[0], LOLA_SENSOR_SIZE); // char* to object handle
  
  // get map of key (category as string) and value (float, except status(int))
  // and copy to data->sensor field
  // first the float categories
  const auto& map = oh.get().via.map;
  auto* category = map.ptr;
  // ignore first category since robot config was read already
  category++;
  for (uint32_t i = 1; i < map.size-1; i++, category++) 
  {
    const auto& arr = category->val.via.array;
    for (uint32_t j = 0; j < arr.size; j++) 
    {
      *data_ptr = static_cast<float>((arr.ptr + j)->via.f64);
      ++data_ptr;
    }
  }
  // .. then the status category
  const auto& arr = category->val.via.array;
  int* data_ptr_int = (int*)data_ptr;
  for (uint32_t j = 0; j < arr.size; j++) 
  {
    *data_ptr_int = static_cast<int>((arr.ptr + j)->via.u64);
    ++data_ptr_int;
  }
  
  data->newestSensors = writingSensors;
  data->sensors[writingSensors].timestamp =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - ndevilsbaseStartTime).count();

  // If lying on front, ignore chest button data. This prevents presses due to ground contact.
  bool chestButtonPressed = !(std::abs(data->sensors[writingSensors].angle[0])*60 < 15
    && data->sensors[writingSensors].angle[1]*60 > 70)
    && data->sensors[writingSensors].touch[NDTouchs::chestButton] == 1.f;
  if (chestButtonPressed)
    data->sensors[writingSensors].touch[NDTouchs::chestButton] = 1.f;
  else
    data->sensors[writingSensors].touch[NDTouchs::chestButton] = 0.f;
  // detect transition or shutdown request via chest-button
  if(!chestButtonPressed)
    startPressedTime = data->sensors[writingSensors].timestamp;
  else if(!shutDownRequested && startPressedTime
    && (data->sensors[writingSensors].timestamp - startPressedTime) > 3000)
  {
    std::cout << "Shutdown requested, startPressedTime = " << startPressedTime << ", sensorTimestamp = " << data->sensors[writingSensors].timestamp << std::endl;
    (void) !system("( systemctl --user stop bhumand && sleep 5s && shutdown -h now ) &");
    shutDownRequested = true;
  }

  chestButtonPressedAndReleased = triplePressLastButtonState != chestButtonPressed && !chestButtonPressed;
  if (chestButtonPressedAndReleased)
  {
    // detect triple button press for sitdown  
    if (state != sitting)
    {
      if (triplePressCount == 0) triplePressStartTime = data->sensors[writingSensors].timestamp;
      triplePressCount++;
    }
    else
      triplePressCount = 0;
    std::cout << "chestButtonPressed" << std::endl;
    std::cout << "triplePressCount: " << triplePressCount << std::endl;
  }
  triplePressLastButtonState = chestButtonPressed;

  if ((data->sensors[writingSensors].timestamp - triplePressStartTime) > 2000 
    || triplePressSitDownRequested) triplePressCount = 0;
  if (triplePressCount == 3 && !chestButtonPressed)
    triplePressSitDownRequested = true;
  else
    triplePressSitDownRequested = false;

  // detect charing status
  chargingStatus = !(static_cast<int>(data->sensors[writingSensors].battery[NDBattery::status]) & 0b100000);
  
  // raise the semaphore
  if(sem != SEM_FAILED)
  {
    int sval;
    if(sem_getvalue(sem, &sval) == 0)
    {
      if(sval < 1)
      {
        sem_post(sem);
        if (frameDrops > 0)
          std::cout << "dropped " << frameDrops << " sensor data" << std::endl;
        frameDrops = 0;
      }
      else
      {
        if(frameDrops == 0)
          fprintf(stderr, "ndevilsbase: dropped sensor data.\n");
        ++frameDrops;
      }
    }
  }
}

void NDevils::pack_data(msgpack::sbuffer &sbuffer)
{
  data->readingActuators = data->newestActuators;

  if(data->readingActuators == lastReadingActuators)
  {
    if(actuatorDrops == 0)
      fprintf(stderr, "ndevilsbase: missed actuator request.\n");
    ++actuatorDrops;
  }
  else
  {
    if (actuatorDrops > 0)
      std::cout << "dropped " << actuatorDrops << " actuator requests" << std::endl;
    actuatorDrops = 0;
  }
  lastReadingActuators = data->readingActuators;
  disableFrameworkWhenCharging();
  handleState();
  /*std::cout << "stabilization: " << stabilizationStatus << std::endl;
  for (int i = 0; i < NDJoints::numOfJoints; i++)
    std::cout << data->sensors[data->newestSensors].position[i] << "; ";
  std::cout << std::endl;*/

  setLEyeLeds(&data->actuators[data->readingActuators].lEyeLEDs[0][0]);
  setREyeLeds(&data->actuators[data->readingActuators].rEyeLEDs[0][0]);

  setSkullLeds(&data->actuators[data->readingActuators].skullLEDs[0]);
  setChestLeds(&data->actuators[data->readingActuators].chestLEDs[0]);

  msgpack::packer<msgpack::sbuffer> pk(&sbuffer);
  pk.pack_map(11);
  float* data_ptr = data->actuators[data->readingActuators].chestLEDs;
  
  pack_map(pk, std::string("Chest"), numOfRGBLEDs, data_ptr);
  pack_map(pk, std::string("LEar"), numOfLEarLEDs, data_ptr);
  pack_map(pk, std::string("LEye"), numOfLEyeLEDs * numOfRGBLEDs, data_ptr);
  pack_map(pk, std::string("LFoot"), numOfRGBLEDs, data_ptr);
  pack_map(pk, std::string("Position"), numOfJoints, data_ptr);
  pack_map(pk, std::string("REar"), numOfREarLEDs, data_ptr);
  pack_map(pk, std::string("REye"), numOfREyeLEDs * numOfRGBLEDs, data_ptr);
  pack_map(pk, std::string("RFoot"), numOfRGBLEDs, data_ptr);
  pack_map(pk, std::string("Skull"), numOfSkullLEDs, data_ptr);
  pack_map(pk, std::string("Stiffness"), numOfJoints, data_ptr);
  bool* data_ptr_bool = &data->actuators[0].sonars[0];
  pack_map_bool(pk, std::string("Sonar"), numOfSonars, data_ptr_bool);
  return;
}

void NDevils::pack_map(msgpack::packer<msgpack::sbuffer> &pk, const std::string &name, const int size, float* &data_ptr)
{
  pk.pack(name);
  pk.pack_array(size);
  for (int i = 0; i < size; ++i, ++data_ptr)
    pk.pack(*data_ptr);
}

void NDevils::pack_map_bool(msgpack::packer<msgpack::sbuffer> &pk, const std::string &name, const int size, bool* &data_ptr)
{
  pk.pack(name);
  pk.pack_array(size);
  for (int i = 0; i < size; ++i, ++data_ptr)
    pk.pack(*data_ptr);
}

static const float sitDownAngles[25] =
{
  0.f, //head
  0.f,

  0.89f, //l arm
  0.06f,
  0.26f,
  -0.62f,
  -1.57f,
  
  0.f, // l leg
  0.f,
  -0.855f,
  2.11f,
  -1.18f,
  0.f,

  0.f, // r leg
  -0.855f,
  2.11f,
  -1.18f,
  0.f,

  0.89f, // r arm
  -0.06f,
  -0.26f,
  0.62f,
  1.57f,

  0.f, // l hand
  0.f // r hand

};

static const float sitDownAnglesHighArms[25] =
{
  0.f, // head
  0.f,

  0.712f, // l arm
  0.164f,
  0.080f,
  -0.045f,
  -1.57f,
  
  0.f, // l leg
  0.f,
  -0.855f,
  2.16f,
  -1.18f,
  0.f,

  0.f, // r leg
  -0.855f,
  2.16f,
  -1.18f,
  0.f,

  0.712f, // r arm
  -0.164f,
  -0.080f,
  0.045f,
  1.57f,
  
  0.f, // l hand
  0.f // r hand
};

static const float standHighAngles[25] =
{
  0.f, // head
  0.349f,
  
  1.571f, // l arm
  0.201f,
  1.571f,
  0.f,
  -1.571f,
  
  0.f, // l leg
  -0.035f,
  0.157f,
  -0.087f,
  0.035f,
  0.f,
  
  -0.035f, // r leg
  0.157f,
  -0.087f,
  0.035f,
  0.f,

  1.571f, // r arm
  -0.201f,
  -1.571f,
  0.f,
  1.571f,
  
  0.f, // l hand
  0.f // r hand
};

int main()
{
  NDevils theDevilsInstance;
  sched_param param;
  param.sched_priority = 18;
  pthread_t handle = pthread_self();
  if (pthread_setschedparam(handle, SCHED_FIFO, &param))
    return 12;
  return theDevilsInstance.main();
}

int NDevils::main()
{
  ndevilsbaseStartTime = std::chrono::steady_clock::now();

  // create shared memory
  memoryHandle = shm_open(ND_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (memoryHandle == -1)
  {
    perror("ndevilsbase: shm_open");
    return 3;
  }
  else if (ftruncate(memoryHandle, sizeof(NDData)) == -1)
  {
    perror("ndevilsbase: ftruncate");
    cleanUp();
    return 4;
  }

  // map the shared memory
  data = (NDData*) mmap(nullptr, sizeof(NDData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
  if (data == MAP_FAILED)
  {
    perror("ndevilsbase: mmap");
    cleanUp();
    return 1;
  }
  memset(data, 0, sizeof(NDData));

  // open semaphore
  sem = sem_open(ND_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
  if (sem == SEM_FAILED)
  {
    perror("ndevilsbase: sem_open");
    cleanUp();
    return 2;
  }
  // TODO: get body and head ids

  signal(SIGTERM, sighandlerShutdown);
  signal(SIGINT, sighandlerShutdown);

  struct sockaddr_un addr;
  socklen_t addr_length;
  char buffer[LOLA_SENSOR_SIZE];
  int size = 0;
  while (run)
  {
    if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)
    {
      sleep(3);
      printf("ndevilsbase: could not create socket\n");
      continue;
    }
    printf("ndevilsbase: created socket!\n");
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, socket_path);
    addr_length = sizeof(addr.sun_family) + strlen(addr.sun_path);

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
      sleep(3);
      printf("ndevilsbase: could not connect to socket\n");
      continue;
    }
    printf("ndevilsbase: connected to LoLA socket!\n");
    while (run)
    {
      size = recv(fd, buffer, LOLA_SENSOR_SIZE, 0);
      if (size == LOLA_SENSOR_SIZE)
      {
        printf("ndevilsbase: Got the LoLA header package!\n");
        msgpack::object_handle oh = 
          msgpack::unpack(&buffer[0], LOLA_SENSOR_SIZE); // char* to object handle
        // std::cout << oh.get() << std::endl; // dump header definition into cout
        // get body id and head id
        const auto& map = oh.get().via.map;
        auto* robotConfig = map.ptr;
        
        std::string key = to_string(robotConfig->key.via.str);
        if (key == "RobotConfig")
        {
          const auto& rc_arr = robotConfig->val.via.array;
          // order in array is body id, body version, head id, head version
          strcpy(data->bodyId, to_string((rc_arr.ptr + 0)->via.str).c_str());
          strcpy(data->headId, to_string((rc_arr.ptr + 2)->via.str).c_str());
          
          lolaConnectionAttempts = 5;
        }
        else
        {
          printf("ndevilsbase: failed to read RobotConfig from LoLA package, self destruction commencing in %d.\n", lolaConnectionAttempts);
          lolaConnectionAttempts--;
          break;
        }
        std::cout << "My Head ID is: " << std::string(data->headId, 20) << std::endl;
        std::cout << "My Body ID is: " << std::string(data->bodyId, 20) << std::endl;
        while (run)
        {
          size = recv(fd, buffer, LOLA_SENSOR_SIZE, 0);
          if (size == LOLA_SENSOR_SIZE)
          {
            unpack_data(&buffer[0]);
            msgpack::sbuffer sbuffer;
            pack_data(sbuffer);
            //unsigned size = 
            send(fd, sbuffer.data(), sbuffer.size(), 0);
            //printf("libndevils: sent %u bytes!\n", size);
          }
          else
          {
            std::cout << "ndevilsbase: inner loop: LoLA package does not have correct size!" << std::endl;
            break;
          }
        } // end of sensor read loop (inner loop)
      } // end of if (first package received)
      else
      {
        std::cout << "ndevilsbase: outer loop: LoLA package does not have correct size!" << std::endl;
        std::cout << "ndevilsbase: retry socket connection in 3s." << std::endl;
        sleep(3);
        break;
      }
    } // end of head read loop (outer loop)
  }
  cleanUp();
  return 0;
}

void NDevils::sighandlerShutdown(int sig)
{
  if (run)
    printf("ndevilsbase: Caught signal %i\nShutting down...\n", sig);
  run = false;
}

void NDevils::cleanUp()
{
  fprintf(stderr, "ndevilsbase: Stopping.\n");

  if (fd)
    close(fd);
  if (sem != SEM_FAILED)
  {
    sem_close(sem);
    sem_unlink(ND_SEM_NAME);
  }
  if(data != MAP_FAILED)
    munmap(data, sizeof(NDData));
  if (memoryHandle != -1)
  {
    close(memoryHandle);
    memoryHandle = -1;
    shm_unlink(ND_MEM_NAME);
  }

  fprintf(stderr, "ndevilsbase: Stopped.\n");
}

/**
  * Set the eye LEDs based on the current state.
  * Shutting down -> Lower segments are red.
  * bhuman crashed -> Whole eyes alternately flash red.
  * bhuman not running -> Lower segments flash blue.
  * @param actuators The actuator values a part of which will be set by this method.
  */
void NDevils::setLEyeLeds(float* eyeLEDs)
{
  if (!shutDownRequested && data->state == okState && frameDrops < allowedFrameDrops)
    return;
  int sensorTimestamp = data->sensors[data->newestSensors].timestamp;
  for (int rgb = 0; rgb < numOfRGBLEDs; rgb++)
    for(int i = 0; i < numOfLEyeLEDs; ++i)
      eyeLEDs[i+rgb*numOfLEyeLEDs] = 0.f;

  if(shutDownRequested)
  {
    eyeLEDs[NDLEyeLEDs::lEyeDeg180 + NDRGBLEDs::r * numOfLEyeLEDs ] = 1.f;
  }
  else if(data->state != okState && data->state != NDState::sigTERMState)
  {
    // set the "ndevilsbase is active and bhuman crashed" leds
    float blink = float(sensorTimestamp / 500 & 1);
    for(int i = 0; i < numOfLEyeLEDs; ++i)
      eyeLEDs[i + NDRGBLEDs::r * numOfLEyeLEDs] = blink;
  }
  else if (frameDrops >= allowedFrameDrops || data->state == NDState::sigTERMState)
  {
    // set the "ndevilsbase is active and bhuman is not running" LEDs
    float blink = float(sensorTimestamp / 500 & 1);
    eyeLEDs[NDLEyeLEDs::lEyeDeg180 + NDRGBLEDs::b * numOfLEyeLEDs ] = blink;
  }
}

void NDevils::setREyeLeds(float* eyeLEDs)
{
  if (!shutDownRequested && data->state == okState && frameDrops < allowedFrameDrops)
    return;
  int sensorTimestamp = data->sensors[data->newestSensors].timestamp;
  for (int rgb = 0; rgb < numOfRGBLEDs; rgb++)
    for(int i = 0; i < numOfREyeLEDs; ++i)
      eyeLEDs[i+rgb*numOfREyeLEDs] = 0.f;

  if(shutDownRequested)
  {
    eyeLEDs[NDREyeLEDs::rEyeDeg180 + NDRGBLEDs::r * numOfREyeLEDs ] = 1.f;
  }
  else if (data->state != okState && data->state != NDState::sigTERMState)
  {
    // set the "ndevilsbase is active and bhuman crashed" leds
    float blink = float(sensorTimestamp / 500 & 1);
    for(int i = 0; i < numOfREyeLEDs; ++i)
      eyeLEDs[i + NDRGBLEDs::r * numOfREyeLEDs] = blink;
  }
  else if (frameDrops >= allowedFrameDrops || data->state == NDState::sigTERMState)
  {
    // set the "ndevilsbase is active and bhuman is not running" LEDs
    float blink = float(sensorTimestamp / 500 & 1);
    eyeLEDs[NDREyeLEDs::rEyeDeg180 + NDRGBLEDs::b * numOfREyeLEDs ] = blink;
  }
}

/**
* Shows the battery state in the right ear if the robot is in the standing state
* and bhuman has not used one of these LEDs in the past 5 seconds.
* @param actuators The actuator values a part of which will be set by this method.
*/
void NDevils::setSkullLeds(float* actuators)
{
  int sensorTimestamp = data->sensors[data->newestSensors].timestamp;
  //std::cout << "sensor timestamp set skull: " << sensorTimestamp << std::endl;
  
  if(chargingStatus) 
  {
    float blink = float(sensorTimestamp / 500 & 1);
    actuators[NDSkullLEDs::rearLeft0] = 1.f - blink;
    actuators[NDSkullLEDs::rearLeft1] = 1.f - blink;
    actuators[NDSkullLEDs::rearLeft2] = 1.f - blink;
    actuators[NDSkullLEDs::rearRight0] = blink;
    actuators[NDSkullLEDs::rearRight1] = blink;
    actuators[NDSkullLEDs::rearRight2] = blink;
    actuators[NDSkullLEDs::middleRight0] = blink;
    actuators[NDSkullLEDs::frontRight0] = blink;
    actuators[NDSkullLEDs::frontRight1] = blink;
    actuators[NDSkullLEDs::frontLeft0] = 1.f - blink;
    actuators[NDSkullLEDs::frontLeft1] = 1.f - blink;
    actuators[NDSkullLEDs::middleLeft0] = 1.f - blink;
  }

  if (stabilizationStatus)
  {
    for (int i = 0; i < numOfSkullLEDs; i++)
      if(actuators[i] == 0.f) actuators[i] = 0.1f;
  }
}

/**
  * Shows the transitionToBHuman state with the chest LEDs
  * @param actuators The actuator values a part of which will be set by this method.
  */
void NDevils::setChestLeds(float* chestLEDs)
{
  int sensorTimestamp = data->sensors[data->newestSensors].timestamp;
  
  if (state == sitting || triplePressSitDownRequested)
  {
    chestLEDs[NDRGBLEDs::r] = 0.f;
    chestLEDs[NDRGBLEDs::g] = 0.f;
    chestLEDs[NDRGBLEDs::b] = float(sensorTimestamp / 500 & 1);
  }
}

/**
  * Stabilize robot if in sitDown position.
  */
void NDevils::stabilize(float* controlledPositions, float* controlledStiffnesses)
{
  stabilizationStatus = true;

  // check if robot is sitting correctly and stabilization should be enabled
  for (int i = NDJoints::lHipYawPitch; i <= NDJoints::rAnkleRoll; ++i)
  {
    // don't stabilize if joint is not close enough to sitDown position
    if (std::abs(data->sensors[data->newestSensors].position[i] - sitDownAngles[i]) > 0.3f)
    {
      stabilizationStatus = false;
      break;
    }
  }
    
  static bool handsClosed[2] = { false, false };
    
  // set joints to sitDown position
  if (stabilizationStatus)
  { 
    // stabilization
    controlledPositions[NDJoints::lHipYawPitch] = sitDownAngles[NDJoints::lHipYawPitch];
    controlledStiffnesses[NDJoints::lHipYawPitch] = 0.1f;
    controlledPositions[NDJoints::lHipPitch] = sitDownAngles[NDJoints::lHipPitch];
    controlledStiffnesses[NDJoints::lHipPitch] = 0.1f;
    controlledPositions[NDJoints::rHipPitch] = sitDownAngles[NDJoints::rHipPitch];
    controlledStiffnesses[NDJoints::rHipPitch] = 0.1f;

    // close hands if opened
    if (std::abs(data->sensors[data->newestSensors].position[NDJoints::lHand] - sitDownAngles[NDJoints::lHand]) > 0.25f)
      handsClosed[0] = false;
    if (std::abs(data->sensors[data->newestSensors].position[NDJoints::rHand] - sitDownAngles[NDJoints::rHand]) > 0.25f)
      handsClosed[1] = false;
    // skip closing hands if closed
    if (std::abs(data->sensors[data->newestSensors].position[NDJoints::lHand] - sitDownAngles[NDJoints::lHand]) < 0.05f)
      handsClosed[0] = true;
    if (std::abs(data->sensors[data->newestSensors].position[NDJoints::rHand] - sitDownAngles[NDJoints::rHand]) < 0.05f)
      handsClosed[1] = true;

    if (handsClosed[0] == false)
    {
      controlledPositions[NDJoints::lHand] = sitDownAngles[NDJoints::lHand];
      controlledStiffnesses[NDJoints::lHand] = 0.4f;
    }
    if (handsClosed[1] == false)
    {
      controlledPositions[NDJoints::rHand] = sitDownAngles[NDJoints::rHand];
      controlledStiffnesses[NDJoints::rHand] = 0.4f;
    }
  }
}

/**
  * Handles the different states ndevilsbase can be in.
  * @param actuators The actuator values requested. They will not be changed, but might
  *                  be used as result of this method.
  * @return The actuator values that should be set. In the standing state, they are
  *         identical to the actuators passed to this method. In all other states,
  *         they are different.
  */
void NDevils::handleState()
{
  // too many frame drops-> sit down
  // framework stopped or crashed -> sit down
  if ((frameDrops > allowedFrameDrops || data->state != okState) && state != sitting && state != sittingDown)
  {
    std::cout << "frameDrops > allowedFrameDrops || data->state != okState" << std::endl;

    if (isPlayDead() || stabilizationStatus)
    {
      std::cout << "switching to state: sitting" << std::endl;
      state = sitting;
    }
    else
    {
      std::cout << "switching to state: sittingDown" << std::endl;
      state = sittingDown;
      // reset start angles
      for (int i = 0; i < NDJoints::numOfJoints; ++i)
        startAngles[i] = data->sensors[data->newestSensors].position[i];
      phase = 0.f;
    }
  }

  stabilizationStatus = false;

  // state transitions
  switch(state)
  {
    case sitting:
    {
      bool bodyUpright = std::abs(data->sensors[data->newestSensors].angle[0]) < 0.2f
        && (data->sensors[data->newestSensors].angle[1]) < 0.2f;
      if (!shutDownRequested && frameDrops <= allowedFrameDrops && chestButtonPressedAndReleased && bodyUpright)
      {
        std::cout << "switching to state: standingUp" << std::endl;
        state = standingUp;
        // reset start angles
        for(int i = 0; i < NDJoints::numOfJoints; ++i)
          startAngles[i] = data->sensors[data->newestSensors].position[i];
        phase = 0.f;
      }
      break;
    }
    case standingUp: // if standing up complete, goto standing, framedrops goto sittingDown
    {
      if (phase >= 1.f)
      {
        state = standing;
        std::cout << "switching to state: standing" << std::endl;
      }
      break;
    }
    case standing:
    {
      if (triplePressSitDownRequested || shutDownRequested)
      {
        if (isPlayDead())
        {
          std::cout << "switching to state: sitting" << std::endl;
          state = sitting;
        }
        else
        {
          std::cout << "switching to state: sittingDown" << std::endl;
          state = sittingDown;
          // reset start angles
          for (int i = 0; i < NDJoints::numOfJoints; ++i)
            startAngles[i] = data->sensors[data->newestSensors].position[i];
          phase = 0.f;
        }
      }
      break;
    }
    case sittingDown:
    {
      if (phase >= 1.f)
      {
        state = sitting;
        std::cout << "switching to state: sitting" << std::endl;
      }
      break;
    }
  }

  // action in states
  switch(state)
  {
    case sitting:
    {
      float* positions = data->actuators[data->readingActuators].positions;
      float* stiffness = data->actuators[data->readingActuators].stiffness;
      memset(positions, 0, NDJoints::numOfJoints * sizeof(float));
      memset(stiffness, 0, NDJoints::numOfJoints * sizeof(float));
      data->transitionToBhuman = 0.f;

      stabilize(positions, stiffness);

      break;
    }
    case standingUp:
    {
      triplePressCount = 0;
      phase = std::min(phase + 0.005f, 1.f);
      data->transitionToBhuman = phase;
      if (isPlayDead())
        break;
      float sinPhase = (std::cos(phase * 3.1416f) - 1.f) / -2.f;
      for (int i = 0; i < NDJoints::numOfJoints; ++i)
      {
        float* actuator = &data->actuators[data->readingActuators].positions[i];
        *actuator = *actuator * sinPhase + startAngles[i] * (1.f - sinPhase);
        float* stiffness = &data->actuators[data->readingActuators].stiffness[i];
        float h = std::max(*stiffness, 0.5f);
        *stiffness = *stiffness * sinPhase + h * (1.f - sinPhase);
      }
      break;
    } 
    case standing:
    {
      // stabilize if robot has no stiffness or frameDrops increase
      if(!hasStiffness() || frameDrops > 0)
      {
        stabilize(data->actuators[data->readingActuators].positions, data->actuators[data->readingActuators].stiffness);
      }
      break;
    }
    case sittingDown:
    {
      phase = std::min(phase + 0.005f, 1.f);
      data->transitionToBhuman = 1.f - phase;
      float sinPhase = (std::cos(phase * 3.1416f) - 1.f) / -2.f;
      if (phase < 0.75f)
        sinPhase = (std::cos(phase / 0.75f * 3.1416f) - 1.f) / -2.f;
      else
        sinPhase = (std::cos((phase - 0.75f) / 0.25f * 3.1416f) - 1.f) / -2.f;
      for (int i = 0; i < NDJoints::numOfJoints; ++i)
      {
        float* actuator = &data->actuators[data->readingActuators].positions[i];
        if (phase < 0.75f)
          *actuator = startAngles[i] * (1.f - sinPhase) + sitDownAnglesHighArms[i] * sinPhase;
        else
          *actuator = sitDownAnglesHighArms[i] * (1.f - sinPhase) + sitDownAngles[i] * sinPhase;
        float* stiffness = &data->actuators[data->readingActuators].stiffness[i];
        *stiffness = 0.5f; // TODO: test
      }
      break;
    }
  }
}
 
/** This method disables the framework if the robot has no stiffness and starts charging. */
void NDevils::disableFrameworkWhenCharging() 
{
  static bool lastChargingStatus = false;
  if(state == standing &&
    !hasStiffness() &&
    !isStandHigh() &&
    chargingStatus &&
    !lastChargingStatus)
    triplePressSitDownRequested = true;
    
  lastChargingStatus = chargingStatus;
}
  
/** This method checks if the given actuator request has stiffness > 0 */
bool NDevils::hasStiffness() 
{
  for (int i = 0; i < NDJoints::numOfJoints; ++i) 
  {
    if(data->actuators[data->readingActuators].stiffness[i] > 0.f) return true;
  }
    
  return false;
}
  
/** This method checks if the robot is in stand high pose */
bool NDevils::isStandHigh() 
{
  for (int i = NDJoints::lHipYawPitch; i <= NDJoints::rAnkleRoll; ++i)
  {
    if (std::abs(data->sensors[data->newestSensors].position[i] - standHighAngles[i]) > 0.2f)
    {
      return false;
    }
  }
    
  // check if body is upright
  if(std::abs(data->sensors[data->newestSensors].angle[0]) > 0.2f) return false;
  if(std::abs(data->sensors[data->newestSensors].angle[1]) > 0.2f) return false;
    
  return true;
}

/** This method checks if the robot is in play dead */
bool NDevils::isPlayDead() 
{
  if (isStandHigh())
    return false;
  for (int i = NDJoints::lHipYawPitch; i <= NDJoints::rAnkleRoll; ++i)
  {
    if (data->actuators[data->readingActuators].stiffness[i] > 0.0f)
    {
      return false;
    }
  }
      
  return true;
}
