/**
 * @file naodevilsbase.cpp
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
#include <iostream>
#include "stdlib.h"

#include "naodevilsbase.h"

#include <chrono>
#include <nlohmann/json.hpp>


/*
  @class NDevils
  Handles connection between LoLA and our shared memory, starts in stand by state (chest button blinks blue).
  If chest button is pressed once, framework data is accepted.
  If chest button is then pressed thrice, this exe goes to standby state.
  If in standby state and sitting upright, hip pitch and yaw have stiffness for stabilization.
  If chest button is held for >3 seconds, a controlled shut down is initiated.
*/

using namespace std::chrono_literals;

class NDevils
{
public:
  int main();

private:
  static constexpr int allowedFrameDrops = 10; /**< Maximum number of frame drops allowed before Nao sits down. */
  static constexpr const char* socket_path = "/tmp/robocup";
  static constexpr size_t lola_sensor_size = 896;
  static constexpr std::chrono::milliseconds cycle_time = 12ms; /**< Expected cycle time. */
  static constexpr std::chrono::microseconds time_before_deadline = 4000us; /**< Send latest actuator data if the framework does not respond. */

  static const std::array<float, NDData::Joint::numOfJoints> sitDownAngles;
  static const std::array<float, NDData::Joint::numOfJoints> sitDownAnglesHighArms;
  static const std::array<float, NDData::Joint::numOfJoints> standHighAngles;

  /**
   * Unpacks msgpack buffer into sensor data.
   */
  void unpack_data(const std::vector<uint8_t>& buffer, NDData::SensorData& sensordata);

  /**
   * Packs request data into msgpack encoded buffer.
   */
  void pack_data(const NDData::ActuatorData& actuatordata, std::vector<uint8_t>& buffer);

  void processSensorData(const NDData::SensorData& sensordata);
  void processActuatorData(NDData::ActuatorData& actuatordata);

  void setEyeLeds(std::array<std::array<float, NDData::numOfLEyeLEDs>, NDData::numOfRGBLEDs>& leftEye, std::array<std::array<float, NDData::numOfREyeLEDs>, NDData::numOfRGBLEDs>& rightEye) const;
  void setSkullLeds(std::array<float, NDData::numOfSkullLEDs>& skull) const;
  void setChestLeds(std::array<float, NDData::numOfRGBLEDs>& chest) const;
  void modifyJoints(std::array<float, NDData::numOfJoints>& positions, std::array<float, NDData::numOfJoints>& stiffness);

  void stabilize(std::array<float, NDData::numOfJoints>& controlledPositions, std::array<float, NDData::numOfJoints>& controlledStiffnesses); /**< If robot is in sitting position, slight stabilization is applied. */

  void handleState(const NDData::SensorData& sensordata); /**< State transitions and actions for the bases State is handled. */
  void disableFrameworkWhenCharging();

  bool stiffnessRequested() const; /**< Checks for stiffness. Helper function for safe transitions. */
  bool isStandHigh() const; /**< Since our high stand uses no stiffness, check this for safe sitdown. */
  bool playDeadRequested() const;

  void waitFramework(std::chrono::system_clock::time_point deadline);
  void postFramework();

  void checkRead(bool success);

  static void sighandlerShutdown(int sig);
  void cleanUp();

  int fd = 0;
  NDData* data = reinterpret_cast<NDData*>(MAP_FAILED); /**< The shared memory. */
  sem_t* sem_sensors = SEM_FAILED; /**< The semaphore used to notify framework about new data. */
  sem_t* sem_actuators = SEM_FAILED; /**< The semaphore used to notify base about new data. */

  int actuatorMisses = 0; /**< The number of frames without seemingly new data from framework. */
  int sensorDrops = 0; /**< The number frames without a reaction from framework. */
  int lastSensorTimestamp = -1;

  const NDData::SensorData* lastSensorData = nullptr;
  const NDData::ActuatorData* lastActuatorData = nullptr;

  NDData::SteadyTimePoint ndevilsbaseStartTime;
  float blink = 0.f;

  //TODO: state names are misleading, since standing means framework active but can mean the robot is sitting.
  enum State
  {
    sitting,
    standingUp,
    standing,
    sittingDown
  } state = sitting;
  static bool shutDownRequested; /**< True, if chest button was pressed for 3 seconds. */
  bool chestButtonPressedAndReleased = false; // /**< Chest button was pressed previously and now released. */
  float phase = 0.f; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */

  std::array<float, NDData::numOfJoints> startAngles; /**< Start angles for standing up or sitting down. */

  bool stabilizationStatus = false; // If the robots is in sitDown position and stabilizing.
  bool chargingStatus = false; // If the robot is currently charging or not.
  std::array<bool, 2> handsClosed = {false, false};

  int startPressedTime = 0; /**< The last time the chest button was not pressed. */
  NDData::SteadyTimePoint lastNDevilsStartTime; /**< The last time the framework was started. */

  bool lastChargingStatus = false;

  bool chestButtonLastState = false; /**< Remember last chest button state. */
  int tripleHeadPressStartTime = 0; /**< After all three head buttons are pressed, this timestamp is set. */
  bool transitionTriggered = false; /**< Trigger transition to/from framework. */
};

bool NDevils::shutDownRequested = false;

const std::array<float, NDData::Joint::numOfJoints> NDevils::sitDownAngles = {
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

const std::array<float, NDData::Joint::numOfJoints> NDevils::sitDownAnglesHighArms = {
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

const std::array<float, NDData::Joint::numOfJoints> NDevils::standHighAngles = {
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
  std::cout << "Starting Nao Devils base." << std::endl;
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

  // lock memory and avoid page faults
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    perror("mlockall() failed");

  // create shared memory
  {
    int memoryHandle = shm_open(NDData::mem_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (memoryHandle == -1)
    {
      perror("shm_open failed");
      return 1;
    }

    if (ftruncate(memoryHandle, sizeof(NDData)) == -1)
    {
      perror("ftruncate failed");
      close(memoryHandle);
      cleanUp();
      return 2;
    }

    // map the shared memory
    void* mem = mmap(nullptr, sizeof(NDData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
    if (mem == MAP_FAILED)
    {
      perror("mmap failed");
      close(memoryHandle);
      cleanUp();
      return 3;
    }
    memset(mem, 0, sizeof(NDData));

    // after mmap, memory handle can be closed
    close(memoryHandle);

    // call constructor on allocated memory
    data = new (mem) NDData();
  }

  // open semaphore
  sem_sensors = sem_open(NDData::sem_name_sensors, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
  if (sem_sensors == SEM_FAILED)
  {
    perror("sem_open failed");
    cleanUp();
    return 4;
  }
  sem_actuators = sem_open(NDData::sem_name_actuators, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
  if (sem_actuators == SEM_FAILED)
  {
    perror("sem_open failed");
    cleanUp();
    return 5;
  }

  signal(SIGTERM, sighandlerShutdown);
  signal(SIGINT, sighandlerShutdown);

  if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)
  {
    perror("socket failed");
    cleanUp();
    return 6;
  }

  struct sockaddr_un addr;
  addr.sun_family = AF_UNIX;
  strcpy(addr.sun_path, socket_path);

  while (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    std::cerr << "Could not connect to socket!" << std::endl;
    sleep(3);

    if (shutDownRequested)
    {
      cleanUp();
      return 7;
    }
  }

  std::vector<uint8_t> recvbuffer, sendbuffer;
  recvbuffer.resize(lola_sensor_size);

  auto start = std::chrono::steady_clock::now();
  auto end = std::chrono::steady_clock::now();
  const auto checkTiming = [&](const char* message, const long long threshold)
  {
    end = std::chrono::steady_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (time > threshold)
      std::cerr << message << " took " << time << "ms!" << std::endl;
    start = end;
  };

  while (!shutDownRequested || state != sitting)
  {
    // this call blocks until LoLA provides new data
    ssize_t size = recv(fd, recvbuffer.data(), recvbuffer.size(), 0);
    auto nextDeadline = std::chrono::system_clock::now() + cycle_time - time_before_deadline;
    checkTiming("LoLA: receiving sensor data", cycle_time.count());

    if (size != lola_sensor_size)
    {
      std::cerr << "LoLA: package does not have correct size!" << std::endl;
      cleanUp();
      return 8;
    }

    // sensor data
    unpack_data(recvbuffer, data->sensors.writeBuffer());
    lastSensorData = &data->sensors.writeBuffer();
    data->sensors.finishWrite();
    postFramework();
    processSensorData(*lastSensorData);
    checkTiming("Base: processing sensor data", 0);

    // wait for framework
    waitFramework(nextDeadline);
    checkTiming("Base: waiting for framework", (cycle_time - time_before_deadline).count());

    // actuator data
    checkRead(data->actuators.beginRead());
    lastActuatorData = &data->actuators.readBuffer();
    NDData::ActuatorData actuatordata = data->actuators.readBuffer(); // make copy for modifications
    processActuatorData(actuatordata);
    pack_data(actuatordata, sendbuffer);
    checkTiming("Base: processing actuator data", 0);

    send(fd, sendbuffer.data(), sendbuffer.size(), 0);
    checkTiming("LoLA: sending actuator data", 0);
  }

  cleanUp();
  return 0;
}

void NDevils::checkRead(bool success)
{
  if (!success)
  {
    if (actuatorMisses == 0)
      std::cerr << "Framework: did not send actuator request!" << std::endl;
    ++actuatorMisses;
  }
  else
  {
    if (actuatorMisses > 0)
      std::cerr << "Framework: missed " << actuatorMisses << " actuator requests" << std::endl;
    actuatorMisses = 0;
  }
}


void NDevils::unpack_data(const std::vector<uint8_t>& buffer, NDData::SensorData& sensordata)
{
  sensordata.timestamp = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - ndevilsbaseStartTime).count());

  nlohmann::json json = nlohmann::json::from_msgpack(buffer);

  if (lastSensorTimestamp == -1)
  {
    std::cout << "LoLA: received first package!" << std::endl;

    strncpy(data->bodyId, json["RobotConfig"][0].get<std::string>().c_str(), 21);
    strncpy(data->headId, json["RobotConfig"][2].get<std::string>().c_str(), 21);

    std::cout << "My Head ID is: " << std::string(data->headId, 20) << std::endl;
    std::cout << "My Body ID is: " << std::string(data->bodyId, 20) << std::endl;
  }
  else
  {
    int timediff = sensordata.timestamp - lastSensorTimestamp;
    if (timediff > static_cast<int>(cycle_time.count() * 1.1f))
      std::cerr << "LoLA: time difference between sensor updates was " << timediff << "ms!" << std::endl;
  }
  lastSensorTimestamp = sensordata.timestamp;

  sensordata.acc = json["Accelerometer"];
  sensordata.angle = json["Angles"];
  sensordata.battery = json["Battery"];
  sensordata.current = json["Current"];
  sensordata.fsr = json["FSR"];
  sensordata.gyro = json["Gyroscope"];
  sensordata.position = json["Position"];
  sensordata.sonar = json["Sonar"];
  sensordata.status = json["Status"];
  sensordata.stiffness = json["Stiffness"];
  sensordata.temperature = json["Temperature"];
  sensordata.touch = json["Touch"];


  // If lying on front, ignore chest button data. This prevents presses due to ground contact.
  bool chestButtonPressed = !(std::abs(sensordata.angle[0]) * 60 < 15 && sensordata.angle[1] * 60 > 70) && sensordata.touch[NDData::Touch::chestButton] == 1.f;

  if (chestButtonPressed)
    sensordata.touch[NDData::Touch::chestButton] = 1.f;
  else
    sensordata.touch[NDData::Touch::chestButton] = 0.f;
}

void NDevils::processSensorData(const NDData::SensorData& sensordata)
{
  // detect charging status
  chargingStatus = !(static_cast<int>(sensordata.battery[NDData::Battery::status]) & 0b100000);

  const bool chestButtonPressed = sensordata.touch[NDData::Touch::chestButton] == 1.f;

  // detect transition or shutdown request via chest-button
  if (!chestButtonPressed)
    startPressedTime = sensordata.timestamp;
  else if (!shutDownRequested && startPressedTime && (sensordata.timestamp - startPressedTime) > 3000)
  {
    std::cout << "Shutdown requested, startPressedTime = " << startPressedTime << ", sensorTimestamp = " << sensordata.timestamp << std::endl;
    (void)!system("sudo poweroff &");
    startPressedTime = 0;
  }

  // detect triple press of chestbutton for transition from framework
  chestButtonPressedAndReleased = chestButtonLastState != chestButtonPressed && !chestButtonPressed;
  chestButtonLastState = chestButtonPressed;

  bool isTripleHeadPressed = sensordata.touch[NDData::Touch::headFront] == 1.f && sensordata.touch[NDData::Touch::headMiddle] == 1.f && sensordata.touch[NDData::Touch::headFront] == 1.f;

  // detect triple head button press (all three head buttons pressed at the same time)
  // if we hold those buttons for > 1 sec, tripleHeadPressed is set to true
  if (!isTripleHeadPressed || transitionTriggered)
    tripleHeadPressStartTime = sensordata.timestamp;

  transitionTriggered = sensordata.timestamp - tripleHeadPressStartTime > 1000 && !(chargingStatus && state == sitting);


  disableFrameworkWhenCharging();

  handleState(sensordata);
}

void NDevils::postFramework()
{
  // reset actuators semaphore
  while (sem_trywait(sem_actuators) == 0)
    ;

  // raise the sensors semaphore
  if (sem_sensors != SEM_FAILED)
  {
    int sval;
    if (sem_getvalue(sem_sensors, &sval) == 0)
    {
      if (sval >= 1)
      {
        if (sensorDrops == 0)
          std::cerr << "Framework: did not receive sensor data!" << std::endl;
        ++sensorDrops;
      }
      else
      {
        sem_post(sem_sensors);
        if (sensorDrops > 0)
          std::cerr << "Framework: dropped " << sensorDrops << " sensor data" << std::endl;
        sensorDrops = 0;
      }
    }
  }
}

void NDevils::waitFramework(std::chrono::system_clock::time_point deadline)
{
  auto secs = std::chrono::time_point_cast<std::chrono::seconds>(deadline);
  auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(deadline) - std::chrono::time_point_cast<std::chrono::nanoseconds>(secs);

  timespec ts{secs.time_since_epoch().count(), ns.count()};
  if (sem_timedwait(sem_actuators, &ts) == -1 && errno != ETIMEDOUT)
  {
    std::cerr << "Framework: ";
    perror("wait interrupted");
  }
}

void NDevils::pack_data(const NDData::ActuatorData& actuatordata, std::vector<uint8_t>& buffer)
{
  nlohmann::json json;

  static constexpr size_t lEyeSize = NDData::ActuatorData().lEyeLEDs.size() * NDData::ActuatorData().lEyeLEDs[0].size();
  static constexpr size_t rEyeSize = NDData::ActuatorData().rEyeLEDs.size() * NDData::ActuatorData().rEyeLEDs[0].size();

  json["Chest"] = actuatordata.chestLEDs;
  json["LEar"] = actuatordata.lEarLEDs;
  json["LEye"] = reinterpret_cast<const std::array<float, lEyeSize>&>(actuatordata.lEyeLEDs); // 1d array needed
  json["LFoot"] = actuatordata.lFootLEDs;
  json["Position"] = actuatordata.positions;
  json["REar"] = actuatordata.rEarLEDs;
  json["REye"] = reinterpret_cast<const std::array<float, rEyeSize>&>(actuatordata.rEyeLEDs); // 1d array needed
  json["RFoot"] = actuatordata.rFootLEDs;
  json["Skull"] = actuatordata.skullLEDs;
  json["Stiffness"] = actuatordata.stiffness;
  json["Sonar"] = actuatordata.sonars;

  buffer = nlohmann::json::to_msgpack(json);
}

void NDevils::processActuatorData(NDData::ActuatorData& actuatordata)
{
  // set blink float for synchronous LED blinking :-)
  auto systemtime = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
  blink = static_cast<float>((systemtime.time_since_epoch().count() / 500) & 1);

  modifyJoints(actuatordata.positions, actuatordata.stiffness);
  setEyeLeds(actuatordata.lEyeLEDs, actuatordata.rEyeLEDs);
  setSkullLeds(actuatordata.skullLEDs);
  setChestLeds(actuatordata.chestLEDs);
}

/**
  * Set the eye LEDs based on the current state.
  * Shutting down -> Lower segments are red.
  * framework crashed -> Whole eyes alternately flash red.
  * framework not running -> Lower segments flash blue.
  * @param actuators The actuator values a part of which will be set by this method.
  */
void NDevils::setEyeLeds(std::array<std::array<float, NDData::numOfLEyeLEDs>, NDData::numOfRGBLEDs>& leftEye, std::array<std::array<float, NDData::numOfREyeLEDs>, NDData::numOfRGBLEDs>& rightEye) const
{
  if (!shutDownRequested && data->state == NDData::okState && sensorDrops < allowedFrameDrops)
    return;

  for (auto& color : leftEye)
    color.fill(0.f);
  for (auto& color : rightEye)
    color.fill(0.f);

  if (shutDownRequested)
  {
    leftEye[NDData::RGBLED::r][NDData::LEyeLED::lEyeDeg180] = 1.f;
    rightEye[NDData::RGBLED::r][NDData::REyeLED::rEyeDeg180] = 1.f;
  }
  else if (data->state != NDData::okState && data->state != NDData::State::sigTERMState)
  {
    // set the "ndevilsbase is active and framework crashed" leds
    leftEye[NDData::RGBLED::r].fill(blink);
    rightEye[NDData::RGBLED::r].fill(blink);
  }
  else if (sensorDrops >= allowedFrameDrops || data->state == NDData::State::sigTERMState)
  {
    // set the "ndevilsbase is active and framework is not running" LEDs
    leftEye[NDData::RGBLED::b][NDData::LEyeLED::lEyeDeg180] = blink;
    rightEye[NDData::RGBLED::b][NDData::REyeLED::rEyeDeg180] = blink;
  }
}


/**
* Shows the battery state in the right ear if the robot is in the standing state
* and framework has not used one of these LEDs in the past 5 seconds.
* @param actuators The actuator values a part of which will be set by this method.
*/
void NDevils::setSkullLeds(std::array<float, NDData::numOfSkullLEDs>& skull) const
{
  if (chargingStatus)
  {
    skull[NDData::SkullLED::rearLeft0] = 1.f - blink;
    skull[NDData::SkullLED::rearLeft1] = 1.f - blink;
    skull[NDData::SkullLED::rearLeft2] = 1.f - blink;
    skull[NDData::SkullLED::rearRight0] = blink;
    skull[NDData::SkullLED::rearRight1] = blink;
    skull[NDData::SkullLED::rearRight2] = blink;
    skull[NDData::SkullLED::middleRight0] = blink;
    skull[NDData::SkullLED::frontRight0] = blink;
    skull[NDData::SkullLED::frontRight1] = blink;
    skull[NDData::SkullLED::frontLeft0] = 1.f - blink;
    skull[NDData::SkullLED::frontLeft1] = 1.f - blink;
    skull[NDData::SkullLED::middleLeft0] = 1.f - blink;
  }

  if (stabilizationStatus)
  {
    for (float& led : skull)
      if (led == 0.f)
        led = 0.1f;
  }
}

/**
  * Shows the transitionToFramework state with the chest LEDs
  * @param actuators The actuator values a part of which will be set by this method.
  */
void NDevils::setChestLeds(std::array<float, NDData::numOfRGBLEDs>& chest) const
{
  if (state == sitting || state == sittingDown)
  {
    chest[NDData::RGBLED::r] = 0.f;
    chest[NDData::RGBLED::g] = 0.f;
    chest[NDData::RGBLED::b] = blink;
  }
}

/**
  * Stabilize robot if in sitDown position.
  */
void NDevils::stabilize(std::array<float, NDData::numOfJoints>& positions, std::array<float, NDData::numOfJoints>& stiffness)
{
  const NDData::SensorData& sensordata = *lastSensorData;

  stabilizationStatus = true;

  // check if robot is sitting correctly and stabilization should be enabled
  for (size_t i = NDData::Joint::lHipYawPitch; i <= NDData::Joint::rAnkleRoll; ++i)
  {
    // don't stabilize if joint is not close enough to sitDown position
    if (std::abs(sensordata.position[i] - sitDownAngles[i]) > 0.3f)
    {
      stabilizationStatus = false;
      break;
    }
  }

  // set joints to sitDown position
  if (stabilizationStatus)
  {
    // stabilization
    positions[NDData::Joint::lHipYawPitch] = sitDownAngles[NDData::Joint::lHipYawPitch];
    stiffness[NDData::Joint::lHipYawPitch] = 0.1f;
    positions[NDData::Joint::lHipPitch] = sitDownAngles[NDData::Joint::lHipPitch];
    stiffness[NDData::Joint::lHipPitch] = 0.1f;
    positions[NDData::Joint::rHipPitch] = sitDownAngles[NDData::Joint::rHipPitch];
    stiffness[NDData::Joint::rHipPitch] = 0.1f;

    // close hands if opened
    if (std::abs(sensordata.position[NDData::Joint::lHand] - sitDownAngles[NDData::Joint::lHand]) > 0.25f)
      handsClosed[0] = false;
    if (std::abs(sensordata.position[NDData::Joint::rHand] - sitDownAngles[NDData::Joint::rHand]) > 0.25f)
      handsClosed[1] = false;
    // skip closing hands if closed
    if (std::abs(sensordata.position[NDData::Joint::lHand] - sitDownAngles[NDData::Joint::lHand]) < 0.05f)
      handsClosed[0] = true;
    if (std::abs(sensordata.position[NDData::Joint::rHand] - sitDownAngles[NDData::Joint::rHand]) < 0.05f)
      handsClosed[1] = true;

    if (handsClosed[0] == false)
    {
      positions[NDData::Joint::lHand] = sitDownAngles[NDData::Joint::lHand];
      stiffness[NDData::Joint::lHand] = 0.4f;
    }
    if (handsClosed[1] == false)
    {
      positions[NDData::Joint::rHand] = sitDownAngles[NDData::Joint::rHand];
      stiffness[NDData::Joint::rHand] = 0.4f;
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
void NDevils::handleState(const NDData::SensorData& sensordata)
{
  // too many frame drops-> sit down
  // framework stopped or crashed -> sit down
  if ((sensorDrops > allowedFrameDrops || data->state != NDData::okState) && state != sitting && state != sittingDown)
  {
    std::cout << "Framework: stopped, crashed or too many frame drops" << std::endl;

    if (playDeadRequested())
    {
      std::cout << "switching to state: sitting" << std::endl;
      state = sitting;
    }
    else
    {
      std::cout << "switching to state: sittingDown" << std::endl;
      state = sittingDown;

      // reset start angles
      startAngles = sensordata.position;
      phase = 0.f;
    }
  }

  // state transitions
  switch (state)
  {
  case sitting:
  {
    bool bodyUpright = std::abs(sensordata.angle[0]) < 0.2f && (sensordata.angle[1]) < 0.2f;
    if (!shutDownRequested && sensorDrops <= allowedFrameDrops && (transitionTriggered || chestButtonPressedAndReleased) && bodyUpright)
    {
      std::cout << "switching to state: standingUp" << std::endl;
      state = standingUp;

      // reset start angles
      startAngles = sensordata.position;
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
    if (transitionTriggered || shutDownRequested)
    {
      if (playDeadRequested())
      {
        std::cout << "switching to state: sitting" << std::endl;
        state = sitting;
      }
      else
      {
        std::cout << "switching to state: sittingDown" << std::endl;
        state = sittingDown;

        // reset start angles
        startAngles = sensordata.position;
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
}

void NDevils::modifyJoints(std::array<float, NDData::numOfJoints>& positions, std::array<float, NDData::numOfJoints>& stiffness)
{
  // action in states
  switch (state)
  {
  case sitting:
  {
    positions.fill(0.f);
    stiffness.fill(0.f);

    data->transitionToFramework.store(0.f, std::memory_order_relaxed);

    break;
  }
  case standingUp:
  {
    phase = std::min(phase + 0.005f, 1.f);
    data->transitionToFramework.store(phase, std::memory_order_relaxed);
    if (playDeadRequested())
      break;
    float sinPhase = (std::cos(phase * 3.1416f) - 1.f) / -2.f;
    for (size_t i = 0; i < NDData::Joint::numOfJoints; ++i)
    {
      positions[i] = positions[i] * sinPhase + startAngles[i] * (1.f - sinPhase);
      float h = std::max(stiffness[i], 0.5f);
      stiffness[i] = stiffness[i] * sinPhase + h * (1.f - sinPhase);
    }
    break;
  }
  case standing:
  {
    // Default case, do nothing and forward framework data as it is.
    break;
  }
  case sittingDown:
  {
    phase = std::min(phase + 0.005f, 1.f);
    data->transitionToFramework.store(1.f - phase, std::memory_order_relaxed);
    float sinPhase = (std::cos(phase * 3.1416f) - 1.f) / -2.f;
    if (phase < 0.75f)
      sinPhase = (std::cos(phase / 0.75f * 3.1416f) - 1.f) / -2.f;
    else
      sinPhase = (std::cos((phase - 0.75f) / 0.25f * 3.1416f) - 1.f) / -2.f;
    for (size_t i = 0; i < NDData::Joint::numOfJoints; ++i)
    {
      if (phase < 0.75f)
        positions[i] = startAngles[i] * (1.f - sinPhase) + sitDownAnglesHighArms[i] * sinPhase;
      else
        positions[i] = sitDownAnglesHighArms[i] * (1.f - sinPhase) + sitDownAngles[i] * sinPhase;
      stiffness[i] = 0.5f; // TODO: test
    }
    break;
  }
  }

  // stabilize if robot has no stiffness
  if (state == NDevils::State::sitting || playDeadRequested())
    stabilize(positions, stiffness);
  else
    stabilizationStatus = false;
}

/** This method disables the framework if the robot has no stiffness and starts charging. */
void NDevils::disableFrameworkWhenCharging()
{
  if (state == standing && playDeadRequested() && chargingStatus && !lastChargingStatus)
    transitionTriggered = true;

  lastChargingStatus = chargingStatus;
}

/** This method checks if the given actuator request has stiffness > 0 */
bool NDevils::stiffnessRequested() const
{
  const auto& stiffness = lastActuatorData->stiffness;

  for (const float joint : stiffness)
    if (joint > 0.f)
      return true;

  return false;
}

/** This method checks if the robot is in stand high pose */
bool NDevils::isStandHigh() const
{
  const auto& positions = lastSensorData->position;

  for (size_t i = NDData::Joint::lHipYawPitch; i <= NDData::Joint::rAnkleRoll; ++i)
  {
    if (std::abs(positions[i] - standHighAngles[i]) > 0.2f)
    {
      return false;
    }
  }

  // check if body is upright
  const auto& angle = lastSensorData->angle;
  if (std::abs(angle[0]) > 0.2f || std::abs(angle[1]) > 0.2f)
    return false;

  return true;
}

/** This method checks if the robot is in play dead */
bool NDevils::playDeadRequested() const
{
  if (isStandHigh())
    return false;

  if (stiffnessRequested())
    return false;

  return true;
}

void NDevils::sighandlerShutdown(int sig)
{
  if (!shutDownRequested)
    printf("ndevilsbase: Caught signal %i\nShutting down...\n", sig);
  shutDownRequested = true;
}

void NDevils::cleanUp()
{
  if (fd)
    close(fd);
  if (sem_sensors != SEM_FAILED)
  {
    sem_close(sem_sensors);
    sem_unlink(NDData::sem_name_sensors);
  }
  if (sem_actuators != SEM_FAILED)
  {
    sem_close(sem_actuators);
    sem_unlink(NDData::sem_name_actuators);
  }
  if (data != MAP_FAILED)
    munmap(data, sizeof(NDData));

  shm_unlink(NDData::mem_name);

  std::cerr << "Stopped." << std::endl;
}
