/**
 * @file bhuman.cpp
 * Implementation of a NaoQi module that provides basic ipc NaoQi DCM access via semaphore and shared memory.
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <csignal>
#include <sys/resource.h>
#include <ctime>
#include <cstring>

#ifdef __clang__
#pragma clang diagnostic push

#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunused-local-typedef"
#endif
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <alcommon/albroker.h>
#include <alcommon/alproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#undef BOOST_SIGNALS_NO_DEPRECATION_WARNING
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "bhuman.h"

static const char* sensorNames[] =
{
  // joint sensors
  "Device/SubDeviceList/HeadYaw/Position/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Position/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/HeadPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/Position/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LWristYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHand/Position/Sensor/Value",
  "Device/SubDeviceList/LHand/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHand/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RShoulderRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RElbowRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/Position/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RWristYaw/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHand/Position/Sensor/Value",
  "Device/SubDeviceList/RHand/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHand/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipYawPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/LAnkleRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipRoll/Temperature/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RHipPitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RKneePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnklePitch/Temperature/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value",
  "Device/SubDeviceList/RAnkleRoll/Temperature/Sensor/Value",
  
  // touch sensors
  "Device/SubDeviceList/Head/Touch/Front/Sensor/Value",
  "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value",
  "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value",
  "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value",
  "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value",

  // switches
  "Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value",
  "Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value",
  "Device/SubDeviceList/ChestBoard/Button/Sensor/Value",

  // inertial sensors
  "Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
  "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",

  // sonar sensors
  "Device/SubDeviceList/US/Left/Sensor/Value",
  "Device/SubDeviceList/US/Left/Sensor/Value1",
  "Device/SubDeviceList/US/Left/Sensor/Value2",
  "Device/SubDeviceList/US/Left/Sensor/Value3",
  "Device/SubDeviceList/US/Left/Sensor/Value4",
  "Device/SubDeviceList/US/Left/Sensor/Value5",
  "Device/SubDeviceList/US/Left/Sensor/Value6",
  "Device/SubDeviceList/US/Left/Sensor/Value7",
  "Device/SubDeviceList/US/Left/Sensor/Value8",
  "Device/SubDeviceList/US/Left/Sensor/Value9",
  "Device/SubDeviceList/US/Right/Sensor/Value",
  "Device/SubDeviceList/US/Right/Sensor/Value1",
  "Device/SubDeviceList/US/Right/Sensor/Value2",
  "Device/SubDeviceList/US/Right/Sensor/Value3",
  "Device/SubDeviceList/US/Right/Sensor/Value4",
  "Device/SubDeviceList/US/Right/Sensor/Value5",
  "Device/SubDeviceList/US/Right/Sensor/Value6",
  "Device/SubDeviceList/US/Right/Sensor/Value7",
  "Device/SubDeviceList/US/Right/Sensor/Value8",
  "Device/SubDeviceList/US/Right/Sensor/Value9",

  // battery sensors
  "Device/SubDeviceList/Battery/Current/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Value",
  "Device/SubDeviceList/Battery/Charge/Sensor/Status",
  "Device/SubDeviceList/Battery/Temperature/Sensor/Value",
  "Device/SubDeviceList/Battery/Temperature/Sensor/Status",
  
  // fsr sensors
  "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value",
  "Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value",
  "Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value",
};

static const char* actuatorNames[] =
{
  "HeadYaw/Position/Actuator/Value",
  "HeadPitch/Position/Actuator/Value",
  "LShoulderPitch/Position/Actuator/Value",
  "LShoulderRoll/Position/Actuator/Value",
  "LElbowYaw/Position/Actuator/Value",
  "LElbowRoll/Position/Actuator/Value",
  "LWristYaw/Position/Actuator/Value",
  "LHand/Position/Actuator/Value",
  "RShoulderPitch/Position/Actuator/Value",
  "RShoulderRoll/Position/Actuator/Value",
  "RElbowYaw/Position/Actuator/Value",
  "RElbowRoll/Position/Actuator/Value",
  "RWristYaw/Position/Actuator/Value",
  "RHand/Position/Actuator/Value",
  "LHipYawPitch/Position/Actuator/Value",
  "LHipRoll/Position/Actuator/Value",
  "LHipPitch/Position/Actuator/Value",
  "LKneePitch/Position/Actuator/Value",
  "LAnklePitch/Position/Actuator/Value",
  "LAnkleRoll/Position/Actuator/Value",
  "RHipRoll/Position/Actuator/Value",
  "RHipPitch/Position/Actuator/Value",
  "RKneePitch/Position/Actuator/Value",
  "RAnklePitch/Position/Actuator/Value",
  "RAnkleRoll/Position/Actuator/Value",

  "HeadYaw/Hardness/Actuator/Value",
  "HeadPitch/Hardness/Actuator/Value",
  "LShoulderPitch/Hardness/Actuator/Value",
  "LShoulderRoll/Hardness/Actuator/Value",
  "LElbowYaw/Hardness/Actuator/Value",
  "LElbowRoll/Hardness/Actuator/Value",
  "LWristYaw/Hardness/Actuator/Value",
  "LHand/Hardness/Actuator/Value",
  "RShoulderPitch/Hardness/Actuator/Value",
  "RShoulderRoll/Hardness/Actuator/Value",
  "RElbowYaw/Hardness/Actuator/Value",
  "RElbowRoll/Hardness/Actuator/Value",
  "RWristYaw/Hardness/Actuator/Value",
  "RHand/Hardness/Actuator/Value",
  "LHipYawPitch/Hardness/Actuator/Value",
  "LHipRoll/Hardness/Actuator/Value",
  "LHipPitch/Hardness/Actuator/Value",
  "LKneePitch/Hardness/Actuator/Value",
  "LAnklePitch/Hardness/Actuator/Value",
  "LAnkleRoll/Hardness/Actuator/Value",
  "RHipRoll/Hardness/Actuator/Value",
  "RHipPitch/Hardness/Actuator/Value",
  "RKneePitch/Hardness/Actuator/Value",
  "RAnklePitch/Hardness/Actuator/Value",
  "RAnkleRoll/Hardness/Actuator/Value",

  "Face/Led/Red/Left/0Deg/Actuator/Value",
  "Face/Led/Red/Left/45Deg/Actuator/Value",
  "Face/Led/Red/Left/90Deg/Actuator/Value",
  "Face/Led/Red/Left/135Deg/Actuator/Value",
  "Face/Led/Red/Left/180Deg/Actuator/Value",
  "Face/Led/Red/Left/225Deg/Actuator/Value",
  "Face/Led/Red/Left/270Deg/Actuator/Value",
  "Face/Led/Red/Left/315Deg/Actuator/Value",
  "Face/Led/Green/Left/0Deg/Actuator/Value",
  "Face/Led/Green/Left/45Deg/Actuator/Value",
  "Face/Led/Green/Left/90Deg/Actuator/Value",
  "Face/Led/Green/Left/135Deg/Actuator/Value",
  "Face/Led/Green/Left/180Deg/Actuator/Value",
  "Face/Led/Green/Left/225Deg/Actuator/Value",
  "Face/Led/Green/Left/270Deg/Actuator/Value",
  "Face/Led/Green/Left/315Deg/Actuator/Value",
  "Face/Led/Blue/Left/0Deg/Actuator/Value",
  "Face/Led/Blue/Left/45Deg/Actuator/Value",
  "Face/Led/Blue/Left/90Deg/Actuator/Value",
  "Face/Led/Blue/Left/135Deg/Actuator/Value",
  "Face/Led/Blue/Left/180Deg/Actuator/Value",
  "Face/Led/Blue/Left/225Deg/Actuator/Value",
  "Face/Led/Blue/Left/270Deg/Actuator/Value",
  "Face/Led/Blue/Left/315Deg/Actuator/Value",
  "Face/Led/Red/Right/0Deg/Actuator/Value",
  "Face/Led/Red/Right/45Deg/Actuator/Value",
  "Face/Led/Red/Right/90Deg/Actuator/Value",
  "Face/Led/Red/Right/135Deg/Actuator/Value",
  "Face/Led/Red/Right/180Deg/Actuator/Value",
  "Face/Led/Red/Right/225Deg/Actuator/Value",
  "Face/Led/Red/Right/270Deg/Actuator/Value",
  "Face/Led/Red/Right/315Deg/Actuator/Value",
  "Face/Led/Green/Right/0Deg/Actuator/Value",
  "Face/Led/Green/Right/45Deg/Actuator/Value",
  "Face/Led/Green/Right/90Deg/Actuator/Value",
  "Face/Led/Green/Right/135Deg/Actuator/Value",
  "Face/Led/Green/Right/180Deg/Actuator/Value",
  "Face/Led/Green/Right/225Deg/Actuator/Value",
  "Face/Led/Green/Right/270Deg/Actuator/Value",
  "Face/Led/Green/Right/315Deg/Actuator/Value",
  "Face/Led/Blue/Right/0Deg/Actuator/Value",
  "Face/Led/Blue/Right/45Deg/Actuator/Value",
  "Face/Led/Blue/Right/90Deg/Actuator/Value",
  "Face/Led/Blue/Right/135Deg/Actuator/Value",
  "Face/Led/Blue/Right/180Deg/Actuator/Value",
  "Face/Led/Blue/Right/225Deg/Actuator/Value",
  "Face/Led/Blue/Right/270Deg/Actuator/Value",
  "Face/Led/Blue/Right/315Deg/Actuator/Value",
  "Ears/Led/Left/36Deg/Actuator/Value",
  "Ears/Led/Left/72Deg/Actuator/Value",
  "Ears/Led/Left/108Deg/Actuator/Value",
  "Ears/Led/Left/144Deg/Actuator/Value",
  "Ears/Led/Left/180Deg/Actuator/Value",
  "Ears/Led/Left/216Deg/Actuator/Value",
  "Ears/Led/Left/252Deg/Actuator/Value",
  "Ears/Led/Left/288Deg/Actuator/Value",
  "Ears/Led/Left/324Deg/Actuator/Value",
  "Ears/Led/Left/0Deg/Actuator/Value",
  "Ears/Led/Right/0Deg/Actuator/Value",
  "Ears/Led/Right/36Deg/Actuator/Value",
  "Ears/Led/Right/72Deg/Actuator/Value",
  "Ears/Led/Right/108Deg/Actuator/Value",
  "Ears/Led/Right/144Deg/Actuator/Value",
  "Ears/Led/Right/180Deg/Actuator/Value",
  "Ears/Led/Right/216Deg/Actuator/Value",
  "Ears/Led/Right/252Deg/Actuator/Value",
  "Ears/Led/Right/288Deg/Actuator/Value",
  "Ears/Led/Right/324Deg/Actuator/Value",
  "ChestBoard/Led/Red/Actuator/Value",
  "ChestBoard/Led/Green/Actuator/Value",
  "ChestBoard/Led/Blue/Actuator/Value", 
  "Head/Led/Rear/Left/0/Actuator/Value",
  "Head/Led/Rear/Left/1/Actuator/Value",
  "Head/Led/Rear/Left/2/Actuator/Value",
  "Head/Led/Rear/Right/0/Actuator/Value",
  "Head/Led/Rear/Right/1/Actuator/Value",
  "Head/Led/Rear/Right/2/Actuator/Value",
  "Head/Led/Middle/Right/0/Actuator/Value",
  "Head/Led/Front/Right/0/Actuator/Value",
  "Head/Led/Front/Right/1/Actuator/Value",
  "Head/Led/Front/Left/0/Actuator/Value",
  "Head/Led/Front/Left/1/Actuator/Value",
  "Head/Led/Middle/Left/0/Actuator/Value",
  "LFoot/Led/Red/Actuator/Value",
  "LFoot/Led/Green/Actuator/Value",
  "LFoot/Led/Blue/Actuator/Value",
  "RFoot/Led/Red/Actuator/Value",
  "RFoot/Led/Green/Actuator/Value",
  "RFoot/Led/Blue/Actuator/Value",

  "US/Actuator/Value"
};

static const char* teamInfoNames[] =
{
  "GameCtrl/teamNumber",
  "GameCtrl/teamColor",
  "GameCtrl/playerNumber"
};

static const float sitDownAngles[25] =
{
  0.f,
  0.f,

  0.89f,
  0.06f,
  0.26f,
  -0.62f,
  -1.57f,
  0.f,

  0.89f,
  -0.06f,
  -0.26f,
  0.62f,
  1.57f,
  0.f,

  0.f,
  0.f,
  -0.855f,
  2.16f,
  -1.18f,
  0.f,

  0.f,
  -0.855f,
  2.16f,
  -1.18f,
  0.f
};

static const float sitDownAnglesHighArms[25] =
{
  0.f,
  0.f,

  0.712f,
  0.164f,
  0.080f,
  -0.045f,
  -1.57f,
  0.f,

  0.712f,
  -0.164f,
  -0.080f,
  0.045f,
  1.57f,
  0.f,

  0.f,
  0.f,
  -0.855f,
  2.16f,
  -1.18f,
  0.f,

  0.f,
  -0.855f,
  2.16f,
  -1.18f,
  0.f
};

static const float standHighAngles[25] =
{
  0.f,
  0.349f,
  
  1.571f,
  0.201f,
  1.571f,
  0.f,
  -1.571f,
  0.f,
  
  1.571f,
  -0.201f,
  -1.571f,
  0.f,
  1.571f,
  0.f,
  
  0.f,
  -0.035f,
  0.157f,
  -0.087f,
  0.035f,
  0.f,
  
  -0.035f,
  0.157f,
  -0.087f,
  0.035f,
  0.f
};

class BHuman : public AL::ALModule
{
private:
  static BHuman* theInstance; /**< The only instance of this class. */

#ifdef NDEBUG
  static const int allowedFrameDrops = 3; /**< Maximum number of frame drops allowed before Nao sits down. */
#else
  static const int allowedFrameDrops = 6; /**< Maximum number of frame drops allowed before Nao sits down. */
#endif

  int memoryHandle; /**< The file handle of the shared memory. */
  LBHData* data; /**< The shared memory. */
  sem_t* sem; /**< The semaphore used to notify bhuman about new data. */
  AL::DCMProxy* proxy;
  AL::ALMemoryProxy* memory;
  AL::ALValue positionRequest;
  AL::ALValue stiffnessRequest;
  AL::ALValue usRequest;
  AL::ALValue ledRequest;
  float* sensorPtrs[lbhNumOfSensorIds]; /** Pointers to where NaoQi stores the current sensor values. */

  int dcmTime; /**< Current dcm time, updated at each onPreProcess call. */

  float requestedActuators[lbhNumOfActuatorIds]; /**< The previous actuator values requested. */

  int lastReadingActuators; /**< The previous actuators read. For detecting frames without seemingly new data from bhuman. */
  int actuatorDrops; /**< The number of frames without seemingly new data from bhuman. */
  int frameDrops; /**< The number frames without a reaction from bhuman. */

  enum State {sitting, standingUp, standing, sittingDown, preShuttingDown, preShuttingDownWhileSitting, shuttingDown} state;
  float phase; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */
  int ledIndex; /**< The index of the last LED set. */

  int rightEarLEDsChangedTime; // Last time when the right ear LEDs were changed by the B-Human code

  float startAngles[lbhNumOfPositionActuatorIds]; /**< Start angles for standing up or sitting down. */
  float startStiffness[lbhNumOfPositionActuatorIds]; /**< Start stiffness for sitting down. */

  bool stabilizationStatus; // If the robots is in sitDown position and stabilizing
  bool chargingStatus; // If the robot is currently charging or not

  int startPressedTime; /**< The last time the chest button was not pressed. */
  uint64_t lastBHumanStartTime; /**< The last time bhuman was started. */

  int triplePressStart;
  int triplePressCount;
  bool triplePressLastButtonState;
  bool triplePressSitDownRequested;

  /** Close all resources acquired. Called when initialization failed or during destruction. */
  void close()
  {
    fprintf(stderr, "libbhuman: Stopping.\n");

    if(proxy)
    {
      proxy->getGenericProxy()->getModule()->removeAllPreProcess();
      proxy->getGenericProxy()->getModule()->removeAllPostProcess();
      delete proxy;
    }
    if(memory)
      delete memory;
    if(sem != SEM_FAILED)
      sem_close(sem);
    if(data != MAP_FAILED)
      munmap(data, sizeof(LBHData));

    fprintf(stderr, "libbhuman: Stopped.\n");
  }

  /**
   * Set the eye LEDs based on the current state.
   * Shutting down -> Lower segments are red.
   * bhuman crashed -> Whole eyes alternately flash red.
   * bhuman not running -> Lower segments flash blue.
   * @param actuators The actuator values a part of which will be set by this method.
   */
  void setEyeLeds(float* actuators)
  {
    for(int i = faceLedRedLeft0DegActuator; i <= faceLedBlueRight315DegActuator; ++i)
      actuators[i] = 0.f;

    if(state == shuttingDown)
    {
      actuators[faceLedRedLeft180DegActuator] = 1.f;
      actuators[faceLedRedRight180DegActuator] = 1.f;
    }
    else if(data->state != okState)
    {
      // set the "libbhuman is active and bhuman crashed" leds
      float blink = float(dcmTime / 500 & 1);
      for(int i = faceLedRedLeft0DegActuator; i <= faceLedRedLeft315DegActuator; ++i)
        actuators[i] = blink;
      for(int i = faceLedRedRight0DegActuator; i <= faceLedRedRight315DegActuator; ++i)
        actuators[i] = 1.f - blink;
    }
    else
    {
      // set the "libbhuman is active and bhuman is not running" LEDs
      float blink = float(dcmTime / 500 & 1);
      actuators[faceLedBlueLeft180DegActuator] = blink;
      actuators[faceLedBlueRight180DegActuator] = blink;
    }
  }

  /**
  * Shows the battery state in the right ear if the robot is in the standing state
  * and bhuman has not used one of these LEDs in the past 5 seconds.
  * @param actuators The actuator values a part of which will be set by this method.
  */
  void setBatteryLeds(float* actuators)
  {
    for (int i = earsLedRight0DegActuator; i <= earsLedRight324DegActuator; ++i)
      if (actuators[i] != requestedActuators[i])
      {
        rightEarLEDsChangedTime = dcmTime;
        requestedActuators[i] = actuators[i];
      }

    if (state != standing || dcmTime - rightEarLEDsChangedTime > 5000)
      for (int i = 0; i < int(*sensorPtrs[batteryChargeSensor] * 10.f) && i < 10; ++i)
        actuators[earsLedRight0DegActuator + i] = 1.f;

    if(chargingStatus) {
      float blink = float(dcmTime / 500 & 1);
      actuators[headLedRearLeft0Actuator] = 1.f - blink;
      actuators[headLedRearLeft1Actuator] = 1.f - blink;
      actuators[headLedRearLeft2Actuator] = 1.f - blink;
      actuators[headLedRearRight0Actuator] = blink;
      actuators[headLedRearRight1Actuator] = blink;
      actuators[headLedRearRight2Actuator] = blink;
      actuators[headLedMiddleRight0Actuator] = blink;
      actuators[headLedFrontRight0Actuator] = blink;
      actuators[headLedFrontRight1Actuator] = blink;
      actuators[headLedFrontLeft0Actuator] = 1.f - blink;
      actuators[headLedFrontLeft1Actuator] = 1.f - blink;
      actuators[headLedMiddleLeft0Actuator] = 1.f - blink;
    }
  }

  /**
   * Shows the stabilization state with the head LEDs
   * @param actuators The actuator values a part of which will be set by this method.
   */
  void setStabilizationLeds(float* actuators)
  {
    if (stabilizationStatus)
    {
      for (int i = headLedRearLeft0Actuator; i <= headLedMiddleLeft0Actuator; i++)
        if(actuators[i] == 0.f) actuators[i] = 0.1f;
    }

    if (state == sitting || triplePressSitDownRequested)
    {
      actuators[chestBoardLedRedActuator] = 0.f;
      actuators[chestBoardLedGreenActuator] = 0.f;
      actuators[chestBoardLedBlueActuator] = float(dcmTime / 500 & 1);
    }
  }

  /**
   * Copies everything that's not for servos from one set of actuator values to another.
   * @param srcActuators The actuator values from which is copied.
   * @param destActuators The actuator values to which is copied.
   */
  void copyNonServos(const float* srcActuators, float* destActuators)
  {
    for(int i = faceLedRedLeft0DegActuator; i < lbhNumOfActuatorIds; ++i)
      destActuators[i] = srcActuators[i];
  }

  /** Resets ultrasound measurements so new ones can be detected. */
  void resetUsMeasurements()
  {
    for(int i = lUsSensor; i <= rUs9Sensor; ++i)
      *sensorPtrs[i] = 0.f;
  }

  /**
    * Stabilize robot if in sitDown position.
    */
  void stabilize(float* controlledActuators)
  {
    stabilizationStatus = true;

    // check if robot is sitting correctly and stabilization should be enabled
    for (int i = lHipYawPitchPositionActuator; i < lbhNumOfPositionActuatorIds; ++i)
    {
      // don't stabilize if joint is not close enough to sitDown position
      if (std::abs(*sensorPtrs[i * 3] - sitDownAngles[i]) > 0.3f)
      {
        stabilizationStatus = false;
        break;
      }
    }
    
    static bool handsClosed[2] = { false, false };
    
    // set joints to sitDown position
    if (stabilizationStatus)
    {
      for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
      {
        // skip arms
        if ((i >= lShoulderPitchPositionActuator && i <= lWristYawPositionActuator)
          || (i >= rShoulderPitchPositionActuator && i <= rWristYawPositionActuator))
          continue;

        // skip head
        if (i == headYawPositionActuator || i == headPitchPositionActuator)
          continue;

        if (i == lHandPositionActuator || i == rHandPositionActuator){
            // close hands if opened
            if(std::abs(*sensorPtrs[i * 3] - sitDownAngles[i]) > 0.25f)
                handsClosed[i == rHandPositionActuator] = false;
            
            // skip closing hands if closed
            if(std::abs(*sensorPtrs[i * 3] - sitDownAngles[i]) < 0.05f)
                handsClosed[i == rHandPositionActuator] = true;
            
            if(handsClosed[i == rHandPositionActuator]) continue;
        }

        // only stabilize hip pitches and yaw
        if (i >= lHipYawPitchPositionActuator
          && i != lHipYawPitchPositionActuator
          && i != lHipPitchPositionActuator
          && i != rHipPitchPositionActuator)
          continue;

        controlledActuators[i] = sitDownAngles[i];
        controlledActuators[i + headYawStiffnessActuator] = 0.1f;

        // head pitch and fingers need more stiffness
        if (i == headPitchPositionActuator)
          controlledActuators[i + headYawStiffnessActuator] = 0.3f;
        if (i == lHandPositionActuator || i == rHandPositionActuator)
          controlledActuators[i + headYawStiffnessActuator] = 0.4f;
      }
    }
  }

  /**
   * Handles the different states libbhuman can be in.
   * @param actuators The actuator values requested. They will not be changed, but might
   *                  be used as result of this method.
   * @return The actuator values that should be set. In the standing state, they are
   *         identical to the actuators passed to this method. In all other states,
   *         they are different.
   */
  float* handleState(float* actuators)
  {
    static float controlledActuators[lbhNumOfActuatorIds];
    stabilizationStatus = false;

    switch(state)
    {
      sitting:
        state = sitting;
        lastBHumanStartTime = 0; // force gamectrl reinit when sitting

      case sitting:
        memset(controlledActuators, 0, sizeof(controlledActuators));
        data->transitionToBhuman = 0.f;

        stabilize(controlledActuators);
        triplePressSitDownRequested = false;

        // stay here as long as framework is not running or chest button is not pressed
        if(frameDrops > allowedFrameDrops || triplePressCount == 0)
          return controlledActuators;

        // don't count first press for triplePressSitDown
        triplePressCount = 0;

        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          startAngles[i] = *sensorPtrs[i * 3];

      standingUp:
        state = standingUp;
        phase = 0.f;

      case standingUp:
        if(phase < 1.f && frameDrops <= allowedFrameDrops)
        {
          memset(controlledActuators, 0, sizeof(controlledActuators));
          phase = std::min(phase + 0.004f, 1.f);
          data->transitionToBhuman = phase;
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          {
            float sinPhase = (std::cos(phase * 3.1416f) - 1.f) / -2.f;
            controlledActuators[i] = actuators[i] * sinPhase + startAngles[i] * (1.f - sinPhase);
            float h = std::min(actuators[i + headYawStiffnessActuator], 0.5f);
            controlledActuators[i + headYawStiffnessActuator] = actuators[i + headYawStiffnessActuator] * sinPhase + h * (1.f - sinPhase);
          }
          return controlledActuators;
        }
        state = standing;

      case standing:
        // framework is running and no sitDown is requested
        if(frameDrops <= allowedFrameDrops && !triplePressSitDownRequested)
        {
          // stabilize if robot has no stiffness
          if(!hasStiffness(actuators))
          {
            // memcopy here to keep LED requests from framework and avoid modifing the original data
            memcpy(controlledActuators, actuators, sizeof(controlledActuators));
            stabilize(controlledActuators);
            return controlledActuators;
          }
          else
          {
            return actuators; // use original actuators
          }
        }

      case preShuttingDown:
        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        {
          startAngles[i] = positionRequest[5][i][0];
          if(isStandHigh())
            startStiffness[i] = (i >= lShoulderPitchPositionActuator && i <= rElbowRollPositionActuator) ? 0.4f : 0.3f;
          else if(!hasStiffness(actuators))
            startStiffness[i] = 0.f;
          else if(i >= lShoulderPitchPositionActuator && i <= rElbowRollPositionActuator)
            startStiffness[i] = 0.4f;
          else
            startStiffness[i] = std::min<float>(stiffnessRequest[5][i][0], 0.3f);
        }
        state = state == preShuttingDown ? shuttingDown : sittingDown;
        phase = 0.f;

      case sittingDown:
      case shuttingDown:
      shuttingDown:
        // sit down if framework is stopped, sitDown is requested or system is shutting down
        if((phase < 1.f && (frameDrops > allowedFrameDrops || triplePressSitDownRequested)) || state == shuttingDown)
        {
          memset(controlledActuators, 0, sizeof(controlledActuators));
          phase = std::min(phase + 0.005f, 1.f);
          data->transitionToBhuman = 1.f - phase;
          
          float sinPhase;
          if(phase < 0.75f) {
            sinPhase = (std::cos(phase / 0.75f * 3.1416f) - 1.f) / -2.f;
          }else{
            sinPhase = (std::cos((phase - 0.75f) / 0.25f * 3.1416f) - 1.f) / -2.f;
          }
          
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          {
            if(phase < 0.75f) {
              controlledActuators[i] = sitDownAnglesHighArms[i] * sinPhase + startAngles[i] * (1.f - sinPhase);
            }else{
              controlledActuators[i] = sitDownAngles[i] * sinPhase + sitDownAnglesHighArms[i] * (1.f - sinPhase);
            }
            
            controlledActuators[i + headYawStiffnessActuator] = startStiffness[i];
          }
          return controlledActuators;
        }
        else if(frameDrops <= allowedFrameDrops && !triplePressSitDownRequested)
        {
          for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
            startAngles[i] = *sensorPtrs[i * 3];
          goto standingUp;
        }
        else
          goto sitting;

      case preShuttingDownWhileSitting:
        for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
          startStiffness[i] = 0.f;
        state = shuttingDown;
        phase = 0.995f;
        goto shuttingDown;
    }
  }
  
  /** This method disables the framework if the robot has no stiffness and starts charging. */
  void disableFrameworkWhenCharging(float* actuators) {
    static bool lastChargingStatus = false;
    if(state == standing &&
      !hasStiffness(actuators) &&
      !isStandHigh() &&
      chargingStatus &&
      !lastChargingStatus)
      triplePressSitDownRequested = true;
    
    
    lastChargingStatus = chargingStatus;
  }
  
  /** This method checks if the given actuator request has stiffness > 0 */
  bool hasStiffness(float* actuators) {
    for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
      if(actuators[i + headYawStiffnessActuator] > 0.f) return true;
    }
    
    return false;
  }
  
  /** This method checks if the robot is in stand high pose */
  bool isStandHigh() {
    for (int i = lHipYawPitchPositionActuator; i < lbhNumOfPositionActuatorIds; ++i)
    {
      if (std::abs(*sensorPtrs[i * 3] - standHighAngles[i]) > 0.2f)
      {
        return false;
      }
    }
    
    // check if body is upright
    if(std::abs(*sensorPtrs[angleXSensor]) > 0.2f) return false;
    if(std::abs(*sensorPtrs[angleYSensor]) > 0.2f) return false;
    
    return true;
  }
  
  /** The method sets all actuators. */
  void setActuators()
  {
    // set all actuator values according to the values in the shared memory block
    try
    {
      dcmTime = proxy->getTime(0);

      data->readingActuators = data->newestActuators;
      if(data->readingActuators == lastReadingActuators)
      {
        if(actuatorDrops == 0)
          fprintf(stderr, "libbhuman: missed actuator request.\n");
        ++actuatorDrops;
      }
      else
        actuatorDrops = 0;
      lastReadingActuators = data->readingActuators;
      float* readingActuators = data->actuators[data->readingActuators];
      disableFrameworkWhenCharging(readingActuators);
      float* actuators = handleState(readingActuators);

      if(state != standing)
      {
        if(frameDrops > 0 || state == shuttingDown)
          setEyeLeds(actuators);
        else
          copyNonServos(readingActuators, actuators);
      }
      setBatteryLeds(actuators);
      setStabilizationLeds(actuators);

      // set position actuators
      positionRequest[4][0] = dcmTime; // 0 delay!
      for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        positionRequest[5][i][0] = actuators[i];
      proxy->setAlias(positionRequest);

      // set stiffness actuators
      bool requestedStiffness = false;
      for(int i = headYawStiffnessActuator; i < headYawStiffnessActuator + lbhNumOfStiffnessActuatorIds; ++i)
        if(actuators[i] != requestedActuators[i])
        {
          stiffnessRequest[4][0] = dcmTime; // 0 delay!
          for(int j = 0; j < lbhNumOfStiffnessActuatorIds; ++j)
            stiffnessRequest[5][j][0] = requestedActuators[headYawStiffnessActuator + j] = actuators[headYawStiffnessActuator + j];
          proxy->setAlias(stiffnessRequest);
          requestedStiffness = true;
          break;
        }

      // set us actuator
      bool requestedUs = false;
      if(requestedActuators[usActuator] != actuators[usActuator])
      {
        requestedActuators[usActuator] = actuators[usActuator];
        if(actuators[usActuator] >= 0.f)
        {
          resetUsMeasurements();
          usRequest[4][0] = dcmTime;
          usRequest[5][0][0] = actuators[usActuator];
          proxy->setAlias(usRequest);
          requestedUs = true;
        }
      }

      // set led
      if(!requestedStiffness && !requestedUs)
        for(int i = 0; i < lbhNumOfLedActuatorIds; ++i)
        {
          int index = faceLedRedLeft0DegActuator + ledIndex;
          if(++ledIndex == lbhNumOfLedActuatorIds)
            ledIndex = 0;
          if(actuators[index] != requestedActuators[index])
          {
            ledRequest[0] = std::string(actuatorNames[index]);
            ledRequest[2][0][0] = requestedActuators[index] = actuators[index];
            ledRequest[2][0][1] = dcmTime;
            proxy->set(ledRequest);
            break;
          }
        }

      // set team info
      // since this should very rarely, we don't use a proxy here
      if(data->bhumanStartTime != lastBHumanStartTime)
      {
        for(int i = 0; i < lbhNumOfTeamInfoIds; ++i)
          memory->insertData(teamInfoNames[i], data->teamInfo[i]);
        lastBHumanStartTime = data->bhumanStartTime;
      }
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.what());
    }
  }

  /**
   * The method reads all sensors. It also detects if the chest button was pressed
   * for at least three seconds. In that case, it shuts down the robot.
   */
  void readSensors()
  {
    // get new sensor values and copy them to the shared memory block
    try
    {
      // copy sensor values into the shared memory block
      int writingSensors = 0;
      if(writingSensors == data->newestSensors)
        ++writingSensors;
      if(writingSensors == data->readingSensors)
        if(++writingSensors == data->newestSensors)
          ++writingSensors;
      assert(writingSensors != data->newestSensors);
      assert(writingSensors != data->readingSensors);

      float* sensors = data->sensors[writingSensors];
      for(int i = 0; i < lbhNumOfSensorIds; ++i)
        sensors[i] = *sensorPtrs[i];

      AL::ALValue value = memory->getData("GameCtrl/RoboCupGameControlData");
      if(value.isBinary() && value.getSize() == sizeof(RoboCup::RoboCupGameControlData))
        memcpy(&data->gameControlData[writingSensors], value, sizeof(RoboCup::RoboCupGameControlData));

      data->newestSensors = writingSensors;



      // detect shutdown request via chest-button
      if(*sensorPtrs[chestButtonSensor] == 0.f)
        startPressedTime = dcmTime;
      else if(state != shuttingDown && startPressedTime && dcmTime - startPressedTime > 3000)
      {
        (void) !system("( /home/nao/bin/bhumand stop && sudo shutdown -h now ) &");
        state = state == sitting ? preShuttingDownWhileSitting : preShuttingDown;
      }

      // detect triple button press for sitdown
      if (triplePressLastButtonState != static_cast<bool>(*sensorPtrs[chestButtonSensor]) && *sensorPtrs[chestButtonSensor] == 1.f)
      {
          if (triplePressCount == 0) triplePressStart = dcmTime;
          triplePressCount++;
      }
      triplePressLastButtonState = static_cast<bool>(*sensorPtrs[chestButtonSensor]);

      if (dcmTime - triplePressStart > 2000 || triplePressSitDownRequested) triplePressCount = 0;
      if (triplePressCount == 3 && *sensorPtrs[chestButtonSensor] == 0.f)
        triplePressSitDownRequested = true;

      // detect charing status
      chargingStatus = !(static_cast<int>(*sensorPtrs[batteryStatusSensor]) & 0b100000);
    }
    catch(AL::ALError& e)
    {
      fprintf(stderr, "libbhuman: %s\n", e.what());
    }

    // raise the semaphore
    if(sem != SEM_FAILED)
    {
      int sval;
      if(sem_getvalue(sem, &sval) == 0)
      {
        if(sval < 1)
        {
          sem_post(sem);
          frameDrops = 0;
        }
        else
        {
          if(frameDrops == 0)
            fprintf(stderr, "libbhuman: dropped sensor data.\n");
          ++frameDrops;
        }
      }
    }
  }

  /**
   * The method is called by NaoQi immediately before it communicates with the chest board.
   * It sets all the actuators.
   */
  static void onPreProcess()
  {
    theInstance->setActuators();
  }

  /**
   * The method is called by NaoQi immediately after it communicated with the chest board.
   * It reads all sensors.
   */
  static void onPostProcess()
  {
    theInstance->readSensors();
  }

public:
  /**
   * The constructor initializes the shared memory for communicating with bhuman.
   * It also establishes a communication with NaoQi and prepares all data structures
   * required for this communication.
   * @param pBroker A NaoQi broker that allows accessing other NaoQi modules.
   */
  BHuman(boost::shared_ptr<AL::ALBroker> pBroker) :
    ALModule(pBroker, "BHuman"),
    data((LBHData*)MAP_FAILED),
    sem(SEM_FAILED),
    proxy(0),
    memory(0),
    dcmTime(0),
    lastReadingActuators(-1),
    actuatorDrops(0),
    frameDrops(allowedFrameDrops + 1),
    state(sitting),
    phase(0.f),
    ledIndex(0),
    rightEarLEDsChangedTime(0),
    stabilizationStatus(false),
    chargingStatus(false),
    startPressedTime(0),
    lastBHumanStartTime(0),
    triplePressStart(0),
    triplePressCount(0),
    triplePressLastButtonState(false),
    triplePressSitDownRequested(false)
  {
    setModuleDescription("A module that provides basic ipc NaoQi DCM access using shared memory.");
    fprintf(stderr, "libbhuman: Starting.\n");

    assert(lbhNumOfSensorIds == sizeof(sensorNames) / sizeof(*sensorNames));
    assert(lbhNumOfActuatorIds == sizeof(actuatorNames) / sizeof(*actuatorNames));
    assert(lbhNumOfTeamInfoIds == sizeof(teamInfoNames) / sizeof(*teamInfoNames));

    // create shared memory
    memoryHandle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if(memoryHandle == -1)
      perror("libbhuman: shm_open");
    else if(ftruncate(memoryHandle, sizeof(LBHData)) == -1)
      perror("libbhuman: ftruncate");
    else
    {
      // map the shared memory
      data = (LBHData*) mmap(nullptr, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, memoryHandle, 0);
      if(data == MAP_FAILED)
        perror("libbhuman: mmap");
      else
      {
        memset(data, 0, sizeof(LBHData));

        // open semaphore
        sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
        if(sem == SEM_FAILED)
          perror("libbhuman: sem_open");
        else
          try
          {
            // get the robot name
            memory = new AL::ALMemoryProxy(pBroker);

            std::string bodyId = (std::string) memory->getData("Device/DeviceList/ChestBoard/BodyId", 0);
            strncpy(data->bodyId, bodyId.c_str(), sizeof(data->bodyId));
            data->bodyId[15] = 0;

            std::string headId = (std::string) memory->getData("RobotConfig/Head/FullHeadId", 0);
            strncpy(data->headId, headId.c_str(), sizeof(data->headId));
            data->headId[15] = 0;

            // create "positionRequest" and "stiffnessRequest" alias
            proxy = new AL::DCMProxy(pBroker);

            AL::ALValue params;
            AL::ALValue result;
            params.arraySetSize(1);
            params.arraySetSize(2);

            params[0] = std::string("positionActuators");
            params[1].arraySetSize(lbhNumOfPositionActuatorIds);
            for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
              params[1][i] = std::string(actuatorNames[i]);
            result = proxy->createAlias(params);

            params[0] = std::string("stiffnessActuators");
            params[1].arraySetSize(lbhNumOfStiffnessActuatorIds);
            for(int i = 0; i < lbhNumOfStiffnessActuatorIds; ++i)
              params[1][i] = std::string(actuatorNames[headYawStiffnessActuator + i]);
            result = proxy->createAlias(params);

            params[0] = std::string("usRequest");
            params[1].arraySetSize(1);
            params[1][0] = std::string(actuatorNames[usActuator]);
            result = proxy->createAlias(params);

            // prepare positionRequest
            positionRequest.arraySetSize(6);
            positionRequest[0] = std::string("positionActuators");
            positionRequest[1] = std::string("ClearAll");
            positionRequest[2] = std::string("time-separate");
            positionRequest[3] = 0;
            positionRequest[4].arraySetSize(1);
            positionRequest[5].arraySetSize(lbhNumOfPositionActuatorIds);
            for(int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
              positionRequest[5][i].arraySetSize(1);

            // prepare stiffnessRequest
            stiffnessRequest.arraySetSize(6);
            stiffnessRequest[0] = std::string("stiffnessActuators");
            stiffnessRequest[1] = std::string("ClearAll");
            stiffnessRequest[2] = std::string("time-separate");
            stiffnessRequest[3] = 0;
            stiffnessRequest[4].arraySetSize(1);
            stiffnessRequest[5].arraySetSize(lbhNumOfStiffnessActuatorIds);
            for(int i = 0; i < lbhNumOfStiffnessActuatorIds; ++i)
              stiffnessRequest[5][i].arraySetSize(1);

            // prepare usRequest
            usRequest.arraySetSize(6);
            usRequest[0] = std::string("usRequest");
            usRequest[1] = std::string("Merge"); // doesn't work with "ClearAll"
            usRequest[2] = std::string("time-separate");
            usRequest[3] = 0;
            usRequest[4].arraySetSize(1);
            usRequest[5].arraySetSize(1);
            usRequest[5][0].arraySetSize(1);

            // prepare ledRequest
            ledRequest.arraySetSize(3);
            ledRequest[1] = std::string("ClearAll");
            ledRequest[2].arraySetSize(1);
            ledRequest[2][0].arraySetSize(2);
            ledRequest[2][0][1] = 0;

            // prepare sensor pointers
            for(int i = 0; i < lbhNumOfSensorIds; ++i)
              sensorPtrs[i] = (float*) memory->getDataPtr(sensorNames[i]);
            resetUsMeasurements();

            // initialize requested actuators
            memset(requestedActuators, 0, sizeof(requestedActuators));
            for(int i = faceLedRedLeft0DegActuator; i < chestBoardLedRedActuator; ++i)
              requestedActuators[i] = -1.f;

            // register "onPreProcess" and "onPostProcess" callbacks
            theInstance = this;
            proxy->getGenericProxy()->getModule()->atPreProcess(&onPreProcess);
            proxy->getGenericProxy()->getModule()->atPostProcess(&onPostProcess);

            fprintf(stderr, "libbhuman: Started!\n");
            return; // success
          }
          catch(AL::ALError& e)
          {
            fprintf(stderr, "libbhuman: %s\n", e.what());
          }
      }
    }
    close(); // error
  }

  /** Close all resources acquired. */
  ~BHuman()
  {
    close();
  }
};

BHuman* BHuman::theInstance = 0;

/**
 * This method is called by NaoQi when loading this library.
 * Creates an instance of class BHuman.
 * @param pBroker A NaoQi broker that allows accessing other NaoQi modules.
 */
extern "C" int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
{
  AL::ALModule::createModule<BHuman>(pBroker);
  return 0;
}
