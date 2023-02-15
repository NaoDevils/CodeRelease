/**
* @file ndevils.h
* Declaration of shared data provided by ndevils.
* @author TheCanadianGuy & schwingmar
*/

#pragma once

#include <chrono>
#include <array>
#include "../Tools/TripleBuffer.h"

struct NDData
{
  using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;

  enum Joint
  {
    headYaw,
    headPitch,
    lShoulderPitch,
    lShoulderRoll,
    lElbowYaw,
    lElbowRoll,
    lWristYaw,
    lHipYawPitch,
    lHipRoll,
    lHipPitch,
    lKneePitch,
    lAnklePitch,
    lAnkleRoll,
    rHipRoll,
    rHipPitch,
    rKneePitch,
    rAnklePitch,
    rAnkleRoll,
    rShoulderPitch,
    rShoulderRoll,
    rElbowYaw,
    rElbowRoll,
    rWristYaw,
    lHand,
    rHand,

    numOfJoints,
  };

  enum Touch
  {
    chestButton,
    headFront,
    headMiddle,
    headRear,
    lBumperLeft,
    lBumperRight,
    lHandBack,
    lHandLeft,
    lHandRight,
    rBumperLeft,
    rBumperRight,
    rHandBack,
    rHandLeft,
    rHandRight,

    numOfTouchs,
  };

  enum FSR
  {
    lFrontLeft,
    lFrontRight,
    lRearLeft,
    lRearRight,
    rFrontLeft,
    rFrontRight,
    rRearLeft,
    rRearRight,
    numOfFsrs,
  };

  enum Sonar
  {
    left,
    right,
    numOfSonars,
  };

  enum Battery
  {
    charge,
    status,
    current,
    temperature,
    numOfBattery,
  };

  enum SkullLED
  {
    frontLeft0,
    frontLeft1,
    middleLeft0,
    rearLeft0,
    rearLeft1,
    rearLeft2,
    rearRight0,
    rearRight1,
    rearRight2,
    middleRight0,
    frontRight0,
    frontRight1,
    numOfSkullLEDs,
  };

  enum LEyeLED
  {
    lEyeDeg45,
    lEyeDeg0,
    lEyeDeg315,
    lEyeDeg270,
    lEyeDeg225,
    lEyeDeg180,
    lEyeDeg135,
    lEyeDeg90,
    numOfLEyeLEDs,
  };

  enum REyeLED
  {
    rEyeDeg0,
    rEyeDeg45,
    rEyeDeg90,
    rEyeDeg135,
    rEyeDeg180,
    rEyeDeg225,
    rEyeDeg270,
    rEyeDeg315,
    numOfREyeLEDs,
  };

  enum LEarLED
  {
    lEarDeg0,
    lEarDeg36,
    lEarDeg72,
    lEarDeg108,
    lEarDeg144,
    lEarDeg180,
    lEarDeg216,
    lEarDeg252,
    lEarDeg288,
    lEarDeg324,
    numOfLEarLEDs,
  };

  enum REarLED
  {
    rEarDeg324,
    rEarDeg288,
    rEarDeg252,
    rEarDeg216,
    rEarDeg180,
    rEarDeg144,
    rEarDeg108,
    rEarDeg72,
    rEarDeg36,
    rEarDeg0,
    numOfREarLEDs,
  };

  enum RGBLED
  {
    r,
    g,
    b,
    numOfRGBLEDs,
  };

  enum State
  {
    okState = 0,
    abnormalTerminationState = 1,
    sigINTState = 2,
    sigQUITState = 3,
    sigILLState = 4,
    sigABRTState = 6,
    sigFPEState = 8,
    sigKILLState = 9,
    sigSEGVState = 11,
    sigPIPEState = 13,
    sigALRMState = 14,
    sigTERMState = 15,
  };

  struct SensorData
  {
    std::array<float, 3> acc;
    std::array<float, 2> angle;
    std::array<float, numOfBattery> battery;
    std::array<float, numOfJoints> current;
    std::array<float, numOfFsrs> fsr;
    std::array<float, 3> gyro;
    std::array<float, numOfJoints> position;
    std::array<float, numOfSonars> sonar;
    std::array<float, numOfJoints> stiffness;
    std::array<float, numOfJoints> temperature;
    std::array<float, numOfTouchs> touch;
    std::array<int, numOfJoints> status; // overheated or not
    int timestamp;
  };

  struct ActuatorData
  {
    std::array<float, numOfRGBLEDs> chestLEDs;
    std::array<float, numOfLEarLEDs> lEarLEDs;
    std::array<std::array<float, numOfLEyeLEDs>, numOfRGBLEDs> lEyeLEDs;
    std::array<float, numOfRGBLEDs> lFootLEDs;
    std::array<float, numOfJoints> positions;
    std::array<float, numOfREarLEDs> rEarLEDs;
    std::array<std::array<float, numOfREyeLEDs>, numOfRGBLEDs> rEyeLEDs;
    std::array<float, numOfRGBLEDs> rFootLEDs;
    std::array<float, numOfSkullLEDs> skullLEDs;
    std::array<float, numOfJoints> stiffness;
    std::array<bool, numOfSonars> sonars;
  };

  static constexpr const char* sem_name_sensors = "/ndevils_sem_sensors";
  static constexpr const char* sem_name_actuators = "/ndevils_sem_actuators";
  static constexpr const char* mem_name = "/ndevils_mem";

  char bodyId[21]; /* Device/DeviceList/ChestBoard/BodyId */
  char headId[21]; /* RobotConfig/Head/FullHeadId */
  TripleBuffer<SensorData> sensors; /* Triple buffer for sensor data from LoLA. */
  TripleBuffer<ActuatorData> actuators; /* Triple buffer for actuator data to LoLA. */

  State state;
  SteadyTimePoint ndevilsStartTime;

  std::atomic<float> transitionToFramework; /** If ndevilsbase has given full control to framework. (0 = ndevilsbase, 1 = framework) Range: [0.0, 1.0] */
};

#ifdef NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NDData::SensorData, acc, angle, battery, current, fsr, gyro, position, sonar, stiffness, temperature, touch, status, timestamp)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NDData::ActuatorData, chestLEDs, lEarLEDs, lEyeLEDs, lFootLEDs, positions, rEarLEDs, rEyeLEDs, rFootLEDs, skullLEDs, stiffness, sonars)
#endif
