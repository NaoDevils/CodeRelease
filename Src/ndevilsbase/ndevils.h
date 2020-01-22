/**
* @file ndevils.h
* Declaration of shared data provided by ndevils.
* @author TheCanadianGuy & schwingmar
*/

#pragma once

#include "stdint.h"
#include <iostream>
#include <chrono>

//#include "Representations/Infrastructure/RoboCupGameControlData.h"

#define ND_SEM_NAME "/ndevils_sem"
#define ND_MEM_NAME "/ndevils_mem"

typedef std::chrono::time_point<std::chrono::steady_clock> SteadyTimePoint;

enum NDJoints
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

enum NDTouchs
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

enum NDFsrs
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

enum NDSonars
{
  left,
  right,
  numOfSonars,
};

enum NDBattery
{
  charge,
  status,
  current,
  temperature,
  numOfBattery,
};

enum NDSkullLEDs
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

enum NDLEyeLEDs
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

enum NDREyeLEDs
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

enum NDLEarLEDs
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

enum NDREarLEDs
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

enum NDRGBLEDs
{
  r,
  g,
  b,
  numOfRGBLEDs,
};

enum NDState
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

struct NDSensorData
{
  float acc[3];
  float angle[2];
  float battery[numOfBattery];
  float current[numOfJoints];
  float fsr[numOfFsrs];
  float gyro[3];
  float position[numOfJoints];
  float sonar[numOfSonars];
  float stiffness[numOfJoints];
  float temperature[numOfJoints];
  float touch[numOfTouchs];
  int status[numOfJoints]; // overheated or not
  int timestamp;
};

struct NDActuatorData
{
  float chestLEDs[numOfRGBLEDs];
  float lEarLEDs[numOfLEarLEDs];
  float lEyeLEDs[numOfRGBLEDs][numOfLEyeLEDs];
  float lFootLEDs[numOfRGBLEDs];
  float positions[numOfJoints];
  float rEarLEDs[numOfREarLEDs];
  float rEyeLEDs[numOfRGBLEDs][numOfREyeLEDs];
  float rFootLEDs[numOfRGBLEDs];
  float skullLEDs[numOfSkullLEDs];
  float stiffness[numOfJoints];
  bool sonars[numOfSonars];
};


struct NDData
{
  volatile int readingSensors; /**< Index of sensor data reserved for reading. */
  volatile int newestSensors; /**< Index of the newest sensor data. */
  volatile int readingActuators; /**< Index of actuator commands reserved for reading. */
  volatile int newestActuators; /**< Index of the newest actuator command. */

  char bodyId[21]; /* Device/DeviceList/ChestBoard/BodyId */
  char headId[21]; /* RobotConfig/Head/FullHeadId */
  NDSensorData sensors[3]; /* Triple buffer for sensor data from LoLA. */
  NDActuatorData actuators[3]; /* Triple buffer for actuator data to LoLA. */
  
  NDState state;
  SteadyTimePoint ndevilsStartTime;

  /*
   * I don't think triple buffering is really necessary here.
   * In my opinion this would be harder than for the other data
   * as we have to pass data from setActuators() function to the framework,
   * what is no intended by the current design.
   */
  float transitionToBhuman; /** If ndevilsbase has given full control to bhuman. (0 = ndevilsbase, 1 = bhuman) Range: [0.0, 1.0] */
};
