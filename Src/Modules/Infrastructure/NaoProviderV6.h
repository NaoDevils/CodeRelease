/**
 * @file Modules/Infrastructure/NaoProviderV6.h
 * The file declares a module that provides information from the Nao via LoLA.
 * @author TheCanadianGuy & schwingmar
 */

#pragma once

#include "Platform/Linux/NaoBodyV6.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/USRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/UsSensorData.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(NaoProviderV6,
{,
  REQUIRES(JointCalibration),
  REQUIRES(LEDRequest),
  REQUIRES(USRequest),

  PROVIDES(FrameInfo),
  REQUIRES(FrameInfo),

  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(SystemSensorData),
  PROVIDES(UsSensorData),
  USES(JointRequest), // Will be accessed in send()
});

#ifdef TARGET_ROBOT

/**
 * @class NaoProviderV6
 * A module that provides information from the Nao.
 */
class NaoProviderV6 : public NaoProviderV6Base
{
private:
  static PROCESS_LOCAL NaoProviderV6* theInstance; /**< The only instance of this module. */

  NaoBodyV6 naoBody;
  float clippedLastFrame[Joints::numOfJoints]; /**< Array that indicates whether a certain joint value was clipped in the last frame (and what was the value)*/
  RingBufferWithSum<float, 20> gyroBufferX;
  RingBufferWithSum<float, 20> gyroBufferY;
  RingBufferWithSum<float, 20> gyroBufferZ;
  float gyroXBias = 0.f;
  float gyroYBias = 0.f;
  float gyroZBias = 0.f;
  bool soundPlayed = false;
  unsigned lastBodyTemperatureReadTime = 0;
  bool on = false;
  bool fastOn = false;

public:
  NaoProviderV6();
  ~NaoProviderV6();

  /** The method is called by process Motion to send the requests to the Nao. */
  static void finishFrame();
  static void waitForFrameData();

private:
  void update(FrameInfo& frameInfo);
  void update(FsrSensorData& fsrSensorData);
  void update(InertialSensorData& inertialSensorData);
  void update(JointSensorData& jointSensorData);
  void update(KeyStates& keyStates);
  void update(SystemSensorData& systemSensorData);
  void update(UsSensorData& usSensorData);

  /** The function sends a command to the Nao. */
  void send();

  void copyStiffness(const JointRequest &theJointRequest, NDActuatorData* actuators);
  void copyLEDs(const LEDRequest &theLEDRequest, NDActuatorData* actuators);
  void setLED(const LEDRequest::LEDState &state, float& actuator);
  void copyJointValues(float* source, Angle* target);
  void copyCurrents(float* source, short* target);
  void copyTemperatures(float* source, unsigned char* target);
  void copyTemperatureStatus(int* source, unsigned char* target);
};

#else
//TARGET_ROBOT not defined here (Simulator).

class NaoProviderV6 : public NaoProviderV6Base
{
private:
  void update(FrameInfo& frameInfo) {}
  void update(FsrSensorData& fsrSensorData) {}
  void update(InertialSensorData& inertialSensorData) {}
  void update(JointSensorData& jointSensorData) {}
  void update(KeyStates& keyStates) {}
  void update(SystemSensorData& systemSensorData) {}
  void update(UsSensorData& usSensorData) {}
  void send();

public:
  static void finishFrame() {}
  static void waitForFrameData() {}
};

#endif
