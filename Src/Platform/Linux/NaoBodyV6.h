/**
* @file Platform/linux/NaoBodyV6.h
* Declaration of a class for accessing the body of the nao 
* @author Colin Graf
* @author TheCanadianGuy & schwingmar
*/

#pragma once

#include <cstdio>
#include "naodevilsbase/naodevilsbase.h"

/**
* @class NaoBodyV6
* Encapsulates communication with ndevilsbase.
*/
class NaoBodyV6
{
private:
  FILE* fdCpuTemp = nullptr;

public:
  ~NaoBodyV6();

  /** Initializes access to ndevilsbase
  * @return Whether the initialization was successful or not */
  bool init();

  /** Finalize access to ndevilsbase */
  void cleanup();

  /** Waits for a new set of sensor data */
  bool wait();

  /** Activates the eye-blinking mode to indicate a crash.
  * @param termSignal The termination signal that was raised by the crash. */
  void setCrashed(int termSignal);

  /**
   * Accesses the id of the robot's head.
   * @return The name.
   */
  const char* getHeadId() const;

  /**
   * Accesses the id of the robot's body.
   * @return The name.
   */
  const char* getBodyId() const;

  /** Accesses the sensor value buffer.
  * @return An array of sensor values. Ordered corresponding to 
  * a SensorData struct. */
  const NDData::SensorData& getSensors();

  /** Accesses CPU temperature sensor */
  float getCPUTemperature();

  /** If ndevilsbase has given full control to framework. (0 = ndevilsbase, 1 = framework) Range: [0.0, 1.0] */
  float getTransitionToFramework();

  /** Determine status of wlan hardware. */
  static bool getWlanStatus();

  /** Accesses the actuator value buffer for writing.
  * @param actuators A reference to a variable to store a pointer to the actuator value buffer in.*/
  NDData::ActuatorData& openActuators();

  /** Commits the actuator value buffer. */
  void closeActuators();
};
