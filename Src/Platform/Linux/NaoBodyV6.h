/**
* @file Platform/linux/NaoBodyV6.h
* Declaration of a class for accessing the body of the nao 
* @author Colin Graf
* @author TheCanadianGuy & schwingmar
*/

#pragma once

#include <cstdio>
#include "ndevilsbase/ndevils.h"

/**
* @class NaoBodyV6
* Encapsulates communication with ndevilsbase.
*/
class NaoBodyV6
{
private:
  int writingActuators = -1; /**< The index of the opened exclusive actuator writing buffer. */

  FILE* fdCpuTemp = nullptr;

public:
  ~NaoBodyV6();

  /** Initializes access to libbhuman
  * @return Whether the initialization was successful or not */
  bool init();

  /** Finalize access to libbhuman */
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
  * @return An array of sensor values. Ordered corresponding to \c lbhSensorNames of \c bhuman.h for V5 
  * or as a NDSensorData struct for V6. */
  NDSensorData* getSensors();

  /** Accesses CPU temperature sensor */
  float getCPUTemperature();

  /** If ndevilsbase has given full control to bhuman. (0 = ndevilsbase, 1 = bhuman) Range: [0.0, 1.0] */
  float getTransitionToBhuman();

  /** Determine status of wlan hardware. */
  bool getWlanStatus();

  /** Accesses the actuator value buffer for writing.
  * @param actuators A reference to a variable to store a pointer to the actuator value buffer in.*/
  void openActuators(NDActuatorData*& actuators);

  /** Commits the actuator value buffer. */
  void closeActuators();
};
