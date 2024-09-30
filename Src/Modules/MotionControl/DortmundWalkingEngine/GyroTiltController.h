/** 
* @file Modules/MotionControl/GyroTiltController.h
* This file implements a simple body orientation controller.
* @author <A href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</A>
*/

#pragma once


#include "Tools/Module/Module.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/Sensing/JoinedIMUData.h"

MODULE(GyroTiltController,
  REQUIRES(JoinedIMUData),
  REQUIRES(Footpositions),
  USES(WalkingInfo),
  PROVIDES(BodyTilt),
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (float)(0.01f) gyroFilterReactivity,
    (float)(0.001f) gyroDriftCompensationFilterReactivity,
    (float)(0.f) gyroOffsetX,
    (float)(0.f) gyroOffsetY,
    (float)(0.05f) orientationFilterReactivity,
    (float[3]) rollControllerParams, // p, i, d
    (float[3]) tiltControllerParams // p, i, d
  )
);


/**
 * @class GyroTiltController
 * Determines the target orientation of the body.
 */
class GyroTiltController : public GyroTiltControllerBase
{
public:
  /** Constructor with all needed source data structures.
	 * @param theSensorData Measured data.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
  GyroTiltController();

  /** Destructor */
  ~GyroTiltController();

  /** 
	 * Calculates the next target orientation.
	 * @param bodyTilt Target data structure.
	 */
  void update(BodyTilt& bodyTilt);

  void init();

private:
  bool initialized;
  float gyroX, gyroY, oriX, oriY, IX, IY; // , eX, eY;

  void filterGyroValues();
};
