/** 
* @file Modules/MotionControl/GyroTiltController.h
* This file implements a simple body orientation controller.
* @author <A href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</A>
*/

#pragma once


#include "Tools/Module/Module.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"

MODULE(GyroTiltController,
{,
  REQUIRES(InertialSensorData),
  REQUIRES(WalkingEngineParams),
  PROVIDES(BodyTilt),
  DEFINES_PARAMETERS(
  {,
    (float)(0.01f) gyroFilterReactivity,
    (float)(0.001f) gyroDriftCompensationFilterReactivity,
    (float)(0.f) gyroOffsetX,
    (float)(0.f) gyroOffsetY,
  }),
});

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
	void update(BodyTilt &bodyTilt);

  void init();

private:
  bool initialized;
  float gyroX,gyroY;

  void filterGyroValues();
};

