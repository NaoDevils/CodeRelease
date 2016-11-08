/** 
* @file Modules/MotionControl/GyroTiltController.cpp
* This file implements a simple body orientation controller.
* @author <A href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</A>
*/

#include "GyroTiltController.h"
#include "Tools/Debugging/DebugDrawings.h"

GyroTiltController::GyroTiltController()
{
	
  initialized = false;
}


GyroTiltController::~GyroTiltController(void)
{
}

void GyroTiltController::init()
{
  initialized = true;
  gyroX = gyroY = 0;
}


void GyroTiltController::update(BodyTilt &bodyTilt)
{
  if (!initialized)
    init();
  filterGyroValues();
	bodyTilt.x = theWalkingEngineParams.rollControllerParams[0] * gyroX;
	bodyTilt.y = theWalkingEngineParams.tiltControllerParams[0] * gyroY;
}


void GyroTiltController::filterGyroValues()
{
  const float alpha = gyroDriftCompensationFilterReactivity;
  gyroOffsetX = (1.0f - alpha) * gyroOffsetX + alpha * theInertialSensorData.gyro.x() * 0.0075f;
  gyroOffsetY = (1.0f - alpha) * gyroOffsetY + alpha * theInertialSensorData.gyro.y() * 0.0075f;
  PLOT("modules:GyroTiltController:gyroOffsetY", gyroOffsetY);
  gyroX = (1.0f - gyroFilterReactivity)*gyroX + gyroFilterReactivity* (theInertialSensorData.gyro.x() * 0.0075f - gyroOffsetX);
  gyroY = (1.0f - gyroFilterReactivity)*gyroY + gyroFilterReactivity* (theInertialSensorData.gyro.y() * 0.0075f - gyroOffsetY);
}


MAKE_MODULE(GyroTiltController, dortmundWalkingEngine)