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


GyroTiltController::~GyroTiltController(void) {}

void GyroTiltController::init()
{
  initialized = true;
  gyroX = gyroY = 0;
  oriX = oriY = 0;
  IX = IY = 0;
}


void GyroTiltController::update(BodyTilt& bodyTilt)
{
  if (!initialized || !theWalkingInfo.isRunning)
    init();
  float sign = 1.0f;
  if (theWalkingInfo.isRunning)
  {
    filterGyroValues();
    sign = theFootpositions.speed.translation.x() < 0.f ? -1.0f : 1.0f;
    bodyTilt.x = rollControllerParams[2] * -gyroX; // Assuming the desired angle speed is 0
    bodyTilt.x += rollControllerParams[0] * (theWalkingInfo.desiredBodyRot.x() - oriX);
    bodyTilt.x += (IX += -(theJoinedIMUData.imuData[anglesource].angle.x()), rollControllerParams[1] * IX);

    bodyTilt.y = sign * tiltControllerParams[2] * -gyroY;
    bodyTilt.y += sign * tiltControllerParams[0] * (theWalkingInfo.desiredBodyRot.y() - oriY);
    bodyTilt.y += (IY += theWalkingInfo.desiredBodyRot.y() - (theJoinedIMUData.imuData[anglesource].angle.y()), sign * tiltControllerParams[1] * IY);
  }
  PLOT("module:GyroTiltController:sign", sign);
  PLOT("module:GyroTiltController:gyroX", gyroX);
  PLOT("module:GyroTiltController:gyroY", gyroY);
  PLOT("module:GyroTiltController:IX", IX);
  PLOT("module:GyroTiltController:IY", IY);
  PLOT("module:GyroTiltController:oriX", oriX);
  PLOT("module:GyroTiltController:oriY", oriY);
  PLOT("module:GyroTiltController:desiredBodyRot.y()", theWalkingInfo.desiredBodyRot.y());
}


void GyroTiltController::filterGyroValues()
{
  const float alpha = gyroDriftCompensationFilterReactivity;
  gyroOffsetX = (1.0f - alpha) * gyroOffsetX + alpha * (theJoinedIMUData.imuData[anglesource].gyro.x()) * 0.0075f;
  gyroOffsetY = (1.0f - alpha) * gyroOffsetY + alpha * (theJoinedIMUData.imuData[anglesource].gyro.y()) * 0.0075f;
  PLOT("modules:GyroTiltController:gyroOffsetY", gyroOffsetY);
  gyroX = (1.0f - gyroFilterReactivity) * gyroX + gyroFilterReactivity * ((theJoinedIMUData.imuData[anglesource].gyro.x()) * 0.0075f - gyroOffsetX);
  gyroY = (1.0f - gyroFilterReactivity) * gyroY + gyroFilterReactivity * ((theJoinedIMUData.imuData[anglesource].gyro.y()) * 0.0075f - gyroOffsetY);
  oriX = (1.0f - orientationFilterReactivity) * oriX + orientationFilterReactivity * (theJoinedIMUData.imuData[anglesource].angle.x());
  oriY = (1.0f - orientationFilterReactivity) * oriY + orientationFilterReactivity * (theJoinedIMUData.imuData[anglesource].angle.y());
}


MAKE_MODULE(GyroTiltController, dortmundWalkingEngine)
