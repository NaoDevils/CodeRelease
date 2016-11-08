/** 
* @file Modules/MotionControl/DortmundWalkingEngine/TiltControllerModule.cpp
* Module wrapper for the TiltController
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "TiltControllerModule.h"

TiltControllerModule::TiltControllerModule():
controller(
		theInertialSensorData,
		theWalkingEngineParams/*,
		theJointCalibration,
		theJointRequest,
		theSpeedInfo,
		theFreeLegPhaseParams,
		theTargetCoM*/)
{
}

void TiltControllerModule::update(BodyTilt &theBodyTilt)
{
	controller.updateBodyTilt(theBodyTilt);
};


MAKE_MODULE(TiltControllerModule, dortmundWalkingEngine)