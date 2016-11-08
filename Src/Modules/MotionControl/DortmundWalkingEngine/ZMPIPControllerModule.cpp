/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPController.cpp
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "ZMPIPControllerModule.h"
#include "Tools/Debugging/DebugDrawings.h"
ZMPIPControllerModule::ZMPIPControllerModule():
controller(
		theRobotModel,
		theRefZMP,
		theInertialSensorData,
		theWalkingEngineParams,
		thePatternGenRequest,
		theFallDownState,
		theControllerParams,
		theZMPModel,
		theWalkingInfo)
{
}

void ZMPIPControllerModule::update(TargetCoM &theTargetCoM)
{
	Vector3f x, y;
	controller.updateKinematicRequest(theTargetCoM);
	controller.getObservations(x, y);
	PLOT("module:ZMPIPController:ZMP.x",x[2]);
	PLOT("module:ZMPIPController:ZMP.y",y[2]);

	PLOT("module:ZMPIPController:Spd.x",x[1]);
	PLOT("module:ZMPIPController:Spd.y",y[1]);

	PLOT("module:ZMPIPController:Pos.x",x[0]);
	PLOT("module:ZMPIPController:Pos.y",y[0]);
};


MAKE_MODULE(ZMPIPControllerModule, dortmundWalkingEngine)