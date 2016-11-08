/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPControllerNG.cpp
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "ZMPIPControllerNGModule.h"
#include "Tools/Debugging/DebugDrawings.h"
ZMPIPControllerNGModule::ZMPIPControllerNGModule():
controller(
		theRobotModel,
		theRefZMP,
		theInertialSensorData,
		theWalkingEngineParams,
		thePatternGenRequest,
		theFallDownState,
		theControllerParams,
		theZMPModel,
		theWalkingInfo,
		theJointSensorData,
		theActualCoM)
{
}


void ZMPIPControllerNGModule::update(TargetCoM &theTargetCoM)
{
	Vector3f x, y;
	Vector2f refZMP=controller.getReferenceZMP();

	controller.updateKinematicRequest(theTargetCoM);
	controller.getObservations(x, y);

	PLOT("module:ZMPIPControllerNG:ZMP.x",x[2]);
	PLOT("module:ZMPIPControllerNG:ZMP.y",y[2]);

	PLOT("module:ZMPIPControllerNG:Spd.x",x[1]);
	PLOT("module:ZMPIPControllerNG:Spd.y",y[1]);

	PLOT("module:ZMPIPControllerNG:Pos.x",x[0]);
	PLOT("module:ZMPIPControllerNG:Pos.y",y[0]);

	PLOT("module:ZMPIPControllerNG:ReferenceZMP.x", refZMP[0]);
	PLOT("module:ZMPIPControllerNG:ReferenceZMP.y", refZMP[1]);
};


MAKE_MODULE(ZMPIPControllerNGModule, dortmundWalkingEngine)