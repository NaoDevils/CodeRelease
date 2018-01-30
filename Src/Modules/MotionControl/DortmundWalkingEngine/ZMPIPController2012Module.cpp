/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPControllerNG.cpp
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "ZMPIPController2012Module.h"
#include "Tools/Debugging/DebugDrawings.h"
ZMPIPController2012Module::ZMPIPController2012Module():
controller(
		theRefZMP,
        theWalkingEngineParams,
		theControllerParams,
        theObservedError,
        theReferenceModificator)
{
}

//void ZMPIPController2012Module::update(TargetCoMLIPM & theTargetCoMLIPM) {
//  update((TargetCoM&)theTargetCoMLIPM);
//}

void ZMPIPController2012Module::update(TargetCoM &theTargetCoM)
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

  theTargetCoM.state_x[0] = x[0];
  theTargetCoM.state_x[1] = x[1];
  theTargetCoM.state_x[2] = x[2];

  theTargetCoM.state_y[0] = y[0];
  theTargetCoM.state_y[1] = y[1];
  theTargetCoM.state_y[2] = y[2];


};


MAKE_MODULE(ZMPIPController2012Module, dortmundWalkingEngine)