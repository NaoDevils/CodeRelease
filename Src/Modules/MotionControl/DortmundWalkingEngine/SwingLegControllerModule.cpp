/** 
* @file Modules/MotionControl/DortmundWalkingEngine/SwingLegControllerModule.cpp
* Module wrapper for the SwingLegController
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "SwingLegControllerModule.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"
SwingLegControllerModule::SwingLegControllerModule():
controller(
  theWalkingEngineParams,
  theFootSteps,
  theBallModel,
  theMotionRequest,
  theWalkingInfo,
  theFallDownState,
  thePatternGenRequest,
  theObservedError,
  theControllerParams,
  theInertialSensorData)
{
}

void SwingLegControllerModule::update(Footpositions &footpositions)
{
  controller.updateFootpositions(footpositions);
  
  DECLARE_DEBUG_DRAWING3D("Preview", "robot");
  if (!controller.phases.empty())
	  for (SwingLegController::PhaseList::iterator p = ++controller.phases.begin(); p != controller.phases.end(); p++)
	  {
		  for (int footNum = 0; footNum < 2; footNum++)
		  {
		    Point fp = p->members.front().footPos[footNum];
		    Vector2f fpRCS = theWalkingInfo.toRobotCoords(Vector2f(fp.x, fp.y));
		    SPHERE3D("Preview", fpRCS.x() * 1000, fpRCS.y() * 1000, -230, 20, ColorRGBA::blue);
		  }
	  }
};


void SwingLegControllerModule::update(ReferenceModificator &referenceModificator)
{
  controller.update(referenceModificator);
};

MAKE_MODULE(SwingLegControllerModule, dortmundWalkingEngine)