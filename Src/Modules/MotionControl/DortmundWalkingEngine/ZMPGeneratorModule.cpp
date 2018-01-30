/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPGeneratorModule.cpp
* Module wrapper for the ZMPGenerator
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "ZMPGeneratorModule.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDrawings.h"

ZMPGeneratorModule::ZMPGeneratorModule():
generator(
		theFootSteps,
		theWalkingEngineParams,
		theControllerParams,
		//theWalkingInfo,
        theReferenceModificator)
{
}

void ZMPGeneratorModule::update(RefZMP &theRefZMP)
{
	generator.updateRefZMP(theRefZMP);
  PLOT("module:ZMPGenerator:unrotatedZMP.x", generator.plotZMP.x);
  PLOT("module:ZMPGenerator:unrotatedZMP.y", generator.plotZMP.y);
};


MAKE_MODULE(ZMPGeneratorModule, dortmundWalkingEngine)