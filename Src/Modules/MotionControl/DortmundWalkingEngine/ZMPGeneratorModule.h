/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPGeneratorModule.h
* Module wrapper for the ZMPGenerator
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/FootSteps.h"
#include "ZMPGenerator.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"

MODULE(ZMPGeneratorModule,
{ ,
  REQUIRES(FootSteps),
  //USES(WalkingInfo),
  REQUIRES(ReferenceModificator),
  REQUIRES(WalkingEngineParams),
  REQUIRES(ControllerParams),
  PROVIDES(RefZMP),
});

class ZMPGeneratorModule : public ZMPGeneratorModuleBase
{
public:
  ZMPGeneratorModule();

	ZMPGenerator generator;

	void update(RefZMP &theRefZMP);
};


