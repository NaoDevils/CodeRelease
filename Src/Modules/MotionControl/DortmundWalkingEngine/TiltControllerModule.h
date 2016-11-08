/** 
* @file Modules/MotionControl/DortmundWalkingEngine/TiltControllerModule.h
* Module wrapper for the TiltController
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "TiltController.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
//#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/BodyTilt.h"
//#include "Representations/MotionControl/SpeedInfo.h"
//#include "Representations/MotionControl/TargetCoM.h"
//#include "Representations/Configuration/JointCalibration.h"

MODULE(TiltControllerModule,
{ ,
  REQUIRES(InertialSensorData),
  //USES(JointRequest),
  //REQUIRES(JointCalibration),
  REQUIRES(WalkingEngineParams),
  //REQUIRES(FreeLegPhaseParams),
  //REQUIRES(SpeedInfo),
  //REQUIRES(TargetCoM),
  PROVIDES(BodyTilt),
});

class TiltControllerModule : public TiltControllerModuleBase
{
public:
	TiltControllerModule();

	TiltController controller;

	void update(BodyTilt &theBodyTilt);
};



