/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPControllerNGModule.h
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/Sensing/ZMPModel.h"
#include "ZMPIPControllerNG.h"
#include "Representations/MotionControl/PatternGenRequest.h"

MODULE(ZMPIPControllerNGModule,
{ ,
  REQUIRES(RobotModel),
  REQUIRES(InertialSensorData),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingEngineParams),
  REQUIRES(FallDownState),
  REQUIRES(ActualCoM),
  REQUIRES(PatternGenRequest),
  REQUIRES(ControllerParams),
  REQUIRES(ZMPModel),
  REQUIRES(RefZMP),
  USES(WalkingInfo),
  PROVIDES(TargetCoM),
});

class ZMPIPControllerNGModule : public ZMPIPControllerNGModuleBase 
{
public:
  ZMPIPControllerNGModule();

	ZMPIPControllerNG controller;

	void update(TargetCoM &theTargetCoM);
};


