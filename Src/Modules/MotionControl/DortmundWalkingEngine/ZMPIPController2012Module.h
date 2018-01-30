/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPController2012Module.h
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/RefZMP.h"
#include "ZMPIPController2012.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/ReferenceModificator.h"

MODULE(ZMPIPController2012Module,
{,
  REQUIRES(WalkingEngineParams),
  REQUIRES(ControllerParams),
  REQUIRES(ObservedError),
  REQUIRES(RefZMP),
  REQUIRES(ReferenceModificator),
  PROVIDES(TargetCoM),
});

class ZMPIPController2012Module : public ZMPIPController2012ModuleBase 
{
public:
  ZMPIPController2012Module();
	ZMPIPController2012 controller;
	void update(TargetCoM &theTargetCoM);
};


