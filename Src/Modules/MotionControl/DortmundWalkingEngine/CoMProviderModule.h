/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CoMProviderModule.h
* Module wrapper for the CoMProvider
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingInfo.h"
#include "Tools/Module/Module.h"
#include "CoMProvider.h"
#include "Representations/MotionControl/ActualCoM.h"

MODULE(CoMProviderModule,
  //REQUIRES(JointSensorData),
  //REQUIRES(FootSteps),
  //REQUIRES(RobotModel),
  REQUIRES(ActualCoMRCS),
  USES(WalkingInfo),
  //USES(RawJointRequest),
  //REQUIRES(WalkingEngineParams),
  PROVIDES(ActualCoM)
);

class CoMProviderModule : public CoMProviderModuleBase
{
public:
  CoMProviderModule();

  CoMProvider controller;

  void update(ActualCoM& theActualCoM);
};
