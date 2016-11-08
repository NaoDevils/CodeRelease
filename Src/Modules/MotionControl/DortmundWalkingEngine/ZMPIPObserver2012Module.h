/** 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de"> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/ZMPModel.h"
#include "ZMPIPControllerNG.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "ZMPIPObserver2012.h"

MODULE(ZMPIPObserver2012Module,
{ ,
  REQUIRES(WalkingEngineParams),
  REQUIRES(ActualCoM),
  REQUIRES(PatternGenRequest),
  REQUIRES(ControllerParams),
  REQUIRES(ZMPModel),
  USES(WalkingInfo),
  USES(TargetCoM),
  PROVIDES(ObservedError),
});

class ZMPIPObserver2012Module : public ZMPIPObserver2012ModuleBase 
{
public:
  ZMPIPObserver2012Module();

  ZMPIPObserver2012 observer;

  void update(ObservedError &theObservedError);
};


