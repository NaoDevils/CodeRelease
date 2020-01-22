/**
  * @file FLIPMObservedStateProvider.h
  * @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
  */

#pragma once
#include <list>
#include "Representations/MotionControl/FLIPMObservedState.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/IMUModel.h"

/**
* @class FLIPMObservedStateProvider
* Determines the actual position of the center of mass used in FLIPM
*/

MODULE(FLIPMObservedStateProvider,
{ ,
  REQUIRES(FLIPMObserverParameter),
  REQUIRES(InertialSensorData),
  REQUIRES(RobotModel),
  REQUIRES(IMUModel),
  USES(WalkingInfo),
  PROVIDES(FLIPMObservedState),
});

class FLIPMObservedStateProvider : public FLIPMObservedStateProviderBase
{
public:
  FLIPMObservedStateProvider() {};
  ~FLIPMObservedStateProvider(void) {};

  void update(FLIPMObservedState &theFLIPMObservedState);
};  

