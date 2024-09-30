/**
  * @file FLIPMObservedStateProvider.h
  * @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
  */

#pragma once
#include <list>
#include "Representations/MotionControl/FLIPMObservedState.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Tools/Module/Module.h"

/**
* @class FLIPMObservedStateProvider
* Determines the actual position of the center of mass used in FLIPM
*/

MODULE(FLIPMObservedStateProvider,
  REQUIRES(JoinedIMUData),
  REQUIRES(RobotModel),
  USES(WalkingInfo),
  PROVIDES(FLIPMObservedState),
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource
  )
);

class FLIPMObservedStateProvider : public FLIPMObservedStateProviderBase
{
public:
  FLIPMObservedStateProvider(){};
  ~FLIPMObservedStateProvider(void){};

  void update(FLIPMObservedState& theFLIPMObservedState);
};
