#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FLIPMObserverGains.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"


MODULE(WalkParamsProvider,
{ ,
  PROVIDES(WalkingEngineParams),
  PROVIDES(LegJointSensorControlParameters),
  PROVIDES(FLIPMObserverGains),
});

class WalkParamsProvider : public WalkParamsProviderBase
{
public:
  WalkParamsProvider() = default;

protected:
  bool initializedWP = false;
  bool initializedLJSCP = false;
  bool initializedFOP = false;
  void update(WalkingEngineParams& walkingEngineParams);
  void update(FLIPMObserverGains& flipmObserverGains);
  void update(LegJointSensorControlParameters& legJointSensorControlParameters);
  void load(WalkingEngineParams& walkingEngineParams);
  void load(FLIPMObserverGains& flipmObserverGains);
  void load(LegJointSensorControlParameters& legJointSensorControlParameters);
};

