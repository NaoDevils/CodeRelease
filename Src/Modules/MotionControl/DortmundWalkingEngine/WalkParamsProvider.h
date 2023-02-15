#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FLIPMObserverGains.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"


MODULE(WalkParamsProvider,
  USES(MotionState),
  USES(WalkCalibration),
  PROVIDES(WalkingEngineParams),
  PROVIDES(FLIPMObserverGains),
  LOADS_PARAMETERS(,
    (float)(80.f) minXForward,
    (float)(300.f) maxXForward,
    (float)(70.f) minXForwardOmni,
    (float)(280.f) maxXForwardOmni,
    (float)(60.f) minXForwardArmContact,
    (float)(250.f) maxXForwardArmContact,
    (float)(50.f) minXBackward,
    (float)(200.f) maxXBackward,
    (float)(60.f) minY,
    (float)(250.f) maxY,
    (float)(50.f) minYArmContact,
    (float)(200.f) maxYArmContact,
    (bool)(false) speakOutChanges
  )
);

class WalkParamsProvider : public WalkParamsProviderBase
{
public:
  WalkParamsProvider() = default;

protected:
  bool initializedWP = false;
  bool initializedFOP = false;
  WalkingEngineParams originalParams;

  bool lastXForwardUpdated, lastXBackwardUpdated, lastYUpdated = false;

  void update(WalkingEngineParams& walkingEngineParams);
  void update(FLIPMObserverGains& flipmObserverGains);
  void load(WalkingEngineParams& walkingEngineParams);
  void load(FLIPMObserverGains& flipmObserverGains);
};
