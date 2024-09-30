#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"


MODULE(WalkParamsProvider,
  USES(MotionState),
  USES(WalkCalibration),
  REQUIRES(BehaviorData),
  PROVIDES(WalkingEngineParams),
  PROVIDES(SensorControlParams),
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
  std::string annotation = "";
  bool initializedWP = false;
  bool initializedSCP = false;
  WalkingEngineParams originalParams;

  bool lastXForwardUpdated, lastXBackwardUpdated, lastYUpdated = false;
  bool lastWalkCalibrated = false;

  void update(WalkingEngineParams& walkingEngineParams);
  void update(SensorControlParams& sensorControlParams);
  void load(WalkingEngineParams& walkingEngineParams, bool original = false);
  void load(SensorControlParams& sensorControlParams);

  void setMinMax(WalkingEngineParams& walkingEngineParams);
};
