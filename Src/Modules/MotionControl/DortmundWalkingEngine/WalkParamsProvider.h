#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FLIPMControllerParams.h"
#include "Representations/MotionControl/FLIPMObserverParams.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"


MODULE(WalkParamsProvider,
{ ,
  PROVIDES(WalkingEngineParams),
  PROVIDES(FLIPMControllerParams),
  PROVIDES(LegJointSensorControlParameters),
  PROVIDES(FLIPMObserverParams),
});

class WalkParamsProvider : public WalkParamsProviderBase
{
  ROBOT_PARAMETER_CLASS(walkParamsProvider, WalkParamsProvider)
    PARAM(bool, useRCS)
    PARAM(ParamsFLIPM, paramsX)
    PARAM(ParamsFLIPM, paramsY)
    END_ROBOT_PARAMETER_CLASS(walkParamsProvider)

public:
  WalkParamsProvider() = default;

protected:
  bool initializedWP = false;
  bool initializedLJSCP = false;
  bool initializedFOP = false;
  void update(WalkingEngineParams& walkingEngineParams);
  void update(FLIPMControllerParams & flipmControllerParams)
  {
    paramswalkParamsProvider.handle();
    flipmControllerParams.useRCS = paramswalkParamsProvider.useRCS;
    flipmControllerParams.paramsX = paramswalkParamsProvider.paramsX;
    flipmControllerParams.paramsY = paramswalkParamsProvider.paramsY;
  }
  void update(FLIPMObserverParams& flipmObserverParams);
  void update(LegJointSensorControlParameters& legJointSensorControlParameters);
  void load(WalkingEngineParams& walkingEngineParams);
  void load(FLIPMObserverParams& flipmObserverParams);
  void load(LegJointSensorControlParameters& legJointSensorControlParameters);
};

