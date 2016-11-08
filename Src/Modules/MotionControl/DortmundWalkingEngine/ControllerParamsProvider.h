#pragma once

#include "Representations/MotionControl/ControllerParams.h"
#include "Tools/Module/Module.h"


MODULE(ControllerParamsProvider,
{ ,
  PROVIDES(ControllerParams),
  DEFINES_PARAMETERS(
  { ,
    (std::string)("ZMPIPController2012.dat") paramFileName,
  }),
});

class ControllerParamsProvider : public ControllerParamsProviderBase
{
public:
  ControllerParamsProvider();

protected:
	void update(ControllerParams& controllerParams);
  int load();
  ControllerParams contParams;
};

