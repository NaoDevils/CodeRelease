#pragma once

#include "Representations/MotionControl/ControllerParams.h"
#include "Tools/Module/Module.h"


MODULE(ControllerParamsNGProvider,
{ ,
  PROVIDES(ControllerParams),
  DEFINES_PARAMETERS(
  { ,
    (std::string)("ZMPIPController2012.dat") paramFileName,
  }),
});

class ControllerParamsNGProvider : public ControllerParamsNGProviderBase
{
public:
  ControllerParamsNGProvider();

protected:
	void update(ControllerParams& controllerParams);
  int load();
  ControllerParams contParams;
};

