#pragma once

#include "Representations/MotionControl/ControllerParams.h"
#include "Tools/Module/Module.h"


MODULE(ControllerParams2012Provider,
{ ,
  PROVIDES(ControllerParams),
  DEFINES_PARAMETERS(
  {,
    (std::string)("ZMPIPController2012.dat") paramFileName,
  }),
});

class ControllerParams2012Provider : public ControllerParams2012ProviderBase
{
public:
  ControllerParams2012Provider();

protected:
	void update(ControllerParams& controllerParams);
  int load();
  ControllerParams contParams;
};

