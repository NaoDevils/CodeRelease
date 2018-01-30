#pragma once


#include "Tools/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/PatternGenRequest.h"

MODULE(CoMRCSProvider,
{ ,
  REQUIRES(RobotModel),
  REQUIRES(PatternGenRequest),
  PROVIDES(ActualCoMRCS),
  LOADS_PARAMETERS(
  {,
    (bool)(true) walkFixedCoM,
  }),
});

class CoMRCSProvider : public CoMRCSProviderBase
{
public:
  void update(ActualCoMRCS &actualCoMRCS);
  
};