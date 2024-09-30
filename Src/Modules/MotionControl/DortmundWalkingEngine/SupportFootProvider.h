/**
 * @file SupportFootProvider.h
 * This file declares a module that ...
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

#include "Representations/Infrastructure/FsrModelData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SupportFoot.h"

MODULE(SupportFootProvider,
  REQUIRES(WalkingEngineParams),    
  REQUIRES(FsrModelData),    
  REQUIRES(FsrSensorData),    
  PROVIDES(SupportFoot),
  LOADS_PARAMETERS(,
    (bool)(false) useFSRModel,
    (float)(7.5f) supportFootThresholdFactorFootYDistance
  )
);

class SupportFootProvider : public SupportFootProviderBase
{
private:
  void update(SupportFoot& theSupportFoot);
  RingBufferWithSum<float, 4> supportFootBuffer;
};
