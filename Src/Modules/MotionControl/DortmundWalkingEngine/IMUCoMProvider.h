/**
  * @file IMUCoMProvider.h
  * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
  */

#pragma once
#include <list>
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "StepData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Tools/Module/Module.h"

/**
* @class IMUCoMProvider
* Determines the actual position of the center of mass
* using the orientation given by naoqi.
*/

MODULE(IMUCoMProvider,
{ ,
  REQUIRES(FootSteps),
  REQUIRES(InertialSensorData),
  REQUIRES(WalkingEngineParams),
  USES(WalkingInfo),
  USES(TargetCoM),
  PROVIDES(ActualCoM),
});

class IMUCoMProvider : public IMUCoMProviderBase
{
public:
  IMUCoMProvider() {};

  /** Destructor */
  ~IMUCoMProvider(void) {};

  void update(ActualCoM &theActualCoM);

};

