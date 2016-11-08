#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"

MODULE(OrientationArmAnimator,
{ ,
  REQUIRES(InertialSensorData),
  REQUIRES(WalkingEngineParams),
  PROVIDES(ArmMovement),
});

class OrientationArmAnimator : public OrientationArmAnimatorBase
{
public:
	OrientationArmAnimator(void);
	~OrientationArmAnimator(void);
private:
	void update(ArmMovement& armMovement);
};
