#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/MotionSelection.h"

MODULE(ConstantArmProvider,
{ ,
  REQUIRES(MotionSelection),
  PROVIDES(ArmMovement),
});

class ConstantArmProvider : public ConstantArmProviderBase
{
public:
	ConstantArmProvider(void);
	~ConstantArmProvider(void);
private:
	void update(ArmMovement& armMovement);
	float intfac;
};
