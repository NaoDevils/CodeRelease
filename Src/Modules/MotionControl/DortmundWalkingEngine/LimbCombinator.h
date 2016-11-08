#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/RingBuffer.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Math/angleerror.hpp"

MODULE(LimbCombinator,
{ ,
  REQUIRES(SpeedInfo),
  REQUIRES(KinematicOutput),
  REQUIRES(ArmMovement),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingInfo),
  REQUIRES(MotionRequest),
  REQUIRES(WalkingEngineParams),
  REQUIRES(FreeLegPhaseParams),
  PROVIDES(WalkingEngineOutput),
});

class LimbCombinator : public LimbCombinatorBase
{
public:
	LimbCombinator(void);
	~LimbCombinator(void); 

	static int walkingEngineTime;
private:
	AngleError angerr;
	float getOffset(int j, float targetAngle);
	float angleoffset[Joints::numOfJoints][200];
	RingBuffer<float, 100> delayBuffer[Joints::numOfJoints];
	void update(WalkingEngineOutput& walkingEngineOutput);
	bool init;
	FastFilter<float> filter[Joints::numOfJoints];
};
