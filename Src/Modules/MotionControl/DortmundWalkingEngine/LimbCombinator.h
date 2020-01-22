#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Tools/RingBuffer.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Math/angleerror.hpp"

MODULE(LimbCombinator,
{ ,
  REQUIRES(FrameInfo),
  REQUIRES(SpeedRequest),
  REQUIRES(Footpositions),
  REQUIRES(FootSteps),
  REQUIRES(TargetCoM),
  REQUIRES(SpeedInfo),
  REQUIRES(KinematicOutput),
  REQUIRES(ArmMovement),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingInfo),
  REQUIRES(WalkingEngineParams),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (int)(5) outFilterOrderSize,
    (bool)(false) useKickHack,
  }),
});

class LimbCombinator : public LimbCombinatorBase
{
public:
	LimbCombinator(void);
  ~LimbCombinator(void);
    
  void applyKickHack(WalkingEngineOutput& walkingEngineOutput); 

	static int walkingEngineTime;
private:
	AngleError angerr;
	float getOffset(int j, float targetAngle);
	float angleoffset[Joints::numOfJoints][200];
	RingBuffer<float, 100> delayBuffer[Joints::numOfJoints];
	void update(WalkingEngineOutput& walkingEngineOutput);
	bool init;
	FastFilter<float> filter[Joints::numOfJoints];

  // kick hack -> fixed joints while kicking
  unsigned int timeStampKickStarted = 0;
};
