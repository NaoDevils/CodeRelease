#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Tools/RingBuffer.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Tools/Math/angleerror.hpp"

MODULE(LimbCombinator,
  REQUIRES(SpeedRequest),
  REQUIRES(Footpositions),
  REQUIRES(TargetCoM),
  REQUIRES(SpeedInfo),
  REQUIRES(KinematicOutput),
  REQUIRES(ArmMovement),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingInfo),
  REQUIRES(WalkingEngineParams),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(,
    (float)(0.2f) hipPitchKneeCompensation
  )
);

class LimbCombinator : public LimbCombinatorBase
{
public:
  LimbCombinator(void);
  ~LimbCombinator(void);

  void applyKickHackHip(WalkingEngineOutput& walkingEngineOutput);
  void applyKickHackKnee(WalkingEngineOutput& walkingEngineOutput);
  void applyKickHackKneeReverse(WalkingEngineOutput& walkingEngineOutput);
  void applyAnkleCompensation(WalkingEngineOutput& walkingEngineOutput);

private:
  AngleError angerr;
  float angleoffset[Joints::numOfJoints][200];
  RingBuffer<float, 100> delayBuffer[Joints::numOfJoints];
  void update(WalkingEngineOutput& walkingEngineOutput);
  bool init;
  int lastOutFilterOrder;
  FastFilter<float> filter[Joints::numOfJoints];

  RingBuffer<Angle, MAX_DELAY_FRAMES> lHipPitchTargetAngle, rHipPitchTargetAngle;
};
