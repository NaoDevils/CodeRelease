#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Sensing/ArmContact.h"
#include <algorithm>

MODULE(ArmAnimator,
{ ,
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingEngineParams),
  REQUIRES(FreeLegPhaseParams),
  REQUIRES(KinematicRequest),
  REQUIRES(WalkingInfo),
  REQUIRES(BodyTilt),
  REQUIRES(ArmContact),
  PROVIDES(ArmMovement),
  LOADS_PARAMETERS(
  {,
    (Angle) wristOffset,
    (Angle) handOffset,
    (unsigned) timeToHoldArmBack, // Same value as in ArmContactProvider!
    (Angle) armBackPitch,
    (Angle) armBackElbowRoll,
    (unsigned) timeToPullPitch,
    (unsigned) timeToPullArmIn,
  }),
});

class ArmAnimator : public ArmAnimatorBase
{
public:
  ArmAnimator(void);
  ~ArmAnimator(void);

private:
  void update(ArmMovement& armMovement);

  ArmContact::ArmContactState lastContactStateLeft, lastContactStateRight;
  unsigned leftTimeWhenContactStateChanged, rightTimeWhenContactStateChanged;
  float leftPitchWhenContactStateChanged;
  float rightPitchWhenContactStateChanged;
};
