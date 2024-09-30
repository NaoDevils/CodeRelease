#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/ArmContact.h"
#include <algorithm>

MODULE(ArmAnimator,
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(SensorControlParams),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSelection),
  REQUIRES(KinematicRequest),
  REQUIRES(JoinedIMUData),
  REQUIRES(ArmContact),
  PROVIDES(ArmMovement),
  LOADS_PARAMETERS(,
    (Angle) wristOffset,
    (Angle) handOffset,
    (int) timeToHoldArmBack,
    (Angle) armBackPitch,
    (Angle) armBackElbowRoll,
    (Angle) armBackShoulderRoll,
    (Angle) armToFrontShoulderRoll,
    (int) timeToPullPitch,
    (int) timeToPullArmIn,
    (int) timeUntilArmContactActive,
    (bool)(false) useBodyTiltForArmMovement,
    (float)(0.005f) armFactor,
    (Angle)(10_deg) armsAngle
  )
);

class ArmAnimator : public ArmAnimatorBase
{
public:
  ArmAnimator(void);
  ~ArmAnimator(void);

private:
  void update(ArmMovement& armMovement);

  unsigned leftTimeWhenNoContact, rightTimeWhenNoContact;
  unsigned leftTimeWhenContact, rightTimeWhenContact;
  ArmContact::ArmContactState lastContactStateLeft, lastContactStateRight;
  float leftPitchWhenContactStateChanged;
  float rightPitchWhenContactStateChanged;

  MotionRequest::Motion lastMotion;
  unsigned timeWhenChangedToWalk;
};
