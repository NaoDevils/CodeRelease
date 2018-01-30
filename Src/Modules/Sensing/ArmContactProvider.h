#ifndef __ArmContactProvider_
#define __ArmContactProvider_

#include "Tools/Module/Module.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/RingBufferWithSum.h"

MODULE(ArmContactProvider,
{ ,
  USES(ArmMovement),
  USES(JointRequest),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(InertialSensorData),
  REQUIRES(JointSensorData),
  REQUIRES(MotionSelection),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  PROVIDES(ArmContact),
  LOADS_PARAMETERS(
  {,
    (bool) enableAvoidArmContact,
    (bool) useRobotMap,
    (bool) useArmPitchDiff,
    (float) maxRobotDist,
    (Angle) maxSum,
    (Angle) minAngleDiff,
  }),
});

class ArmContactProvider : public ArmContactProviderBase
{
public:
  
	ArmContactProvider(void);
	~ArmContactProvider(void);

private:
	void update(ArmContact& armContact);

  void resetBuffers();

  ArmContact localArmContact;
  RingBufferWithSum<Angle,200> bufferLeft;
  RingBufferWithSum<Angle,200> bufferRight;
  Angle lastPitchLeft, lastPitchRight;
  bool falling; //FIXME: Value is used uninitialized

  JointRequest lastRequest;
  unsigned timeWhenWalkStarted;
  MotionRequest::Motion lastMotionType;
};

#endif // __ArmContactProvider_
