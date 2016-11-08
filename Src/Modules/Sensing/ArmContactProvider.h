#ifndef __ArmContactProvider_
#define __ArmContactProvider_

#include "Tools/Module/Module.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/RingBufferWithSum.h"

MODULE(ArmContactProvider,
{ ,
  USES(JointRequest),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(InertialSensorData),
  REQUIRES(JointSensorData),
  REQUIRES(MotionRequest),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  PROVIDES(ArmContact),
  LOADS_PARAMETERS(
  {,
    (bool) enableAvoidArmContact,
    (bool) useRobotMap,
    (bool) useArmPitchDiff,
    (float) maxRobotDist,
    (float) maxSum,
    (Angle) minAngleDiff,
    (unsigned) timeToHoldArmBack,
  }),
});

class ArmContactProvider : public ArmContactProviderBase
{
public:
  
	ArmContactProvider(void);
	~ArmContactProvider(void);

private:
	void update(ArmContact& armContact);

  ArmContact localArmContact;
  int directionLeft, directionRight;
  RingBufferWithSum<float,200> bufferLeftFront, bufferLeftBack;
  RingBufferWithSum<float,200> bufferRightFront, bufferRightBack;
  RingBufferWithSum<int,4> bufferDirectionLeft, bufferDirectionRight;
  float lastPitchLeft, lastPitchRight;
  bool falling; //FIXME: Value is used uninitialized

  JointRequest lastRequest;
};

#endif // __ArmContactProvider_
