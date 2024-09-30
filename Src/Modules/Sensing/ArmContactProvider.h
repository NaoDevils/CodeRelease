#ifndef __ArmContactProvider_
#define __ArmContactProvider_

#include "Tools/Module/Module.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/RingBufferWithSum.h"

MODULE(ArmContactProvider,
  USES(ArmMovement),
  USES(RawJointRequest),
  REQUIRES(BallSymbols),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(JoinedIMUData),
  REQUIRES(JointSensorData),
  REQUIRES(WalkingEngineParams),
  REQUIRES(MotionSelection),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(BallChaserDecision),
  REQUIRES(RobotInfo),
  PROVIDES(ArmContact),
  LOADS_PARAMETERS(,
    (bool)(true) enableAvoidArmContact,
    (bool)(true) useRobotMap,
    (bool)(true) useArmPitchDiff,
    (float)(350.f) maxRobotDist,
    (Angle)(150_deg) maxAngleToRobot,
    (Angle)(30_deg) minAngleToRobot,
    (Angle)(100_deg) maxSum,
    (Angle)(10_deg) minAngleDiff,
    (bool)(true) bothArmsBack,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource
  )
);

class ArmContactProvider : public ArmContactProviderBase
{
public:
  ArmContactProvider(void);
  ~ArmContactProvider(void);

private:
  void update(ArmContact& armContact);

  void resetBuffers();

  ArmContact localArmContact;
  RingBuffer<JointRequest, MAX_DELAY_FRAMES> requestSensorDelayBuffer;
  RingBufferWithSum<Angle, 186> bufferLeft; // 2 seconds in Motion frames
  RingBufferWithSum<Angle, 186> bufferRight; // 2 seconds in Motion frames
  Angle lastPitchLeft, lastPitchRight;
  bool falling = false;

  JointRequest lastRequest;
  unsigned timeWhenWalkStarted = 0;
  MotionRequest::Motion lastMotionType = MotionRequest::Motion::specialAction;
};

#endif // __ArmContactProvider_
