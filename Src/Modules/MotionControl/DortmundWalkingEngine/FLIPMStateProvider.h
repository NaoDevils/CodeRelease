/**
 * @file FLIPMStateProvider.h
 * @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
//#include "Tools/Streams/RobotParameters.h"
//#include "Representations/MotionControl/FLIPMObservedState.h"
//#include "Representations/MotionControl/FootSteps.h"
//#include "Representations/Configuration/RobotDimensions.h"
//#include "Representations/MotionControl/Footpositions.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"

STREAMABLE(StateProvider,
  ENUM(CoMProvider,
    targetCoM,
    MRECoM,
    IMUCoM
  );
  ENUM(VelProvider,
    integratedCoM
  );
  ENUM(AccProvider,
    targetAcc,
    ZMPAcc,
    ZMPCoM1Acc,
    ZMPTargetAcc,
    IMUAcc,
    integratedAcc
  ),
  (CoMProvider) CoM1Provider,
  (VelProvider) Vel1Provider,
  (AccProvider) Acc1Provider,
  (CoMProvider) CoM2Provider,
  (VelProvider) Vel2Provider,
  (AccProvider) Acc2Provider
);

STREAMABLE(FLIPMStateProviderParameter,,
    (float)(1.f) gainsFactor,
    (Vector6f)(Vector6f::Ones()) gains,
    (Vector6f)(Vector6f::Ones()) scale,
    (Vector6f)(Vector6f::Zero()) offset
);

MODULE(FLIPMStateProvider,
  REQUIRES(WalkCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(WalkingEngineParams),
  REQUIRES(ZMPModel),
  REQUIRES(FLIPMParameter),
  REQUIRES(JoinedIMUData),
  REQUIRES(RobotModel),
  USES(WalkingInfo),
  USES(TargetCoM),
  PROVIDES(ObservedFLIPMError),
  LOADS_PARAMETERS(,
    (bool)(false) useRCS,
    ((JoinedIMUData)InertialDataSource)(JoinedIMUData::imuModel) anglesource,
    (float)(0.f) acc1Filter,
    (float)(0.f) acc2Filter,
    (float)(0.f) vel1Filter,
    (float)(0.f) vel2Filter,
    (FLIPMStateProviderParameter) xParams,
    (FLIPMStateProviderParameter) yParams,
    (Vector6i)(Vector6i::Zero()) delays,
    (StateProvider) providers
  )
);

class FLIPMStateProvider : public FLIPMStateProviderBase
{
public:
  FLIPMStateProvider();
  void update(ObservedFLIPMError& observedFLIPMError);

private:
  void updateSensorData();
  void declarePlots();

  Vector2f updateCoM1(Pose2f& robotPosition);
  Vector2f updateVel1(Pose2f& robotPosition);
  Vector2f updateAcc1(Pose2f& robotPosition);
  Vector2f updateCoM2(Pose2f& robotPosition);
  Vector2f updateVel2(Pose2f& robotPosition);
  Vector2f updateAcc2(Pose2f& robotPosition);

  Vector2f targetCoM1WCS, targetVel1WCS, targetAcc1WCS, targetCoM2WCS, targetVel2WCS, targetAcc2WCS;
  Vector2f filteredCoM1, filteredVel1, filteredAcc1, filteredCoM2, filteredVel2, filteredAcc2;

  RingBuffer<Vector2f, PREVIEW_LENGTH> /**< Buffer to deal with the sensor delay. */
      targetCoM1DelayBuffer, targetVel1DelayBuffer, targetAcc1DelayBuffer, targetCoM2DelayBuffer, targetVel2DelayBuffer, targetAcc2DelayBuffer, coM1DelayBuffer, vel1DelayBuffer,
      acc1DelayBuffer, coM2DelayBuffer, vel2DelayBuffer, acc2DelayBuffer;

  Vector3f CoM_MRE, CoM_IMU, Acc_IMU, Acc_ZMP;
};
