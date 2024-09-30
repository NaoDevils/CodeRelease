#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/Path.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "SimplePathProvider.h" // for parameters
#include "Tools/Module/Module.h"

MODULE(PathToSpeedStable,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(MotionState),
  REQUIRES(RoleSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(Path),
  PROVIDES(SpeedRequest),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  REQUIRES(JoinedIMUData),
  REQUIRES(GameSymbols),
  REQUIRES(TeammateData),
  REQUIRES(Footpositions),
  LOADS_PARAMETERS(,
    (float)(0.75f) speedPercentageWhenNotChasingBall, /** fixed percentage of speed for all robots who are not going to the ball except goalie */
    (float)(0.5f) speedPercentageCalibration, /** fixed percentage of speed for calibration goto */
    (bool)(false) useDistanceBasedSpeedPercentageInReady, /** dynamic speed percentage, depending on distance to walk in ready state; TODO: any timed state? */
    (float)(0.6f) minSpeedPercentageInReady,
    (float) targetStateSwitchDistanceFieldplayer,
    (float) targetStateSwitchDistanceKeeper,
    (float) targetStateSwitchDistanceHysteresis,
    (float) omniStateSwitchDistance,
    (float) omniStateSwitchDistanceHysteresis,
    (float) omniStateSwitchAngle,
    (float) walkBackWardsDistance,
    (float) walkBackWardsRotation,
    (bool)(true) keeperInPenaltyAreaTargetOnly,
    (float)(200.f) walkAroundBallMinDistance,
    (float)(300.f) walkAroundBallMaxDistance,
    (float)(300.f) influenceRadiusOfObstacleOnTranslation,
    (float)(0.5f) influenceOfObstacleOnTranslation, // [this..0]*<above radius>
    (float)(0.8f) influenceOfObstacleOnTranslationTeammate,
    (float)(0.8f) influenceOfObstacleOnTranslationBallchaser,
    (float)(0.7f) influenceOfObstacleOnTranslationCenterCircle,
    (float)(0.7f) influenceOfObstacleOnTranslationSetPlay
  )
);

class PathToSpeedStable : public PathToSpeedStableBase
{
public:
  ENUM(PathFollowState,
    omni, // near obstacles, omnidirectional avoidance
    target, // near target: keep in sight, turn to target rotation
    far
  ); // default: high speed

  PathToSpeedStable();

private:
  std::array<RingBufferWithSum<Angle, 15>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersX;
  std::array<RingBufferWithSum<Angle, 15>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersY;
  PathFollowState state = far;
  bool inDribbling = false;
  bool atBall = false;
  bool rotateAroundBall = false;

  bool wasInEmergencyStop = false;

  SimplePathProviderBase::Params pathParameters;

  void update(SpeedRequest& speedRequest);
};
