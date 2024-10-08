#pragma once

#include "Modules/Modeling/PathProvider/SimplePathProvider.h" // for parameters
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/LiveConfigurationState.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/Path.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Tools/Module/Module.h"

MODULE(PathToSpeedPressToStop,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(LiveConfigurationState),
  REQUIRES(MotionRequest),
  REQUIRES(RoleSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(Path),
  PROVIDES(SpeedRequest),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  REQUIRES(GameSymbols),
  REQUIRES(MotionState),
  REQUIRES(TeammateData),
  LOADS_PARAMETERS(,
    (float)(0.75f) speedPercentageWhenNotChasingBall, /** fixed percentage of speed for all robots who are not going to the ball except goalie */
    (bool)(false) useDistanceBasedSpeedPercentageInReady, /** dynamic speed percentage, depending on distance to walk in ready state; TODO: any timed state? */
    (float)(0.6f) minSpeedPercentageInReady,
    (float) targetStateSwitchDistance,
    (float) targetStateSwitchDistanceHysteresis,
    (float) omniStateSwitchDistance,
    (float) omniStateSwitchDistanceHysteresis,
    (float) omniStateSwitchAngle,
    (float) walkBackWardsDistance,
    (float) walkBackWardsRotation,
    (bool) keeperInGoalAreaOmniOnly,
    (float)(0.5f) walkAroundBallTranslationRotationFactor,
    (float)(300.f) influenceRadiusOfObstacleOnTranslation,
    (float)(0.5f) influenceOfObstacleOnTranslation, // [this..0]*<above radius>
    (float)(0.8f) influenceOfObstacleOnTranslationTeammate,
    (float)(0.7f) influenceOfObstacleOnTranslationCenterCircle,
    (float)(0.7f) influenceOfObstacleOnTranslationSetPlay
  )
);

class PathToSpeedPressToStop : public PathToSpeedPressToStopBase
{
public:
  ENUM(PathFollowState,
    omni, // near obstacles, omnidirectional avoidance
    target, // near target: keep in sight, turn to target rotation
    far
  ); // default: high speed

  PathToSpeedPressToStop();

private:
  PathFollowState state = far;
  bool inDribbling = false;
  bool atBall = false;
  SimplePathProviderBase::Params pathParameters;
  bool stopRequestedByButton = false;

  void update(SpeedRequest& speedRequest);

  /** Toggle between stop and normal path to speed when live config is active and front head button is pressed. */
  void handleStopRequest();
};
