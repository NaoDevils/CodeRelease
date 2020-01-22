#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Path.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FallDownAngleReduction.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Tools/Module/Module.h"
#include "SimplePathProvider.h" // for parameters

MODULE(PathToSpeedStable,
{,
  REQUIRES(BallSymbols),
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RoleSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(PositioningSymbols),
  REQUIRES(Path),
  PROVIDES(SpeedRequest),
  REQUIRES(WalkingEngineParams),
  REQUIRES(BehaviorConfiguration),
  REQUIRES(GameSymbols),
  REQUIRES(FallDownAngleReduction),
  REQUIRES(TeammateData),
  LOADS_PARAMETERS(
  {,
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
    (float)(0.7f) influenceOfObstacleOnTranslationSetPlay,
    (float)(2.0f) emergencyStopFallDownReductionFactorThreshold,
  }),
});

class PathToSpeedStable : public PathToSpeedStableBase
{
public:
  ENUM(PathFollowState,
  { ,
    omni, // near obstacles, omnidirectional avoidance
    target, // near target: keep in sight, turn to target rotation
    far,
  }); // default: high speed

  PathToSpeedStable();

private:

  PathFollowState state = far;
  bool inDribbling = false;
  bool atBall = false;
  SimplePathProviderBase::Params pathParameters;

  void update(SpeedRequest& speedRequest);
};
