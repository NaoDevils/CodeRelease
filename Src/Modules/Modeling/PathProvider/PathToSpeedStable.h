#pragma once

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
#include "Tools/Module/Module.h"

MODULE(PathToSpeedStable,
{,
  REQUIRES(BallModel),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotInfo),
  REQUIRES(RobotMap),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(Path),
  PROVIDES(SpeedRequest),
  REQUIRES(WalkingEngineParams),
  LOADS_PARAMETERS(
  {,
    (float) targetStateSwitchDistance,
    (float) targetStateSwitchDistanceHysteresis,
    (float) omniStateSwitchDistance,
    (float) omniStateSwitchDistanceHysteresis,
    (float) omniStateSwitchAngle,
    (float) walkBackWardsDistance,
    (float) walkBackWardsRotation,
    (bool) keeperInGoalAreaOmniOnly,
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

  PathFollowState state;
  bool inOwnGoalArea;
  bool inDribbling;

  void update(SpeedRequest& speedRequest);
};
