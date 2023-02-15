/**
* \file BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* The ball member variables are the ones the robot's decisions are based on.
* \author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"

/**
* \class BehaviorData
* A class that containts data about the current behavior state.
*/
STREAMABLE(BehaviorData,
  ENUM(BehaviorState,
    frameworkInactive,
    game,
    penaltyShootout,
    // below here only calibration states!
    firstCalibrationState,
    calibrationStarted = firstCalibrationState,
    calibrateBody,
    calibrateCameraMatrix,
    calibrateWalk
  );

  ENUM(RoleAssignment,
    noRole,
    firstRole,
    keeper = firstRole,
    defenderRight,
    defenderLeft,
    defenderSingle,
    backupBallchaser,
    ballchaser,
    ballchaserKeeper,
    replacementKeeper,
    center,
    receiver
  );
  
  ENUM(SoccerState,
    waiting,
    penalized,
    lost,
    numOfUnsafeActions,
    pass = numOfUnsafeActions,
    controlBall,
    positioning,
    searchForBall,
    standUp
  );

  typedef std::vector<RoleAssignment> RobotRoleAssignmentVector;
  
  STREAMABLE(PassiveRolePosition,
    PassiveRolePosition() = default;
    PassiveRolePosition(BehaviorData::RoleAssignment _role, Vector2f _position)
    {
      role = _role;
      position = _position;
    }
    ,
    ((BehaviorData) RoleAssignment)(BehaviorData::defenderSingle) role,
    (Vector2f)(Vector2f::Zero()) position
  );

  typedef std::vector<PassiveRolePosition> PassiveRolePositionVector;
  ,
  (BehaviorState)(game) behaviorState, /**< The current Behavior status. */
  (SoccerState)(waiting) soccerState, /**< What is the robot doing in general? */
  (RoleAssignment)(noRole) role, /**< A dynamically chosen role. */
  (RoleAssignment)(noRole) lastRole, /**< For role sync. */
  (std::vector<RoleAssignment>) roleSuggestions,
  (int)(1) playerNumberToBall,
  (float)(100000.f) ownTimeToBall, /**< Can be either distance or time to ball based on method used. */
  (Vector2f)(Vector2f::Zero()) ballPositionRelative, /**< The robot's relative ball position w/preview. */
  (Vector2f)(Vector2f::Zero()) ballPositionField, /**< The robot's ball position on field. */
  (Vector2f)(Vector2f::Zero()) ballPositionFieldPredicted, /**< The robot's predicted ball position on field. */
  (int)(1000000) timeSinceBallWasSeen, /**< Time since the robot last saw the ball himself. */
  (Vector2f)(Vector2f(1000,0))  kickTarget, /**< The kick target of the robot. */
  (std::vector<PassiveRolePosition>) passiveRolePositions // positions this player would take if selected for it
);

STREAMABLE(BehaviorControlOutput,,
  (BehaviorData) behaviorData
);
