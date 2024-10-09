/**
* \file BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* The ball member variables are the ones the robot's decisions are based on.
* \author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"
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
    testingJoints,
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
    replacementKeeper,
    center,
    receiver,
    leftWing,
    rightWing,
    frontWing,
    backWing,
    remoteControl
  );
  
  ENUM(SoccerState,
    waiting,
    penalized,
    lost,
    safetyShutdown,
    numOfUnsafeActions,
    pass = numOfUnsafeActions,
    controlBall,
    positioning,
    searchForBall,
    standUp
  );

  using RobotRoleAssignmentVector = std::vector<RoleAssignment>;
  using RobotRoleAssignmentVectorCompressed = EnumVectorCompressed<RoleAssignment COMMA RoleAssignment::numOfRoleAssignments>;

  ,
  (BehaviorState)(game) behaviorState, /**< The current Behavior status. */
  (SoccerState)(waiting) soccerState, /**< What is the robot doing in general? */
  (RoleAssignment)(noRole) role, /**< A dynamically chosen role. */
  (RoleAssignment)(noRole) lastRole, /**< For role sync. */
  (RobotRoleAssignmentVector) roleSuggestions,
  (unsigned char)(1) playerNumberToBall,
  (float)(100000.f) ownTimeToBall, /**< Can be either distance or time to ball based on method used. */
  (Vector2f)(Vector2f::Zero()) ballPositionRelative, /**< The robot's relative ball position w/preview. */
  (Vector2f)(Vector2f::Zero()) ballPositionField, /**< The robot's ball position on field. */
  (Vector2f)(Vector2f::Zero()) ballPositionFieldPredicted, /**< The robot's predicted ball position on field. */
  (int)(1000000) timeSinceBallWasSeen, /**< Time since the robot last saw the ball himself. */
  (Vector2f)(Vector2f(1000,0))  kickTarget /**< The kick target of the robot. */
);


STREAMABLE(BehaviorDataCompressed,
  // Increase version number whenever something changes!
  static constexpr unsigned char version = 1;

  BehaviorDataCompressed() = default;
  explicit BehaviorDataCompressed(const BehaviorData& behaviorData);
  ,
  ((BehaviorData) SoccerState)(BehaviorData::SoccerState::waiting) soccerState, /**< What is the robot doing in general? */ // TC: adjust/check falldown
  ((BehaviorData) RoleAssignment)(BehaviorData::RoleAssignment::noRole) role, /**< A dynamically chosen role. */
  ((BehaviorData) RobotRoleAssignmentVectorCompressed) roleSuggestions,
  (unsigned char)(1) playerNumberToBall,
  (Vector2fCompressed)(Vector2f::Zero()) ballPositionField, /**< The robot's ball position on field. */
  (Vector2fCompressed)(1000,0) kickTarget /**< The kick target of the robot. */ // TC: Is this needed? Only for Receiver?
);

inline BehaviorDataCompressed::BehaviorDataCompressed(const BehaviorData& behaviorData)
    : soccerState(behaviorData.soccerState), role(behaviorData.role), roleSuggestions(behaviorData.roleSuggestions), playerNumberToBall(behaviorData.playerNumberToBall),
      ballPositionField(behaviorData.ballPositionField /* TC: We can switch to predicted here */), kickTarget(behaviorData.kickTarget){};
