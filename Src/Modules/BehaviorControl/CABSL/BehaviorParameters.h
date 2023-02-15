#pragma once
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/MotionControl/KickRequest.h"

STREAMABLE(MixedTeamConfig,
  STREAMABLE(PlayerPositions,,
    (Vector2f)(Vector2f::Zero()) goalie,
    (Vector2f)(Vector2f::Zero()) fieldPlayerOffense,
    (Vector2f)(Vector2f::Zero()) fieldPlayerSupporter
  ),
  (PlayerPositions) readyPoses,
  (PlayerPositions) playingPoses
);

STREAMABLE(MixedTeamConfigs,
  ENUM(MixedTeamConfigName,
    defensive,
    offensiveWall,
    offensive
  ),
  (MixedTeamConfigName) defensiveConfig,
  (MixedTeamConfigName) offensiveConfig,
  (std::vector<MixedTeamConfig>) configs
);

STREAMABLE(BehaviorParameters,
  STREAMABLE(PositionConstraints,,
    (float)(-1000.f) minX, //!< Minimum x coordinate of the allowed positions.
    (float)(1000.f) maxX, //!< Maximum x coordinate of the allowed positions.
    (float)(-1000.f) minY, //!< Minimum y coordinate of the allowed positions.
    (float)(1000.f) maxY //!< Maximum y coordinate of the allowed positions.
  );

  ENUM(HeadControlVersion,
    HC2014,
    HC2016
  ),
   
  (HeadControlVersion)(HC2014) headControlVersion, /**< Set head control version. */
  // to activate testing:
  (bool)(false) behaviorTestmode, /**< If true, the robots will use save testing methods instead of competition behavior. */
  // general strategic options
  (int)(0) defensiveBehaviorScoreDiff, /**< If true, the robots will position themselves according to defensive constraints. */
  // general tactic options
  (bool)(false) useBlockForFieldPlayers, /**< If true, the fast block motion will be use defensively by field players. */
  // fine tuning
  // advanced roles? basic roles have fixed position based on ball position
  (float) goalieSaveFriction,
  (float)(400.f) relativeXBallPosition,
  (float)(80.f)  relativeYminBallPosition,
  (float)(350.f) relativeYmaxBallPosition,
  (bool)(true) useDive, // use dive motion?
  (bool)(true) goalieUseIntercept,
  (bool)(false) goalieInterceptOnly,
  (bool)(true) useBallInterception, // if true and the ball moves towards robot's y axis he will attempt an interception
  (float)(0.f) ballInterceptionAccY, // ball interception acceleration can be upped for y since speed is nearly only in y direction
  (float)(0.f) minXPositionForIntercept, // predicted relative x position of ball to trigger intercept
  (bool)(false) useMarking, // pressure opponents
  (bool)(false) useBlindSideKick, // kick to side when ball is next to robot
  (bool)(false) useWEDribbling, // dribbling will be handled by walking engine
  ((KickRequest) KickMotionID)(kickMiddle) longKick, // the used kick in hurry
  ((KickRequest) KickMotionID)(kickMiddle) longKickStable, // the used kick when in no hurry
  (int)(4) minStepsBetweenWalkKicks, // this many steps have to be made before the next walk kick can be taken

  // initial/kickoff options
  (bool)(false) useWalkKickForKickoff, // use walk kick for kickoff instead of long kick
  (bool)(false) neverDribbleForKickoff, // can be used to test the kickoff kicks, even if there are no other robots on field
  
  (bool)(true) kickOffToTheLeftSide, /** decides the initial kickoff direction */
  (bool)(true) useDynamicKickoffSideSwitching, /** If true, kickoff direction will be choosen dynamically by positioninig provider. */
  (int)(5000) initialWalkInTime,
  (float)(100.f) initialWalkInSpeed,
  (float)(0.8f) initialWalkInMinValidity,
  (float)(300.f) kickOffLineDistance, /**< Kickoff robot distance to center circle (opp kickoff) or middle line. */
  
  // striker kickoff position centered as forward as possible, no parameter needed
  (Vector2f)(Vector2f::Zero()) passReceiverKickOffPosition, /**< Kickoff position for pass receiving striker. */
  (Vector2f)(Vector2f::Zero()) offSupporterKickOffDefensePosition, /**< Kickoff position for supporterOff at opp kickoff. */
  // positioning constraints for all roles
  (PositionConstraints) ballchaserPositionConstraints, /**< The constraints for the ballchaser position. */
  (PositionConstraints) centerPositionConstraints, /**< Constraints for center position. */
  (PositionConstraints) defenderLeftPositionConstraints, /**< The constraints for the defenderLeft position. */
  (PositionConstraints) defenderRightPositionConstraints, /**< The constraints for the defenderRight position. */
  (PositionConstraints) defenderSinglePositionConstraints, /**< The constraints for the defenderSingle position. */
  (float)(500.f) positionConflictDistance, /**< If robots are within this distance of desired position, wait near. */

  // parameters for set plays
  (float)(900.f) freeKickCircleRadius, /**< The radius of the area around the ball that cannot be entered when defending a free kick. */
  ((KickRequest) KickMotionID)(kickMiddle) kickForFreeKicks /**< The type of kick to use if we have a free kick and enough time/space. */
);
