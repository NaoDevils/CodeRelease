#pragma once
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"

STREAMABLE(Behavior2015Parameters,
{
  STREAMABLE(PositionConstraints,
  {,
    (float) minX, //!< Minimum x coordinate of the allowed positions.
    (float) maxX, //!< Maximum x coordinate of the allowed positions.
    (float) minY, //!< Minimum y coordinate of the allowed positions.
    (float) maxY, //!< Maximum y coordinate of the allowed positions.
  });

  ENUM(HeadControlVersion,
  {,
    HC2014,
    HC2016,
  }),

  (HeadControlVersion) headControlVersion, /**< Set head control version. */
  (bool)(true) useFastKicks, /**< If true, the fast Kicks will be used. */
  (bool)(false) useOutdoorKick, /**< Change Fastkicks for use on the outdoor field */
  (bool)(true) useSlowAndFastKicks, /* Use Distance dependend Kickdecision */
  (float)(5000.0) maxFastKickDistance, /**< Max distance to kicktarget to use fastKicks */
  (bool) useInstantKick, /**< If true, not only slow kick engine(s) will be used for kicking, depending on situations. */
  (bool) useInstantOnly, /**< If true, only fast instant kicks will be used for kicking. */
  (bool) useSideKick, /**< If true, side kicks are possible (in front of obstacles or if path is shorter). */
  (bool) useRotateSideKick, /**< If true, rUNSWift like side kick is used */
  (bool) usePassing, /**< If true, attempts at passes will be made, else only goal or evasion shots are made. */
  (bool) useSideApproach, /**< If true, the robot will approach the ball from the side if deemed useful. */
  (bool) previewArrival, /**< If true, the robot will use preview for decision if robot has arrived (faster kick trigger). */
  (bool) defensiveBehavior, /**< If true, the robots will position themselves according to defensive constraints. */
  (bool) interceptionEnabled, /**< If true, a rolling ball will be intercepted, if possible. */
  (bool) switchRoles, /**< If true, roles will be switched if one player approaches another roles position (by chasing ball). */
  (bool) avoidPositionConflicts, /**< If true, positions will be adjusted if in way of ball chaser. */
  // advanced roles? basic roles have fixed position based on ball position
  (bool) useAdvancedDefender,
  (bool) useAdvancedSupporterDef,
  (bool) useAdvancedSupporterOff,
  (bool) useAdvancedStriker,
  // fine tuning
  (float) roleChangeDistance, /**< Max Distance of robots to be considered for role switch. */
  (float) criticalOpeningAngle, /**< Critical opening angle of ball to opponent goal posts, used for kick type decision. */
  (float) safeOpeningAngle, /**< Safe opening angle of ball to opponent goal posts, used for kick type decision. */
  (float) criticalPositioningAngle, /**< Critical angle of ball to opponent goal posts, to make sure not to kick over sidelines. */
  (float) decisionFreezeBallDistance, /**< If robot is nearer to ball, the kick decision will be frozen to avoid flickering approach decisions. */
  (float) ballBehindRobotPenalty, /**< Distance penalty if ball is behind robot for ball chase decision. */
  (float) minDistToGCForDribbling, /**< If distance to opp goal center is smaller, maybe dribble. */
  (float) maxOpeningAngleForDribbling, /**< If opening angle of opp goal is smaller, maybe dribble. */
  (float) minWideOpeningAngleForDribbling, /** If opening angle is this wide, just run into goal with the ball. */
  (float) maxDistanceForDefiniteBallChase,
  (int) initialWalkInTime,
  (int) initialWalkInSpeed,
  (float) strikerKickOffLineDistance, /**< Striker distance to center circle (opp kickoff) or middle line. */
  (PositionConstraints) strikerPositionConstraints, /**< The constraints for the striker position if not at the ball. */
  (PositionConstraints) offensiveSupporterPositionConstraintsDef, /**< Constraints for offensive supporter position in defensive behavior. */
  (PositionConstraints) offensiveSupporterPositionConstraintsOff, /**< Constraints for offensive supporter position in offensive behavior. */
  (PositionConstraints) defensiveSupporterPositionConstraintsDef, /**< Constraints for defensive supporter position in defensive behavior. */
  (PositionConstraints) defensiveSupporterPositionConstraintsOff, /**< Constraints for defensive supporter position in offensive behavior. */
  (PositionConstraints) defenderPositionConstraints, /**< The constraints for the defender position if not at the ball. */
                                                   // striker kickoff position centered as forward as possible
  (Vector2f) offensiveSupporterKickOffPositionOff, /**< Kickoff position for off. supporter (offensive behavior). */
  (Vector2f) offensiveSupporterKickOffPositionDef, /**< Kickoff position for off. supporter (defensive behavior). */
  (Vector2f) defensiveSupporterKickOffPositionOff, /**< Kickoff position for def. supporter (offensive behavior). */
  (Vector2f) defensiveSupporterKickOffPositionDef, /**< Kickoff position for def. supporter (defensive behavior). */

});
