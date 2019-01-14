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
  // general strategic options
  (bool) defensiveBehavior, /**< If true, the robots will position themselves according to defensive constraints. */
  (bool) useBallSearch2017, /**< If true, the robots use the fieldcoverage for the ballsearch>*/
  // general tactic options
  (bool) useBlockForFieldPlayers, /**< If true, the fast block motion will be use defensively by field players. */
  // fine tuning
  // advanced roles? basic roles have fixed position based on ball position
  (bool) useAdvancedDefender,
  (bool) useAdvancedSupporterDef,
  (bool) useAdvancedSupporterOff,
  (bool) useAdvancedStriker,
  (bool) useNewGoalie,
  (bool)(false) useMarking, // pressure opponents
  (bool)(false) useBlindSideKick, // kick to side when ball is next to robot
  (bool)(false) useWEDribbling, // dribbling will be handled by walking engine
  // initial/kickoff options
  (int) initialWalkInTime,
  (float) initialWalkInSpeed,
  (float) strikerKickOffLineDistance, /**< Striker distance to center circle (opp kickoff) or middle line. */

  // striker kickoff position centered as forward as possible, no parameter needed
  (Vector2f) offensiveSupporterKickOffPositionOff, /**< Kickoff position for off. supporter (offensive behavior). */
  (Vector2f) offensiveSupporterKickOffPositionDef, /**< Kickoff position for off. supporter (defensive behavior). */
  (Vector2f) defensiveSupporterKickOffPositionOff, /**< Kickoff position for def. supporter (offensive behavior). */
  (Vector2f) defensiveSupporterKickOffPositionDef, /**< Kickoff position for def. supporter (defensive behavior). */
  // positioning constraints for all roles
  (PositionConstraints) strikerPositionConstraints, /**< The constraints for the striker position if not at the ball. */
  (PositionConstraints) offensiveSupporterPositionConstraintsDef, /**< Constraints for offensive supporter position in defensive behavior. */
  (PositionConstraints) offensiveSupporterPositionConstraintsOff, /**< Constraints for offensive supporter position in offensive behavior. */
  (PositionConstraints) defensiveSupporterPositionConstraintsDef, /**< Constraints for defensive supporter position in defensive behavior. */
  (PositionConstraints) defensiveSupporterPositionConstraintsOff, /**< Constraints for defensive supporter position in offensive behavior. */
  (PositionConstraints) defenderPositionConstraints, /**< The constraints for the defender position if not at the ball. */
  // parameters for kickoff defense
  (float) enterDistanceForActiveDefense,
  (float) exitDistanceForActiveDefense,
  (float) formationDistanceToBall,
  (float) robotDistanceInTripleFormation,
  (float) robotDistanceInDoubleFormation,
});
