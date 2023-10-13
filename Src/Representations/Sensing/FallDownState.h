/**
 * @file Representations/Sensing/FallDownState.h
 *
 * Declaration of struct FallDownState
 *
 * @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
 * @struct FallDownState
 *
 * A struct that represents the current state of the robot's body
 */
STREAMABLE(FallDownState,
  ENUM(State,
    undefined,
    upright,
    onGround,
    onGroundLyingStill,
    staggering,
    falling,
    tryStandingUp,
    standingUp,
    heldOnOneShoulder,
    flying,
    diving
  );

  ENUM(Direction,
    none,
    front,
    left,
    back,
    right
  );

  /** Debug drawing. */
  void draw() const,

  (State)(undefined) state, /**< Current state of the robot's body. */
  (Direction)(none) direction, /**< The robot is falling / fell into this direction. */
  (bool)(true) standUpOnlyWhenLyingStill 
);
