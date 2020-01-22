/**
 * @file GroundContactState.h
 * Declaration of struct GroundContactState.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct GroundContactState
 * Describes whether we got contact with ground or not.
 */
STREAMABLE(GroundContactState,
{,
  (bool)(true) contact, /**< a foot of the robot touches the ground */
  (bool)(true) leftFootHasGroundContact, /** true if the left foot has contact to ground */
  (bool)(true) rightFootHasGroundContact, /** true if the right foot has contact to ground */
  (float)(0.f) stepFrequencyLeft, /** measured foot ground contact frequency for the left foot */
  (float)(0.f) stepFrequencyRight, /** measured foot ground contact frequency for the right foot  */
});
