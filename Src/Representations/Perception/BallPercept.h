/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BallPercept,
{
  /**
   * @enum Status
   * Only seen and notSeen are used by NDevils.
   */
  ENUM(Status,
  {,
    notSeen,
    seen,
    checkNoNoise, /**< unused */
    checkBallSpot, /**< unused */
    searchBallPoints, /**< unused */
    checkBallPoints, /**< unused */
    calculateBallInImage, /**< unused */
    checkBallInImage, /**< unused */
    calculateBallOnField, /**< unused */
    checkBallOnField, /**< unused */
    checkJersey, /**< unused */
  });

  /** Draws the ball*/
  void draw() const,

  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
  (Vector2f) relativePositionOnField, /**< Ball position relative to the robot. */
  (float)(35) radiusOnField,          /**< The radius of the ball on the field in mm */
  (float)(0) validity,                /**< The validity of the ball percept. */
  (bool) fromUpper,                   /**< True, if ball was seen in upper image. Use with status. */
});
