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

  ENUM(DetectionType,
  {,
    scanlines,
    yoloOnly,
    yoloHypothesis,
    yoloFallback,
  });

  /** Draws the ball */
  void draw() const,

  (Status)(notSeen) status,                                 /**< Indicates, whether the ball was seen in the current image.
                                                            If \c notSeen all other members of this representation are invalid. */
  (Vector2f)(Vector2f::Zero()) positionInImage,             /**< The position of the ball in the current image */
  (float)(1) radiusInImage,                                 /**< The radius of the ball in the current image */
  (Vector2f)(Vector2f(1000.f,0.f)) relativePositionOnField, /**< Ball position relative to the robot. */
  (float)(50) radiusOnField,                                /**< The radius of the ball on the field in mm */
  (float)(0) validity,                                      /**< The validity of the ball percept in range [0,1]. */
  (bool)(false) fromUpper,                                  /**< True, if ball was seen in upper image. Use with status. */
  (DetectionType)(scanlines) detectionType,                 /**< Who is responsible for the detection. */
});

/**
 * Representation of multiple hypotheses for seen balls.
 */
STREAMABLE(MultipleBallPercept,
{
  MultipleBallPercept() = default;
  
  /** Draws all ball percepts */
  void draw() const
  {
    for (const BallPercept& ball : balls)
      ball.draw();
  },
  
  (std::vector<BallPercept>) balls,   /**< List of multiple \c BallPerceps. */
});
