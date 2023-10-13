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
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImagePatch.h"
#include "Representations/Perception/BallSpot.h"

#define CNN_POSITION_SIZE 32
#define CNN_SCANLINES_SIZE 16

STREAMABLE_WITH_BASE(BallPatch, ImagePatch,
  BallPatch() = default;
  BallPatch(const CheckedBallSpot& ballSpot, const Image& image, const Vector2i& inputPosition, const Vector2i& inputSize, const Vector2i& outputSize, bool rgb = true);
  void fromBallSpot(const CheckedBallSpot& ballSpot);
  ,
  (Vector2f)(Vector2f::Zero()) centerInPatch,
  (float)(CNN_POSITION_SIZE / 2.f) radiusInPatch,
  (float)(1.f) resizeFactor,
  (float)(0.f) validity,
  (bool)(false) fromUpper,
  ((CheckedBallSpot) DetectionSource)(CheckedBallSpot::DetectionSource::scanlines) source,
  ((CheckedBallSpot) DetectionVerifier)(CheckedBallSpot::DetectionVerifier::ballPositionCNN) verifier
);

/**
 * Representation of the single ball p.
 */
STREAMABLE(BallPercept,
  /**
   * @enum Status
   * Only seen and notSeen are used by NDevils.
   */
  ENUM(Status,
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
    checkJersey /**< unused */
  );

  BallPercept() = default;
  BallPercept(const CheckedBallSpot& ballSpot, unsigned timestamp, const Vector2f& posOnField);

  /** Draws the ball */
  void draw() const,

  (Status)(notSeen) status,                                 /**< Indicates, whether the ball was seen in the current image.
                                                            If \c notSeen all other members of this representation are invalid. */
  (unsigned)(0) timestamp,
  (Vector2f)(Vector2f::Zero()) positionInImage,             /**< The position of the ball in the current image */
  (float)(1) radiusInImage,                                 /**< The radius of the ball in the current image */
  (Vector2f)(Vector2f(1000.f,0.f)) relativePositionOnField, /**< Ball position relative to the robot. */
  (float)(50) radiusOnField,                                /**< The radius of the ball on the field in mm */
  (float)(0) validity,                                      /**< The validity of the ball percept in range [0,1]. */
  (bool)(false) fromUpper,                                  /**< True, if ball was seen in upper image. Use with status. */
  ((CheckedBallSpot) DetectionSource)(CheckedBallSpot::scanlines) detectionSource, /**< Who is responsible for the detection. */
  ((CheckedBallSpot) DetectionVerifier)(CheckedBallSpot::scanlinesAndCNN) detectionVerifier, /**< Who is responsible for the verfication. */
  (BallPatch) ballPatch                                    /**< Saved ballPatch image data for the log. */
);

/**
 * Representation of multiple hypotheses for seen balls.
 */
STREAMABLE(MultipleBallPercept,
  MultipleBallPercept() { balls.reserve(10); }

  /** Draws the ball */
  void draw() const,

  (std::vector<BallPercept>) balls   /**< List of multiple \c BallPerceps. */
);

/**
 * Representation of multiple processed ball patches.
 */
STREAMABLE(ProcessedBallPatches,
  ProcessedBallPatches() { patches.reserve(10); }

  /** Draws the patches */
  void draw() const,

  (std::vector<BallPatch>) patches   /**< List of multiple \c BallPatches. */
);
