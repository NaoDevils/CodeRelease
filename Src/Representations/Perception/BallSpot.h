/**
 * @file BallSpot.h
 * Declaration of a struct that represents a spot that might be an indication of a ball.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct BallSpot
 * A struct that represents a spot that might be an indication of a ball.
 */
STREAMABLE(BallSpot,
  BallSpot() = default;
  BallSpot(Vector2i pos, bool upper) : position(pos) COMMA upper(upper) {};
  BallSpot(int x, int y, bool upper) : position(x, y) COMMA upper(upper) {};

  bool operator<(const BallSpot& second) const
  { return (position.y() > second.position.y());
  }
  ,

  (Vector2i)(Vector2i::Zero()) position,
  (float)(0.f) radiusInImage,
  (float)(0.f) validity,
  (bool)(true) upper
);

STREAMABLE_WITH_BASE(ScanlinesBallSpot, BallSpot,
  using BallSpot::BallSpot;
  explicit ScanlinesBallSpot(const BallSpot& ballSpot) : BallSpot(ballSpot) COMMA y(250) COMMA cb(128) COMMA cr(128) {}
  ,
  (bool)(false) found,
  (bool)(false) centerFound,
  (int)(0) y,
  (int)(0) cb,
  (int)(0) cr
);

// A&A TODO: Remove this ugly inheritance from ScanlinesBallSpot.
// Needed to call calcBallSpot2016().
STREAMABLE_WITH_BASE(CheckedBallSpot, ScanlinesBallSpot,
  ENUM(DetectionSource,
    scanlines,
    yoloHypothesis,
    ballModel,
    segmentorHypothesis
  );

  ENUM(DetectionVerifier,
    scanlinesAndCNN,
    ballPositionCNN,
    yolo
  );

  CheckedBallSpot() = default;
  explicit CheckedBallSpot(const BallSpot& ballSpot, DetectionSource source, DetectionVerifier verifier)
    : ScanlinesBallSpot(ballSpot) COMMA source(source) COMMA verifier(verifier) {}
  explicit CheckedBallSpot(const ScanlinesBallSpot& ballSpot, DetectionSource source, DetectionVerifier verifier)
    : ScanlinesBallSpot(ballSpot) COMMA source(source) COMMA verifier(verifier) {}
  ,
  (DetectionSource)(DetectionSource::ballModel) source,
  (DetectionVerifier)(DetectionVerifier::ballPositionCNN) verifier
);
