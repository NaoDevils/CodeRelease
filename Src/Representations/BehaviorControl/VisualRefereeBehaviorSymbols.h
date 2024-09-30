#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(VisualRefereeBehaviorSymbols,
  ENUM(State,
    idle,
    localize,
    look,
    capture
  );

  VisualRefereeBehaviorSymbols() { refereePositionInImage.fill(Vector2i::Zero()); }
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:VisualRefereeBehaviorSymbols:refereePosition", "drawingOnImage");
    POLYGON("representation:VisualRefereeBehaviorSymbols:refereePosition", static_cast<int>(refereePositionInImage.size()), refereePositionInImage, 1, Drawings::solidBrush, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
  }
  ,
  (State)(State::idle) state,
  (unsigned)(0) timestampStarted,
  (Vector2a)(Vector2a::Zero()) targetHeadAngle,
  (std::array<Vector2i,4>) refereePositionInImage
);
