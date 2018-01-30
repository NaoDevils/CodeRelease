#include "BallSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallSymbols::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallSymbols:FieldAbsolute", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:BallSymbols:FieldRelative", "drawingOnField");
  CIRCLE("representation:BallSymbols:FieldAbsolute", ballPositionField.x(), ballPositionField.y(),
    50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA(255,255,255,75));
  CIRCLE("representation:BallSymbols:FieldAbsolute", ballPositionFieldPredicted.x(), ballPositionFieldPredicted.y(),
    50, 0, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA(255,0,0,50));
  if (ballVelocityRelativeWOPreview.norm() > 50)
    ARROW("representation:BallSymbols:FieldRelative", ballPositionRelativeWOPreview.x(), ballPositionRelativeWOPreview.y(),
      ballPositionRelativeWOPreview.x() + ballVelocityRelativeWOPreview.x(), ballPositionRelativeWOPreview.y() + ballVelocityRelativeWOPreview.y(),
      5, Drawings::solidPen, ColorRGBA::black);
  if (std::abs(yPosWhenBallReachesOwnYAxis) < 5000)
  {
    CROSS("representation:BallSymbols:FieldRelative", 0, yPosWhenBallReachesOwnYAxis, 10, 3, Drawings::solidPen, ColorRGBA::green);
  }
  DRAWTEXT("representation:BallSymbols:FieldAbsolute", 1800, 3400, 100, ColorRGBA::white,
    "BallSymbols:timeSinceLastSeen = " << timeSinceLastSeen);
  DRAWTEXT("representation:BallSymbols:FieldAbsolute", 1800, 3250, 100, ColorRGBA::white,
    "BallSymbols:timeSinceLastSeenByTeam = " << timeSinceLastSeenByTeam);
  DRAWTEXT("representation:BallSymbols:FieldAbsolute", 1800, 3100, 100, ColorRGBA::white,
    "BallSymbols:useLocalBallModel = " << (useLocalBallModel ? "true" : "false"));
}
