/**
 * @file Representations/Modeling/TeamBallModel.cpp
 *
 * Implementation of drawings for the team ball model
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "TeamBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void TeamBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamBallModel", "drawingOnField");

  DEBUG_DRAWING3D("representation:TeamBallModel", "field")
  {
    const Vector3f ballPos3d = Vector3f(position.x(), position.y(), 0.0f);
    const Vector3f ballSpeed3d = Vector3f(velocity.x(), velocity.y(), 0.0f);
    SPHERE3D("representation:TeamBallModel", ballPos3d.x(), ballPos3d.y(), 35.f, 35.f, ColorRGBA(128, 64, 0));
    if (ballSpeed3d.squaredNorm() > 0.9f)
    {
      CYLINDERARROW3D("representation:TeamBallModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(128, 64, 0));
    }
  }

  COMPLEX_DRAWING("representation:TeamBallModel")
  {
    CIRCLE("representation:TeamBallModel", position.x(), position.y(), 30, 20, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
    ARROW("representation:TeamBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::blue);
    //DRAWTEXT("representation:TeamBallModel", position.x() + 50, position.y() + 50, 50, ColorRGBA::blue, static_cast<int>(validity * 100.f + 0.5f) << "%");
    DRAWTEXT("representation:TeamBallModel", 0, 3200, 50, ColorRGBA::blue, "TeamBallModel");
    DRAWTEXT("representation:TeamBallModel", 0, 3100, 50, ColorRGBA::blue, (isValid ? (isLocalBallModel ? "valid local" : "valid remote") : "invalid"));
  }
}
