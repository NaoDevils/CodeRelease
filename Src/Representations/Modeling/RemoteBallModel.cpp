/**
 * \file RemoteBallModel.cpp
 *
 * Implementation of debug drawings of the remote ball model.
 *
 * \author Heiner Walter, heiner.walter@tu-dortmund.de
 */

#include "RemoteBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void RemoteBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RemoteBallModel", "drawingOnField");

  DEBUG_DRAWING3D("representation:RemoteBallModel", "field")
  {
    const Vector3f ballPos3d = Vector3f(position.x(), position.y(), 0.0f);
    const Vector3f ballSpeed3d = Vector3f(velocity.x(), velocity.y(), 0.0f);
    SPHERE3D("representation:RemoteBallModel", ballPos3d.x(), ballPos3d.y(), 35.f, 35.f, ColorRGBA(0, 0, 0));
    if(ballSpeed3d.squaredNorm() > 0.9f)
    {
      CYLINDERARROW3D("representation:RemoteBallModel", ballPos3d, ballPos3d + ballSpeed3d, 5.f, 35.f, 35.f, ColorRGBA(0, 0, 0));
    }
  }

  COMPLEX_DRAWING("representation:RemoteBallModel")
  {
    CIRCLE("representation:RemoteBallModel", position.x(), position.y(), 30, 20, Drawings::solidPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);
    ARROW("representation:RemoteBallModel", position.x(), position.y(), position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::black);
    if (!teammates.empty())
    {
      DRAWTEXT("representation:RemoteBallModel",
        position.x() + 70, position.y() - 100,
        150, ColorRGBA::black, "p " << teammates);
    }
  }
}
