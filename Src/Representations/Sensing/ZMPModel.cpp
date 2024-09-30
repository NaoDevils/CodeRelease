/**
* @file ZMPModel.cpp
*
* Implementation of class ZMPModel
*
* @author <A href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
* @author <A href="arne.arne@tu-dortmund.de">Arne Moos</A>
*/

#include "ZMPModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/RobotParts/FootShape.h"

void ZMPModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ZMPModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:ZMPModel:zmp", "drawingOnField");

  Angle fieldDrawingRotation = 90_deg;
  float fieldDrawingScale = 20.f;
  float zDiffForFootSwitch = 0.003f;

  if (std::abs(feetPose.left.translation.z() - feetPose.right.translation.z()) > zDiffForFootSwitch)
  {
    if (feetPose.left.translation.z() < feetPose.right.translation.z())
    {
      drawFoot(true, Pose2f(-fieldDrawingRotation, Vector2f(feetPose.left.translation.head<2>() * 1000.f).rotate(fieldDrawingRotation)), fieldDrawingScale);
    }
    else
    {
      drawFoot(false, Pose2f(-fieldDrawingRotation, Vector2f(feetPose.right.translation.head<2>() * 1000.f).rotate(fieldDrawingRotation)), fieldDrawingScale);
    }
  }
  else
  {
    drawFoot(true, Pose2f(-fieldDrawingRotation, Vector2f(feetPose.left.translation.head<2>() * 1000.f).rotate(fieldDrawingRotation)), fieldDrawingScale);
    drawFoot(false, Pose2f(-fieldDrawingRotation, Vector2f(feetPose.right.translation.head<2>() * 1000.f).rotate(fieldDrawingRotation)), fieldDrawingScale);
  }

  Vector2f ZMP_RCS_rotated(ZMP_RCS.x(), ZMP_RCS.y());
  ZMP_RCS_rotated.rotate(fieldDrawingRotation);
  LARGE_DOT("representation:ZMPModel:zmp", ZMP_RCS_rotated.x() * 1000.f * fieldDrawingScale, ZMP_RCS_rotated.y() * 1000.f * fieldDrawingScale, ColorRGBA::red, ColorRGBA::red);
}

void ZMPModel::drawFoot(bool left, const Pose2f& baseInImage, float scale) const
{
  COMPLEX_DRAWING("representation:ZMPModel")
  {
    ColorRGBA drawColor = left ? ColorRGBA::blue : ColorRGBA::green;
    for (unsigned int i = 0; i < FootShape::polygon.size(); i++)
    {
      Vector2f p1 = FootShape::polygon[i];
      Vector2f p2 = FootShape::polygon[(i + 1) % FootShape::polygon.size()];
      if (left)
      {
        p1.y() = -p1.y();
        p2.y() = -p2.y();
      }
      p1.rotate(baseInImage.rotation);
      p2.rotate(baseInImage.rotation);
      LINE("representation:ZMPModel",
          (p1.x() + baseInImage.translation.x()) * scale,
          (-p1.y() + baseInImage.translation.y()) * scale,
          (p2.x() + baseInImage.translation.x()) * scale,
          (-p2.y() + baseInImage.translation.y()) * scale,
          static_cast<int>(2 * scale),
          Drawings::solidPen,
          drawColor);
    }

    Vector2f FL(70.25f, 29.9f);
    Vector2f FR(70.25f, -23.1f);
    Vector2f RL(-30.25f, 29.9f);
    Vector2f RR(-29.65f, -19.1f);
    if (left)
    {
      FL.y() = -FL.y();
      FR.y() = -FR.y();
      RL.y() = -RL.y();
      RR.y() = -RR.y();
    }

    FL.rotate(baseInImage.rotation);
    FR.rotate(baseInImage.rotation);
    RL.rotate(baseInImage.rotation);
    RR.rotate(baseInImage.rotation);

    CROSS("representation:ZMPModel", (baseInImage.translation.x()) * scale, (baseInImage.translation.y()) * scale, static_cast<int>(2 * scale), static_cast<int>(scale), Drawings::solidPen, drawColor);
    LARGE_DOT("representation:ZMPModel", (FL.x() + baseInImage.translation.x()) * scale, (-FL.y() + baseInImage.translation.y()) * scale, drawColor, drawColor);
    LARGE_DOT("representation:ZMPModel", (FR.x() + baseInImage.translation.x()) * scale, (-FR.y() + baseInImage.translation.y()) * scale, drawColor, drawColor);
    LARGE_DOT("representation:ZMPModel", (RL.x() + baseInImage.translation.x()) * scale, (-RL.y() + baseInImage.translation.y()) * scale, drawColor, drawColor);
    LARGE_DOT("representation:ZMPModel", (RR.x() + baseInImage.translation.x()) * scale, (-RR.y() + baseInImage.translation.y()) * scale, drawColor, drawColor);
  }
}
