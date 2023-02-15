#include "RoleSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Infrastructure/RobotInfo.h"

void RoleSymbols::draw() const
{
  // role symbol drawings in red
  DEBUG_DRAWING("representation:RoleSymbols", "drawingOnField")
  {
    DRAWTEXT("representation:RoleSymbols", -3500, 3400, 100, ColorRGBA::red, "RoleSymbols:role = " << role);
  }
}