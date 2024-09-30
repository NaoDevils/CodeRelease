#include "RoleSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/RobotInfo.h"

void RoleSymbols::draw() const
{
  // role symbol drawings in red
  DEBUG_DRAWING("representation:RoleSymbols", "drawingOnField")
  {
    DRAWTEXT("representation:RoleSymbols", -3500, 3400, 100, ColorRGBA::red, "RoleSymbols:role = " << role);
  }

  DEBUG_DRAWING3D("representation:RoleSymbols", "robot")
  {
    // default color
    ColorRGBA digitColor = ColorRGBA::black;
    switch (role)
    {
    case BehaviorData::defenderLeft:
      digitColor = ColorRGBA::green;
      break;
    case BehaviorData::defenderRight:
      digitColor = ColorRGBA::green;
      break;
    case BehaviorData::defenderSingle:
      digitColor = ColorRGBA::green;
      break;
    case BehaviorData::center:
      digitColor = ColorRGBA::white;
      break;
    case BehaviorData::backupBallchaser:
      digitColor = ColorRGBA::magenta;
      break;
    case BehaviorData::receiver:
      digitColor = ColorRGBA::magenta;
      break;
    case BehaviorData::replacementKeeper:
      digitColor = ColorRGBA::cyan;
      break;
    case BehaviorData::leftWing:
      digitColor = ColorRGBA::orange;
      break;
    case BehaviorData::rightWing:
      digitColor = ColorRGBA::orange;
      break;
    case BehaviorData::frontWing:
      digitColor = ColorRGBA::red;
      break;
    case BehaviorData::backWing:
      digitColor = ColorRGBA::yellow;
      break;
    case BehaviorData::keeper:
      digitColor = ColorRGBA::blue;
      break;
    default:
      break;
    }
    int pNumber = Blackboard::get<RobotInfo>().number;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    DRAWDIGIT3D("representation:RoleSymbols", pNumber, Vector3f(centerDigit, 0.f, 600), 100, 8, digitColor);
    Pose3f origin(0, 0, 370);
    origin.rotateY(90_deg);
  }
}
