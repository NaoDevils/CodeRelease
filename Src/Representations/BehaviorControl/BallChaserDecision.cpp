#include "BallChaserDecision.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"

void BallChaserDecision::draw() const
{
  int pNumber = Blackboard::get<RobotInfo>().number;
  DEBUG_DRAWING3D("representation:BallChaserDecision", "robot")
  {
    if (playerNumberToBall == pNumber)
      SPHERE3D("representation:BallChaserDecision", 0.f, 0.f, 750.f, 50.f, ColorRGBA::red);
  }

  DECLARE_DEBUG_DRAWING("representation:BallChaserDecision", "drawingOnField");
  const FieldDimensions& theFieldDimensions = Blackboard::get<FieldDimensions>();
  DRAWTEXT("representation:BallChaserDecision", theFieldDimensions.xPosOwnGroundline - 500.f, theFieldDimensions.yPosLeftSideline + 300.f, 250, playerNumberToBall == pNumber ? ColorRGBA::green : ColorRGBA::red, "BC");
}
