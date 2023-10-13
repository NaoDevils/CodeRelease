#include "BallChaserDecision.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallChaserDecision::draw() const
{
  DEBUG_DRAWING3D("representation:BallChaserDecision", "robot")
  {
    int pNumber = Blackboard::get<RobotInfo>().number;
    if (playerNumberToBall == pNumber)
      SPHERE3D("representation:BallChaserDecision", 0.f, 0.f, 750.f, 50.f, ColorRGBA::red);
  }
}
