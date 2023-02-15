#include "MotionState.h"
#include "Tools/Debugging/DebugDrawings.h"

void MotionState::draw() const
{
  PLOT("representation:MotionState:fallDownSpeedReductionFactor:x", walkingStatus.fallDownSpeedReductionFactor.x());
  PLOT("representation:MotionState:fallDownSpeedReductionFactor:y", walkingStatus.fallDownSpeedReductionFactor.y());
}
