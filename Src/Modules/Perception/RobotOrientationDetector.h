#pragma once

#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

MODULE(RobotOrientationDetector,
  REQUIRES(RobotsPerceptClassified),

  PROVIDES(RobotsPerceptOrientation)
);


class RobotOrientationDetector : public RobotOrientationDetectorBase
{
public:
  void update(RobotsPerceptOrientation& theRobotsPerceptOrientation);
};
