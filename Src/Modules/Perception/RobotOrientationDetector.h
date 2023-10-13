#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

MODULE(RobotOrientationDetector,
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(RobotsPerceptClassified),

  PROVIDES(RobotsPerceptOrientation)
);


class RobotOrientationDetector : public RobotOrientationDetectorBase
{
public:
  void update(RobotsPerceptOrientation& theRobotsPerceptOrientation);
};
