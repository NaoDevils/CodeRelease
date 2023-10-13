#pragma once

#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

MODULE(RobotsPerceptProvider,
  REQUIRES(RobotsPerceptTeam),
  REQUIRES(RobotsPerceptOrientation),
  PROVIDES(RobotsPercept)
);

class RobotsPerceptProvider : public RobotsPerceptProviderBase
{
private:
  RobotEstimate mergeRobotEstimates(RobotEstimate const& orientation, RobotEstimate const& team);

public:
  void update(RobotsPercept& theRobotsPercept);
};
