#include "RobotsPerceptProvider.h"

void RobotsPerceptProvider::update(RobotsPercept& theRobotsPercept)
{
  ASSERT(theRobotsPerceptOrientation.robots.size() == theRobotsPerceptTeam.robots.size());
  size_t const size = theRobotsPerceptTeam.robots.size();
  theRobotsPercept.robots = std::vector<RobotEstimate>(size);
  for (size_t i = 0; i < size; i++)
  {
    theRobotsPercept.robots.at(i) = mergeRobotEstimates(theRobotsPerceptOrientation.robots.at(i), theRobotsPerceptTeam.robots.at(i));
  }
}

RobotEstimate RobotsPerceptProvider::mergeRobotEstimates(RobotEstimate const& orientation, RobotEstimate const& team)
{
  // TODO
  return team;
}

MAKE_MODULE(RobotsPerceptProvider, perception)
