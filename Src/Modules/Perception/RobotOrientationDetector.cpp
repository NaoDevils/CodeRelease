#include "RobotOrientationDetector.h"

void RobotOrientationDetector::update(RobotsPerceptOrientation& theRobotsPerceptOrientation)
{
  theRobotsPerceptOrientation.robots.clear();
  for (const auto& e : theRobotsPerceptClassified.robots)
  {
    // TODO
    auto estimate(e);
    estimate.locationOnField.rotation = NAN;
    theRobotsPerceptOrientation.robots.push_back(estimate);
  }
}

MAKE_MODULE(RobotOrientationDetector, perception)
