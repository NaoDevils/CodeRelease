#include "CoMRCSProvider.h"

void CoMRCSProvider::update(ActualCoMRCS &actualCoMRCS)
{
  if (!walkFixedCoM || thePatternGenRequest.newState != PatternGenRequest::walking)
    (Point &)actualCoMRCS = Point(theRobotModel.centerOfMass.x()/1000, theRobotModel.centerOfMass.y()/1000, (theRobotModel.centerOfMass.z())/1000 , 0);
  //else
   // actualCoMRCS.y = theRobotModel.centerOfMass.y() / 1000;
  PLOT("module:CoMRCSProvider:x", actualCoMRCS.x);
  PLOT("module:CoMRCSProvider:y", actualCoMRCS.y);
  PLOT("module:CoMRCSProvider:z", actualCoMRCS.z);
}

MAKE_MODULE(CoMRCSProvider, dortmundWalkingEngine);