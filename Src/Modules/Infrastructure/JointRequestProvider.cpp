#include "JointRequestProvider.h"

// Helpful for debugging
#include "Tools/Debugging/DebugDrawings.h"

void JointRequestProvider::update(JointRequest& jointRequest)
{
  jointRequest = theRawJointRequest;

  for (size_t i = 0; i < jointRequest.angles.size(); ++i)
  {
    if (jointRequest.angles[i] != SensorData::off && i >= Joints::firstLeftLegJoint)
    {
      jointRequest.angles[i] += theWalkCalibration.legJointCalibration[i - Joints::firstLeftLegJoint];
    }
  }
}

MAKE_MODULE(JointRequestProvider, motionInfrastructure);
