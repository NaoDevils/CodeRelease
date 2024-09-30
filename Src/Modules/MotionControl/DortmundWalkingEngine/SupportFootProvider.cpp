#include "SupportFootProvider.h"

#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(SupportFootProvider, motionControl);

void SupportFootProvider::update(SupportFoot& theSupportFoot)
{
  if (useFSRModel)
    supportFootBuffer.push_front(theFsrModelData.calcSupportFoot());
  else
    supportFootBuffer.push_front(theFsrSensorData.calcSupportFoot());

  float support = supportFootBuffer[0] + 1.f * (supportFootBuffer[0] - supportFootBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1]);
  float threshold = supportFootThresholdFactorFootYDistance * theWalkingEngineParams.footMovement.footYDistance;

  if (theSupportFoot.supportFootState == SupportFoot::bothFeetSupport)
  {
    if (support > threshold)
      theSupportFoot.supportFootState = SupportFoot::leftSupportOnly;
    else if (support < -threshold)
      theSupportFoot.supportFootState = SupportFoot::rightSupportOnly;
  }
  else if (theSupportFoot.supportFootState == SupportFoot::leftSupportOnly)
  {
    if (support <= threshold)
      theSupportFoot.supportFootState = SupportFoot::bothFeetSupport;
  }
  else if (theSupportFoot.supportFootState == SupportFoot::rightSupportOnly)
  {
    if (support >= -threshold)
      theSupportFoot.supportFootState = SupportFoot::bothFeetSupport;
  }

  theSupportFoot.supportFoot = support;
  theSupportFoot.threshold = threshold;
}
