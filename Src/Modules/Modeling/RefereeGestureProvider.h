/**
 * @file RefereeGestureProvider.h
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/RefereeKeypoints.h"
#include "Representations/Modeling/RefereeGesture.h"

MODULE(RefereeGestureProvider,
  REQUIRES(RefereeKeypoints),
  PROVIDES(RefereeGesture),
  LOADS_PARAMETERS(,
    (bool)(true) useSkeletonCheck,
    (bool)(true) useKeypointThreshold,
    (float)(0.3f) keypointThreshold
  )
);

class RefereeGestureProvider : public RefereeGestureProviderBase
{
public:
  RefereeGestureProvider() { gestureRingBuffer.fill(RefereeGesture::NONE); }

private:
  static const int RINGBUFFERSIZE = 10;
  int current_rb = 0;
  std::array<RefereeGesture::Gesture, RINGBUFFERSIZE> gestureRingBuffer;
  void update(RefereeGesture& refereeGesture);
};
