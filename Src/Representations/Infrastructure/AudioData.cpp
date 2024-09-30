/**
 * @file AudioData.cpp
 * The file defines a struct that stores audio data of up to four channels.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "AudioData.h"
#include "Tools/Debugging/DebugDrawings.h"

void AudioData::draw() const
{
  DECLARE_PLOT("representation:AudioData:rearLeft");
  DECLARE_PLOT("representation:AudioData:rearRight");
  DECLARE_PLOT("representation:AudioData:frontLeft");
  DECLARE_PLOT("representation:AudioData:frontRight");
  for (size_t i = 0; i < samples.size(); ++i)
  {
    switch (i % channels)
    {
    case 0:
      PLOT("representation:AudioData:rearLeft", samples[i]);
      break;
    case 1:
      PLOT("representation:AudioData:rearRight", samples[i]);
      break;
    case 2:
      PLOT("representation:AudioData:frontLeft", samples[i]);
      break;
    case 3:
      PLOT("representation:AudioData:frontRight", samples[i]);
      break;
    }
  }
}
