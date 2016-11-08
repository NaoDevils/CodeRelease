/*
* @file WhistleRecognizer.h
*
* Declaration of module that identifies the whistle
*
* Set sampleRate in AudioProvider to 16384Hz to use this module correctly
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <cstdio>
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Tools/Debugging/DebugImages.h"
MODULE(WhistleDetectorOld,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioDataDortmund),
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(
  {,
    (unsigned int) minFreq,
	(int) threshold,
	(int) dist,
	(int) minDur,
  }),
});

class WhistleDetectorOld : public WhistleDetectorOldBase
{
  //FILE *f;
  DECLARE_DEBUG_IMAGE(FFTTT);
  int minAmp;
  int durCount;
public:

  WhistleDetectorOld();
  void update(WhistleDortmund &whistle);
};