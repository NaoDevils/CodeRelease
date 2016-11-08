#pragma once

#include "Tools/Math/kiss_fft130/kiss_fft.h"

#include "Tools/Module/Module.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <cstdio>
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Platform/SystemCall.h"
#include <string>
#include "Tools/Math/Constants.h"

MODULE(WhistleDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioDataDortmund),
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(
  {,
    (int) WINDOW_SIZE,
    (int) MIN_FREQ,
    (int) MAX_FREQ,
    (float) MIN_AMP,
    (float) OVERTONE_MULT_MIN_1,
    (float) OVERTONE_MULT_MAX_1,
    (float) OVERTONE_MIN_AMP_1,
    (float) OVERTONE_MULT_MIN_2,
    (float) OVERTONE_MULT_MAX_2,
    (float) OVERTONE_MIN_AMP_2,
    (int) RELEASE,
    (int) ATTACK,
    (bool) USE_HANN_WINDOWING,
  }),
});

class WhistleDetector : public WhistleDetectorBase
{
    DECLARE_DEBUG_IMAGE(FFT);
    DECLARE_DEBUG_IMAGE(CHROMA);
    int chromaPos = 0;

    const int AMP_SIZE = (1 + ((WINDOW_SIZE - 1) / 2) + 1);

  unsigned int ringPos, samplesLeft;
  unsigned int releaseCount, attackCount;
  std::vector<float> buffer;
  std::vector<kiss_fft_cpx> in, out;
  std::vector<float> amplitudes;

public:
  WhistleDetector();
  void update(WhistleDortmund &whistle);
  void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);
};
