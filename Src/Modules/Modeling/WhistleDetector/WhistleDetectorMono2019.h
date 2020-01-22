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

STREAMABLE(WhistleDetectorMono2019Params,
{,
	(unsigned) USE_CHANNEL,
	(int) WINDOW_SIZE,
	(int) MIN_FREQ_BACKGROUND_NOISE,
	(int) MAX_FREQ_BACKGROUND_NOISE,
	(int) NOISE_HISTORY_SIZE,
	(int) MIN_FREQ_WHISTLE,
	(int) MAX_FREQ_WHISTLE,
	(float) MIN_AMP,
	(float) OVERTONE_MULT_MIN_1,
	(float) OVERTONE_MULT_MAX_1,
	(float) OVERTONE_MIN_AMP_1,
	(float) OVERTONE_MULT_MIN_2,
	(float) OVERTONE_MULT_MAX_2,
	(float) OVERTONE_MIN_AMP_2,
	(int) RELEASE,
	(int) ATTACK,
	(int) PEAK_DIFF,
	(bool) USE_HANN_WINDOWING,
	(bool) USE_NUTTALL_WINDOWING,
	(int) ATTACK_TIMEOUT_MS,
	(float) DBG_IMG_Y_STRETCH, // streches the debug image y scale
});

MODULE(WhistleDetectorMono2019,
{,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(
  {,
          (WhistleDetectorMono2019Params) params,
  }),
});

class WhistleDetectorMono2019 : public WhistleDetectorMono2019Base
{

    DECLARE_DEBUG_IMAGE(FFT);
    DECLARE_DEBUG_IMAGE(CHROMA);
    int chromaPos = 0;

    const int AMP_SIZE = (1 + ((params.WINDOW_SIZE - 1) / 2) + 1);

  int peakPos = -1;
  unsigned int ringPos, samplesLeft;
  unsigned int releaseCount, attackCount;
  float currMinAmp;
  std::vector<float> buffer;
  std::vector<kiss_fft_cpx> in, out;
  std::vector<float> amplitudes;
  unsigned int currChannel, channelRequest;
  
  unsigned lastAttackTime = 0;

  std::vector<float> history = std::vector<float>(0);
  size_t historyPos = 0;

public:
  WhistleDetectorMono2019();
  void update(WhistleDortmund &whistle);
  void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);
private:
  float calc_noise_based_min_amp(std::vector<float> amplitudes, const int sampleRate, const size_t historySize);
};
