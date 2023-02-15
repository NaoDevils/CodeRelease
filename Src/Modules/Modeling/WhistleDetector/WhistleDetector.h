#pragma once

#include <kissfft/kiss_fft.h>

#include "Tools/Module/Module.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <cstdio>
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Platform/SystemCall.h"
#include <string>
#include "Tools/Math/Constants.h"

#include "Modules/Perception/TFlite.h"

MODULE(WhistleDetector,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  REQUIRES(MotionInfo),
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(,
    (int) windowSize,
    (float) threshold,
    (int) release,
    (int) attack,
    (int) attackTimeout, // in ms
    (bool) useHannWindowing,
    (bool) useNuttallWindowing,
    (int) minFreq, // for debugging
    (int) maxFreq // for debugging
  )
);

class WhistleDetector : public WhistleDetectorBase
{
  DECLARE_DEBUG_IMAGE(FFT);
  DECLARE_DEBUG_IMAGE(CHROMA);

  std::unique_ptr<tflite::Interpreter> interpreter;

  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  int input_tensor, output_tensor;

  int chromaPos = 0;

  const int AMP_SIZE = (1 + ((windowSize - 1) / 2) + 1);

  unsigned int ringPos, samplesLeft;
  unsigned int releaseCount, attackCount;
  std::vector<float> buffer;
  std::vector<kiss_fft_cpx> in, out;
  std::vector<float> amplitudes;

  unsigned lastAttackTime = 0;

public:
  WhistleDetector();
  void update(WhistleDortmund& whistle);
  void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);
};
