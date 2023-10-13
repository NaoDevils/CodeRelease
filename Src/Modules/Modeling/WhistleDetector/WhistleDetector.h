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
#include "Tools/RingBufferWithSum.h"

#include "Modules/Perception/TFlite.h"

MODULE(WhistleDetector,
  REQUIRES(FrameInfo),
  REQUIRES(AudioData),
  REQUIRES(MotionInfo),
  PROVIDES(WhistleDortmund),
  LOADS_PARAMETERS(,
    (std::string) whistleNetPath,
    (unsigned int) micBrokenThreshold,
    (float) limit,
    (float) threshold,
    (bool) useAdaptiveThreshold,
    (int) adaptiveWindowSize,
    (float) nnWeight,
    (float) pmWeight,
    (float) limitWeight,
    (bool) useWeightedPMConfidence,
    (int) release,
    (int) attack,
    (int) attackTimeout, // in ms
    (bool) useHannWindowing,
    (bool) useNuttallWindowing,
    (int) minFreq, 
    (int) maxFreq,
    (bool) freqCalibration,
    (bool) eval
  )
);

class WhistleDetector : public WhistleDetectorBase
{
  DECLARE_DEBUG_IMAGE(FFT);
  DECLARE_DEBUG_IMAGE(CHROMA);

  std::string oldWhistleNetPath;

  std::unique_ptr<tflite::Interpreter> interpreter;

  std::unique_ptr<tflite::FlatBufferModel> model;
  tflite::ops::builtin::BuiltinOpResolver resolver;
  int input_tensor, output_tensor;

  int chromaPos = 0;

  int windowSize = 0;
  int ampSize = 0;

  float pmConfidence = 0.f;
  float nnConfidence = 0.f;
  float relLimitCount = 0.f;

  RingBufferWithSum<float, 20> thresholdBuffer;
  RingBufferWithSum<float, 10> confidenceBuffer;
  int prevAdaptiveWindowSize = 20;

  unsigned int currentMic = 0;
  unsigned int micBrokenCount = 0;
  bool alert = false;

  unsigned int ringPos, samplesLeft;
  unsigned int releaseCount, attackCount;
  int detectedWhistleFrequency = 0;
  RingBufferWithSum<int, 10> whistleFreqBuffer;
  int currentMinFreq = 0;
  int currentMaxFreq = 0;
  int oldMinFreq = 0;
  int oldMaxFreq = 0;
  std::vector<float> buffer;
  std::vector<kiss_fft_cpx> in, out;
  std::vector<float> amplitudes;
  std::vector<float> gradients;
  RingBufferWithSum<float, 200> maxAmpHist;

  unsigned lastAttackTime = 0;
  bool detectionProcessed = false;
  bool evaluationStarted = false;

public:
  WhistleDetector();
  void update(WhistleDortmund& whistle);
  void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);

private:
  void setup();
};
