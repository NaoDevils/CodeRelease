/**
 * @file PortAudioRecorder.h
 * This file declares a module that provides audio samples using PortAudio.
 * @author Aaron Larisch
 */

#pragma once

#include <portaudio.h>
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include <mutex>

MODULE(PortAudioRecorder,
  REQUIRES(FrameInfo),
  PROVIDES(AudioData),
  LOADS_PARAMETERS(,
    (std::string)("") deviceName, /**< Name of audio device. */
    (unsigned)(4) channels, /**< Number of channels to capture. */
    (unsigned)(22050) sampleRate, /**< Sample rate to capture. */
    (float)(0.1f) latency 
  )
);

class PortAudioRecorder : public PortAudioRecorderBase
{
private:
  void update(AudioData& audioData);

  PaStream* stream = nullptr;
  PaStreamParameters inputParameters;
  unsigned noDataCount = 0;
  static std::mutex mutex;

public:
  PortAudioRecorder();
  ~PortAudioRecorder();
};
