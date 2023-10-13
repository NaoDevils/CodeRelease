/**
 * @file AudioPlayer.cpp
 * This file implements a class that plays audio samples using PortAudio.
 * @author Aaron Larisch
 */


#include "AudioPlayer.h"

#include "Representations/Infrastructure/AudioData.h"
#include <portaudio.h>
#include <vector>
#include <iostream>
#include <cassert>

struct AudioPlayer::Pimpl
{
  bool initialized = false;
  PaStream* stream = nullptr;
  PaStreamParameters outputParameters;
  std::vector<float> convBuf;
  unsigned int sampleRate = 0;

  void init(int sampleRate, int channelCount);
};

AudioPlayer::AudioPlayer() : p(std::make_unique<Pimpl>()) {}

AudioPlayer::~AudioPlayer()
{
  if (p->stream)
  {
    if (Pa_IsStreamActive(p->stream))
      Pa_StopStream(p->stream);

    Pa_CloseStream(p->stream);
    p->stream = nullptr;
  }

  if (p->initialized)
    Pa_Terminate();
}

void AudioPlayer::play(const AudioData& audioData)
{
  if (audioData.samples.empty())
    return;

  if (!p->stream)
    p->init(audioData.sampleRate, audioData.channels);

  assert(p->sampleRate == audioData.sampleRate);

  PaError paerr;
  if (Pa_IsStreamStopped(p->stream))
  {
    paerr = Pa_StartStream(p->stream);
    assert(paerr == paNoError);
  }

  const unsigned int inputChannels = audioData.channels;
  const unsigned int outputChannels = static_cast<unsigned int>(p->outputParameters.channelCount);
  const unsigned long frames = static_cast<unsigned long>(audioData.samples.size() / inputChannels);

  const signed long available = Pa_GetStreamWriteAvailable(p->stream);
  assert(available >= 0);

  if (inputChannels != outputChannels)
  {
    // channel mapping
    p->convBuf.clear();
    p->convBuf.resize(frames * outputChannels, 0.f);
    for (size_t f = 0; f < frames; ++f)
    {
      for (size_t c = 0; c < std::max(inputChannels, outputChannels); ++c)
      {
        p->convBuf[f * outputChannels + (c % outputChannels)] += audioData.samples[f * inputChannels + (c % inputChannels)];
      }
    }
    paerr = Pa_WriteStream(p->stream, p->convBuf.data(), std::min(frames, static_cast<unsigned long>(available)));
  }
  else
  {
    paerr = Pa_WriteStream(p->stream, audioData.samples.data(), std::min(frames, static_cast<unsigned long>(available)));
  }

  if (paerr != paNoError)
    std::cerr << "AudioPlayer: Pa_WriteStream failed: " << Pa_GetErrorText(paerr) << std::endl;
}

void AudioPlayer::Pimpl::init(int sampleRate, int channelCount)
{
  [[maybe_unused]] PaError paerr;
  if (!initialized)
  {
    paerr = Pa_Initialize();
    assert(paerr == paNoError);
  }

  outputParameters.device = Pa_GetDefaultOutputDevice();
  assert(outputParameters.device != paNoDevice);

  const PaDeviceInfo* info = Pa_GetDeviceInfo(outputParameters.device);
  outputParameters.channelCount = std::min(channelCount, info->maxOutputChannels);
  outputParameters.sampleFormat = paFloat32;
  outputParameters.suggestedLatency = info->defaultHighOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = nullptr;

  this->sampleRate = sampleRate;

  paerr = Pa_OpenStream(&stream, nullptr, &outputParameters, this->sampleRate, paFramesPerBufferUnspecified, 0, nullptr, nullptr);
  assert(paerr == paNoError);
}
