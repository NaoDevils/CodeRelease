/**
* @file Platform/Common/Text2Speech.cpp
* Declaration of class Text2Speech for Windows and Linux.
*/


#include "Text2Speech.h"
#include <iostream>
#include <thread>
#include "Tools/Debugging/Debugging.h"
#include <portaudio.h>
#include <flite/flite.h>
#include "Platform/File.h"
#include "Platform/BHAssert.h"
#include <limits>
#include <queue>
#include <mutex>
#include "Platform/Thread.h"

Text2Speech* Text2Speech::instance = nullptr;

extern "C"
{
  void usenglish_init(cst_voice* v);
  cst_lexicon* cmulex_init(void);
}

struct Text2SpeechBuffer
{
  Text2SpeechBuffer();
  Text2SpeechBuffer(cst_wave* wave);

  using wave_ptr = std::unique_ptr<cst_wave, void (*)(cst_wave*)>;
  wave_ptr wave{nullptr, nullptr}; /* pointer to locked data buffer */
  unsigned long length = 0; /* length of data buffer */
  unsigned long position = 0; /* current position audio stram */
};

struct Text2SpeechData
{
  Text2SpeechData(cst_voice* voice);

  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long outSamples, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData);

  PaStream* stream = nullptr;
  cst_voice* voice = nullptr;
  bool threadNameSet = false;

  std::mutex mutex;
  std::queue<Text2SpeechBuffer> queue;
  static constexpr size_t maxQueueSize = 10;
};

Text2SpeechSetup::Text2SpeechSetup() : outputParameters(std::make_unique<PaStreamParameters>())
{
  // init portaudio
  PaError paerr;
  paerr = Pa_Initialize();

  outputParameters->device = paNoDevice;
  outputParameters->channelCount = 1;
  outputParameters->sampleFormat = paInt16;
  outputParameters->suggestedLatency = 0;
  outputParameters->hostApiSpecificStreamInfo = nullptr;

  if (paerr == paNoError)
  {
    outputParameters->device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters->device != paNoDevice)
      outputParameters->suggestedLatency = Pa_GetDeviceInfo(outputParameters->device)->defaultLowOutputLatency;
    else
      OUTPUT_ERROR("Cannot get portaudio default output device!");
  }
  else
    OUTPUT_ERROR("Failed to initialize portaudio: " << Pa_GetErrorText(paerr));

  // init flite only if audio device is present
  if (outputParameters->device != paNoDevice)
  {
    flite_init();
    flite_add_lang("eng", usenglish_init, cmulex_init);
    flite_add_lang("usenglish", usenglish_init, cmulex_init);

    auto voice = voices.begin();
    for (const char* voicePath : voicePaths)
    {
      *voice = voice_ptr(flite_voice_load((std::string(File::getBHDir()) + voiceDirectory + voicePath).c_str()), delete_voice);
      if (!*voice)
        OUTPUT_ERROR("Error: Unable to load flite voice " << voicePath << " for text2speech");
      ++voice;
    }
  }

  tts = std::make_unique<Text2Speech>(*this, Voice::male);
}

Text2SpeechSetup::~Text2SpeechSetup()
{
  tts.reset();
  Pa_Terminate();
}

Text2SpeechBuffer::Text2SpeechBuffer() {}
Text2SpeechBuffer::Text2SpeechBuffer(cst_wave* wave) : wave(wave, delete_wave), length(wave->num_samples) {}

int Text2SpeechData::paCallback(
    [[maybe_unused]] const void* inputBuffer, void* outputBuffer, unsigned long outSamples, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
{
  /* Cast data passed through stream to our structure. */
  Text2SpeechData* data = reinterpret_cast<Text2SpeechData*>(userData);
  short* out = reinterpret_cast<short*>(outputBuffer);
  if (!data->threadNameSet)
  {
    Thread<Text2SpeechData>::setName("PortAudioCallback");
    data->threadNameSet = true;
  }

  Text2SpeechBuffer* currentBuffer = nullptr;
  {
    std::lock_guard l(data->mutex);
    while (!data->queue.empty() && data->queue.front().position >= data->queue.front().length)
      data->queue.pop();

    if (!data->queue.empty())
      currentBuffer = &data->queue.front();
  }

  const unsigned long inSamples = currentBuffer ? currentBuffer->length - currentBuffer->position : 0;
  const unsigned long samples = std::min(outSamples, inSamples);

  if (currentBuffer)
  {
    const short* in = currentBuffer->wave->samples;
    std::copy(in + currentBuffer->position, in + currentBuffer->position + samples, out);
    currentBuffer->position += samples;
  }
  std::fill(out + samples, out + outSamples, short(0));

  return paContinue;
}

Text2SpeechData::Text2SpeechData(cst_voice* voice) : voice(voice) {}

Text2Speech::Text2Speech(const Text2SpeechSetup& setup, Text2SpeechSetup::Voice voice) : data(std::make_unique<Text2SpeechData>(setup.getVoice(voice)))
{
  ASSERT(!instance);
  instance = this;

  if (data->voice && setup.getOutputParameters()->device != paNoDevice)
  {
    PaError paerr;
    const int sampleRate = flite_get_param_int(data->voice->features, "sample_rate", 0);
    paerr = Pa_OpenStream(&data->stream, nullptr, setup.getOutputParameters(), sampleRate, paFramesPerBufferUnspecified, 0, &Text2SpeechData::paCallback, data.get());
    if (paerr != paNoError)
    {
      OUTPUT_ERROR("Failed to open portaudio stream: " << Pa_GetErrorText(paerr));
      return;
    }

    paerr = Pa_StartStream(data->stream);
    if (paerr != paNoError)
    {
      OUTPUT_ERROR("Failed to start portaudio stream: " << Pa_GetErrorText(paerr));
      return;
    }
  }
}

Text2Speech::~Text2Speech()
{
  ASSERT(instance);
  instance = nullptr;

  if (data->stream)
  {
    PaError paerr = Pa_StopStream(data->stream);
    if (paerr != paNoError)
    {
      OUTPUT_ERROR("Failed to stop portaudio stream: " << Pa_GetErrorText(paerr));
      return;
    }

    paerr = Pa_CloseStream(data->stream);
    if (paerr != paNoError)
    {
      OUTPUT_ERROR("Failed to close portaudio stream: " << Pa_GetErrorText(paerr));
      return;
    }
  }
}

void Text2Speech::text2Speech(std::string text)
{
  //in order to limit the memory footprint, limit the length of output to 200 chars
  if (text.length() > 200)
  {
    OUTPUT_WARNING("Text2Speech: output too long.");
    text.erase(200, std::string::npos);
  }

  if (!data->voice)
  {
    OUTPUT_TEXT("Say: " << text);
    return;
  }

  Text2SpeechBuffer buffer(flite_text_to_wave(text.c_str(), data->voice));

  //normalize wave data
  {
    short* const samplesBegin = buffer.wave->samples;
    short* const samplesEnd = buffer.wave->samples + buffer.length;
    const auto [min, max] = std::minmax_element(samplesBegin, samplesEnd);

    const short absmax = static_cast<short>(std::max(std::abs(*min), std::abs(*max)));
    if (absmax > 0)
    {
      //calculate factor 90% of max volume
      const float factor = std::clamp(std::numeric_limits<short>::max() * 0.9f / absmax, 0.f, 10.f);

      const auto scale = [factor](short v)
      {
        return static_cast<short>(factor * v);
      };
      std::transform(samplesBegin, samplesEnd, samplesBegin, scale);
    }
  }

  std::lock_guard l(data->mutex);

  //if too many sentences are queued, deny to add more.
  if (data->queue.size() >= data->maxQueueSize)
  {
    OUTPUT_ERROR("Text2Speech: too many sentences queued.");
    return;
  }
  else
  {
    data->queue.push(std::move(buffer));
  }
}
