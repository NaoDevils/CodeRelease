/**
 * @file PortAudioRecorder.cpp
 * This file implements a module that provides audio samples using PortAudio.
 * @author Aaron Larisch
 */

#include "PortAudioRecorder.h"
#include "Tools/Settings.h"

MAKE_MODULE(PortAudioRecorder, cognitionInfrastructure)

std::mutex PortAudioRecorder::mutex;

PortAudioRecorder::PortAudioRecorder()
{
  // PortAudio API is not thread-safe.
  std::lock_guard<std::mutex> l(mutex);

  PaError paerr = Pa_Initialize();
  if (paerr != paNoError)
  {
    OUTPUT_ERROR("PortAudioRecorder: Failed to initialize: " << Pa_GetErrorText(paerr) << "(" << paerr << ")");
    return;
  }

  inputParameters.device = Pa_GetDefaultInputDevice();
  if (!deviceName.empty())
  {
    // get device by name
    for (PaDeviceIndex i = 0; i < Pa_GetDeviceCount(); ++i)
    {
      if (std::string(Pa_GetDeviceInfo(i)->name) == deviceName)
      {
        inputParameters.device = i;
        break;
      }
    }
  }

  if (inputParameters.device == paNoDevice)
  {
    OUTPUT_ERROR("PortAudioRecorder: No input device!");
    return;
  }

  const PaDeviceInfo* info = Pa_GetDeviceInfo(inputParameters.device);
  inputParameters.channelCount = std::min(static_cast<int>(channels), info->maxInputChannels);
  inputParameters.sampleFormat = paFloat32;
  //inputParameters.suggestedLatency = info->defaultHighInputLatency;
  inputParameters.suggestedLatency = latency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  paerr = Pa_OpenStream(&stream, &inputParameters, nullptr, sampleRate, paFramesPerBufferUnspecified, 0, nullptr, nullptr);
  if (paerr != paNoError)
  {
    OUTPUT_ERROR("PortAudioRecorder: Pa_OpenStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")");
    return;
  }
}

PortAudioRecorder::~PortAudioRecorder()
{
  std::lock_guard<std::mutex> l(mutex);

  if (stream)
  {
    if (Pa_IsStreamActive(stream))
      Pa_StopStream(stream);

    Pa_CloseStream(stream);
    stream = nullptr;
  }

  Pa_Terminate();
}

void PortAudioRecorder::update(AudioData& audioData)
{
  audioData.isValid = false;
  audioData.samples.clear();

  if (!stream)
    return;

  PaError paerr;
  if (Pa_IsStreamStopped(stream))
  {
    // fill information about audio device once
    const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(inputParameters.device);
    const PaStreamInfo* streamInfo = Pa_GetStreamInfo(stream);
    audioData.device = deviceInfo->name;
    audioData.api = Pa_GetHostApiInfo(deviceInfo->hostApi)->name;
    audioData.latency = streamInfo->inputLatency;
    audioData.channels = inputParameters.channelCount;
    audioData.sampleRate = static_cast<unsigned>(streamInfo->sampleRate);

    paerr = Pa_StartStream(stream);
    if (paerr != paNoError)
    {
      std::lock_guard<std::mutex> l(mutex);
      OUTPUT_ERROR("PortAudioRecorder: Pa_StartStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")");
      Pa_CloseStream(stream);
      stream = nullptr;
      return;
    }
  }

  signed long available = Pa_GetStreamReadAvailable(stream);
  if (available < 0)
  {
    OUTPUT_ERROR("PortAudioRecorder: Pa_GetStreamReadAvailable failed: " << Pa_GetErrorText(static_cast<PaError>(available)) << "(" << static_cast<PaError>(available) << ")");
    return;
  }

  // restart stream if no data is available over a longer period of time
  if (available == 0)
  {
    if (++noDataCount > 30)
    {
      OUTPUT_WARNING("PortAudioRecorder: Data timeout, restarting stream");
      Pa_StopStream(stream);
      noDataCount = 0;
    }
    return;
  }
  noDataCount = 0;

  audioData.samples.resize(available * audioData.channels);
  paerr = Pa_ReadStream(stream, audioData.samples.data(), available);
  if (paerr != paNoError)
  {
    OUTPUT_ERROR("PortAudioRecorder: Pa_ReadStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")");
    audioData.samples.clear();

    // restart stream immediately after timeout
    if (paerr == paTimedOut)
      Pa_StopStream(stream);

    return;
  }
  audioData.isValid = true;
}
