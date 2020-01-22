/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 */

#include "AudioProviderDortmund.h"
#include "Tools/Settings.h"

MAKE_MODULE(AudioProviderDortmund, cognitionInfrastructure)

#ifdef TARGET_ROBOT

AudioProviderDortmund::AudioProviderDortmund()
{
  samplesS16 = std::vector<short>();

  // load four microphone device
  if (!connectionEstablished)
    loadAudioDevice("multi");
  // failed: fall back on stereo device
  if (!connectionEstablished) {
    params.channels = 2;
    loadAudioDevice("plughw:0,0");
  }

}

bool AudioProviderDortmund::loadAudioDevice(const char* deviceName)
{
  unsigned i = 0;
  for(i = 0; i < params.retries; i++)
  {
      if (snd_pcm_open(&handle, deviceName, snd_pcm_stream_t(SND_PCM_STREAM_CAPTURE | SND_PCM_NONBLOCK), 0) >= 0)
      {
          break;
      }
    SystemCall::sleep(params.retryDelay);
  }
  if (i >= params.retries)
  {
      OUTPUT_WARNING("snd_pcm_open() failed on device" << deviceName << ". No audio connection established!");
      connectionEstablished = false;
      return connectionEstablished;
  }

  snd_pcm_hw_params_t* hw_params;
  VERIFY(!snd_pcm_hw_params_malloc(&hw_params));
  VERIFY(!snd_pcm_hw_params_any(handle, hw_params));
  VERIFY(!snd_pcm_hw_params_set_access(handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED));
  // creating floats directly would be great but cannot be guarateed
  // whistle detection will convert s16 to float
  if(snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_FORMAT_FLOAT_LE)) { //FLOAT_LE S16_LE S32_LE
    OUTPUT_WARNING("AudioProvider: Could not set audio format to SND_PCM_FORMAT_FLOAT_LE");

    floatConversionNeeded = true;
    if(snd_pcm_hw_params_set_format(handle, hw_params, SND_PCM_FORMAT_S16_LE)) { //FLOAT_LE S16_LE S32_LE
      OUTPUT_WARNING("AudioProvider: Could not set audio format to SND_PCM_FORMAT_S16_LE");
      connectionEstablished = false;
      snd_pcm_close(handle);
      return connectionEstablished;
    }
  } else { floatConversionNeeded = false; }

  unsigned int preferredSampleRate = params.sampleRate;
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, hw_params, &params.sampleRate, 0));
  if (preferredSampleRate != params.sampleRate)
      OUTPUT_WARNING("AudioProvider: Trying to set sampleRate to" << preferredSampleRate << "failed. Overwritten by" << params.sampleRate);

  unsigned int preferredChannels = params.channels;
  VERIFY(!snd_pcm_hw_params_set_channels_near(handle, hw_params, &params.channels));
  if (preferredChannels != params.channels)
    OUTPUT_WARNING("AudioProvider: Trying to set channel to" << preferredChannels << "failed. Overwritten by" << params.channels);

  if (!snd_pcm_hw_params(handle, hw_params))
  {
      VERIFY(!snd_pcm_prepare(handle));

      ASSERT(params.channels <= 4);
      float buf[4];
      VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
      connectionEstablished = true;
  }
  else
  {
      OUTPUT_WARNING("snd_pcm_hw_params() failed. No audio connection established!");
      snd_pcm_close(handle);
      connectionEstablished = false;
  }
  snd_pcm_hw_params_free(hw_params);

  // for debug output (TODO remove or change to OUTPUT_TEXT when working safe)
  OUTPUT_TEXT("snd_pcm_open() succeeded on device " << deviceName << ": " << params.channels << " channels on " << params.sampleRate << "Hz.");
  return connectionEstablished;
}

AudioProviderDortmund::~AudioProviderDortmund()
{
    if (connectionEstablished)
      snd_pcm_close(handle);
}

void AudioProviderDortmund::update(AudioData& audioData)
{

  audioData.channels = params.channels;
  audioData.sampleRate = params.sampleRate;
  audioData.isValid = false;

  if (Global::getSettings().gameMode == Settings::demoIRF)
  {
    if (!connectionEstablished)
      return;
  }
  else if (theGameInfo.state != STATE_SET ||
      Global::getSettings().gameMode == Settings::penaltyShootout ||
      (theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) ||
           !connectionEstablished)
  {
    audioData.samples.clear();
    return;
  }

  unsigned available = std::min((unsigned) snd_pcm_avail(handle), params.maxFrames);
  if (floatConversionNeeded)
    samplesS16.resize(available * params.channels);
  audioData.samples.resize(available * params.channels);

  int status;
  if (floatConversionNeeded)
    status = snd_pcm_readi(handle, samplesS16.data(), available);
  else
    status = snd_pcm_readi(handle, audioData.samples.data(), available);

  if(status < 0)
  {
	/** ErrorCode in status
			-EBADFD 77
			-EPIPE 32
			-ESTRPIPE 86
	*/
    OUTPUT_WARNING("Lost audio stream (" << snd_strerror(status) << "), recovering...");
	
    snd_pcm_recover(handle, status, 1);
    ASSERT(params.channels <= 4);
    float buf[4];
    VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
    if (floatConversionNeeded)
      samplesS16.clear();
    audioData.samples.clear();
  }
  else
  {
      audioData.isValid = true;
  }

  if (floatConversionNeeded)
      for (unsigned i = 0; i < audioData.samples.size(); i++)
          audioData.samples[i] = static_cast<float>(samplesS16[i])
              / static_cast<float>(std::numeric_limits<short>::max());
}

#else // !defined(TARGET_ROBOT)

AudioProviderDortmund::AudioProviderDortmund()
{
    audioFile = nullptr;
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    if (params.simulateWhistleInSimulator)
      openAudioFile();
}

AudioProviderDortmund::~AudioProviderDortmund()
{
    closeAudioFile();
}

std::string lastAudioFile = "";
void AudioProviderDortmund::openAudioFile()
{
  if (lastAudioFile == params.audioFileName)
    return;

  closeAudioFile();

  std::string filePath(File::getBHDir());
  filePath += WHISTLE_FOLDER;
  filePath += params.audioFileName;
  audioFile = fopen(filePath.data(), "rb");

  if (audioFile == nullptr || ferror(audioFile))
  {
      connectionEstablished = false;
      OUTPUT_WARNING("AudioFile '" << params.audioFileName << "' could not be opened. File is expected in folder '" << WHISTLE_FOLDER << "'.");
  }
  else
  {
      connectionEstablished = true;
  }

  timestamp = theFrameInfo.time;

  lastAudioFile = params.audioFileName;
}

void AudioProviderDortmund::closeAudioFile()
{
  if (audioFile != nullptr)
    fclose(audioFile);
}

void AudioProviderDortmund::update(AudioData& audioData)
{
  MODIFY("module:AudioProviderDortmund:params", params);

    audioData.channels = params.channels;
    audioData.sampleRate = params.sampleRate;
    audioData.isValid = false;

    if (theGameInfo.state != STATE_SET ||
        Global::getSettings().gameMode == Settings::penaltyShootout ||
        !params.simulateWhistleInSimulator)
    {
      audioData.samples.clear();
      return;
    }

    openAudioFile();

    int deltaTime = theFrameInfo.getTimeSince(timestamp);
    audioData.samples.resize(params.sampleRate * deltaTime / 1000);
    timestamp = theFrameInfo.time;

    if (params.simulateSinusInSimulator >= 0) {
      //create 3 sinus tones plus noise as pseudo audio data
      for (unsigned i = 0; i < audioData.samples.size(); i++)
      {
          audioData.samples[i] = 0.f;
          //add sinus
          if (params.simulateSinusInSimulator >= 1)
            audioData.samples[i] += std::sin((float)i / 2);
          if (params.simulateSinusInSimulator >= 2)
            audioData.samples[i] += std::sin((float)i);
          if (params.simulateSinusInSimulator >= 3)
            audioData.samples[i] += std::sin((float)i*2);
      }
      audioData.isValid = true;

    } else {
      short element;
      size_t i = 0;
      if (connectionEstablished) {
        while (i < audioData.samples.size())
        {
          if (fread(&element, sizeof(element), 1, audioFile) == 1)
          {
              audioData.samples[i] = static_cast<float>(element) / static_cast<float>(std::numeric_limits<short>::max());
              //printf("audiofile element in float: %f\n", (static_cast<float>(element) / static_cast<float>(std::numeric_limits<short>::max())));
              i++;
          }
          else
          {
              if (feof(audioFile))
                  rewind(audioFile);
              else
                  OUTPUT_ERROR("Could not read pseudo audio and did not reach EOF.");
          }
        }
      audioData.isValid = true;
      }
    }
    //add noise
    if (params.simulateNoiseInSimulator >= 0.0f)
    {
      for (unsigned i = 0; i < audioData.samples.size(); i++)
        audioData.samples[i] += (float)((std::rand() % 2001) - 1000) / 1000.f * params.simulateNoiseInSimulator;
      audioData.isValid = true;
    }

}

#endif
