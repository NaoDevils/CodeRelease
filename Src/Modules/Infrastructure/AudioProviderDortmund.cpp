/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 */

#include "AudioProviderDortmund.h"
#include "Tools/Settings.h"

#define PSEUDO_AUDIO_FILE "pseudoAudio.float" // must be a headerless raw floats file

MAKE_MODULE(AudioProviderDortmund, cognitionInfrastructure)

#ifdef TARGET_ROBOT

AudioProviderDortmund::AudioProviderDortmund()
{
  unsigned i;
  for(i = 0; i < retries; ++i)
  {
      if (snd_pcm_open(&handle, "plughw:0", snd_pcm_stream_t(SND_PCM_STREAM_CAPTURE | SND_PCM_NONBLOCK), 0) >= 0)
      {
          break;
      }
    SystemCall::sleep(retryDelay);
  }
  if (i >= retries)
  {
      OUTPUT_WARNING("snd_pcm_open() failed. No audio connection established!");
      connectionEstablished = false;
      return;
  }

  snd_pcm_hw_params_t* params;
  VERIFY(!snd_pcm_hw_params_malloc(&params));
  VERIFY(!snd_pcm_hw_params_any(handle, params));
  VERIFY(!snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED));
  VERIFY(!snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_FLOAT)); //FLOAT_LE S16_LE S32_LE

  unsigned int preferredSampleRate = sampleRate;
  VERIFY(!snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, 0));
  if (preferredSampleRate != sampleRate)
      OUTPUT_WARNING("AudioProvider: Trying to set sampleRate to" << preferredSampleRate << "failed. Overwritten by" << sampleRate);

  unsigned int preferredChannels = channels;
  VERIFY(!snd_pcm_hw_params_set_channels_near(handle, params, &channels));
  if (preferredChannels != channels)
    OUTPUT_WARNING("AudioProvider: Trying to set channel to" << preferredChannels << "failed. Overwritten by" << channels);

  if (!snd_pcm_hw_params(handle, params))
  {
      VERIFY(!snd_pcm_prepare(handle));

      ASSERT(channels <= 4);
      float buf[4];
      VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
      connectionEstablished = true;
  }
  else
  {
      OUTPUT_WARNING("snd_pcm_hw_params() failed. No audio connection established!");
      connectionEstablished = false;
  }
  snd_pcm_hw_params_free(params);
}

AudioProviderDortmund::~AudioProviderDortmund()
{
    if (connectionEstablished)
      snd_pcm_close(handle);
}

void AudioProviderDortmund::update(AudioDataDortmund& audioData)
{
    audioData.channels = channels;
    audioData.sampleRate = sampleRate;
    audioData.isValid = false;

    if (theGameInfo.state != STATE_SET ||
        Global::getSettings().gameMode == Settings::penaltyShootout ||
        Global::getSettings().gameMode == Settings::dropIn ||
        !connectionEstablished)
        return;

  unsigned available = std::min((unsigned) snd_pcm_avail(handle), maxFrames);
  audioData.samples.resize(available * channels);

  int status = snd_pcm_readi(handle, audioData.samples.data(), available);
  if(status < 0)
  {
	/** ErrorCode in status
			-EBADFD 77
			-EPIPE 32
			-ESTRPIPE 86
	*/
    OUTPUT_WARNING("Lost audio stream (" << snd_strerror(status) << "), recovering...");
	
    snd_pcm_recover(handle, status, 1);
    ASSERT(channels <= 4);
    float buf[4];
    VERIFY(snd_pcm_readi(handle, buf, 1) >= 0);
    audioData.samples.clear();
  }
  else
  {
      audioData.isValid = true;
  }
}

#else // !defined(TARGET_ROBOT)

AudioProviderDortmund::AudioProviderDortmund()
{
    std::srand(std::time(nullptr));

    std::string filePath(File::getBHDir());
    filePath += "/Config/Sounds/";
    filePath += PSEUDO_AUDIO_FILE;
    audioFile = fopen(filePath.data(), "rb");

    if (audioFile == nullptr || ferror(audioFile))
    {
        connectionEstablished = false;
        OUTPUT_WARNING("File " << PSEUDO_AUDIO_FILE << " could not be opened. No audio connection established.");
    }
    else
        connectionEstablished = true;

    timestamp = theFrameInfo.time;
}

AudioProviderDortmund::~AudioProviderDortmund()
{
    if (audioFile != nullptr)
      fclose(audioFile);
}

void AudioProviderDortmund::update(AudioDataDortmund& audioData)
{
    audioData.channels = channels;
    audioData.sampleRate = sampleRate;
    audioData.isValid = false;

    if (theGameInfo.state != STATE_SET ||
        Global::getSettings().gameMode == Settings::penaltyShootout ||
        Global::getSettings().gameMode == Settings::dropIn ||
        !connectionEstablished)
        return;

    ////create 3 sinus tones plus noise as pseudo audio data
    //audioData.samples.resize(1000);
    //for (int i = 0; i < audioData.samples.size(); i++)
    //{
    //    audioData.samples[i] = 0.f;
    //    //add noise
    //    audioData.samples[i] += (float)((std::rand() % 201) - 100) / 100.f;
    //    //add sinus
    //    audioData.samples[i] += std::sin((float)i / 2);
    //    audioData.samples[i] += std::sin(i);
    //    audioData.samples[i] += std::sin(i*2);
    //}

    int deltaTime = theFrameInfo.getTimeSince(timestamp);
    audioData.samples.resize(sampleRate * deltaTime / 1000);
    timestamp = theFrameInfo.time;

    float element;
    int i = 0;
    while (i < audioData.samples.size())
    {
        if (fread(&element, sizeof(element), 1, audioFile) == 1)
        {
            audioData.samples[i] = element;
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

#endif
