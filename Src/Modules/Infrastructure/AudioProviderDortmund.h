/**
 * @file AudioProvider.cpp
 * This file declares a module that provides audio samples.
 * @author Thomas Röfer
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#else
#include <ctime>
#include <cstdio>
#include "Platform/File.h"
#endif
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"

STREAMABLE(AudioProviderDortmundParams,
{,
 (unsigned)(2) retries, /**< Number of tries to open one device. */
 (unsigned)(500) retryDelay, /**< Delay before a retry to open device. */
 (unsigned)(4) channels, /**< Number of channels to capture. */
 (unsigned)(22050) sampleRate, //44100/2 hz /**< Sample rate to capture. This variable will contain the framerate the driver finally selected. */
 (unsigned)(10000) maxFrames, /**< Maximum number of frames read in one cycle. */
 (bool)(false) simulateWhistleInSimulator, /**< if enabled a whistle sound is send in set to robots inside the simulator */
 (int)(-1) simulateSinusInSimulator, // 1 for 1 sinus, 2 for 2 sinus tones, 3 for 3 sinus tones
 (float)(-1.f) simulateNoiseInSimulator, // random noise level (set negative to disable)
 (std::string)("") audioFileName, // must be a headerless raw S16 file
});

MODULE(AudioProviderDortmund,
{ ,
  USES(GameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  DEFINES_PARAMETERS(
  {,
    (AudioProviderDortmundParams) params,
  }),
});

class AudioProviderDortmund : public AudioProviderDortmundBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;

  /**
   *  Method loads alsa driver and checks connection to the microphones.
   *  @return true if connection was established, false otherwise.
   */
  bool loadAudioDevice(const char* deviceName);

  bool floatConversionNeeded;
  std::vector<short> samplesS16;

#else
  FILE * audioFile;
  unsigned timestamp;
  void openAudioFile();
  void closeAudioFile();
  const std::string WHISTLE_FOLDER = "/Config/Sounds/Whistle/";
#endif
  void update(AudioData& audioData);
  bool connectionEstablished = false; /**< is set during initializing sound card connection/audioFile opening */

public:
  /**
  * Default constructor.
  */
  AudioProviderDortmund();

  /**
  * Destructor.
  */
  ~AudioProviderDortmund();

};
