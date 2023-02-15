/**
* @file Platform/Common/Text2Speech.cpp
* Declaration of class Text2Speech for Windows and Linux.
*/


#include "Text2Speech.h"
#include <iostream>
#include <atomic>
#include <thread>
#include "Tools/Debugging/Debugging.h"
#include <portaudio.h>
#include <flite/flite.h>
#include "Platform/File.h"
#include <climits>

//todo: use eSpeak until we've solved portaudio issues on the robot
//#define USE_ESPEAK 1

extern "C"
{
  void usenglish_init(cst_voice* v);
  cst_lexicon* cmulex_init(void);
  void flite_set_lang_list(void);
}


typedef struct Text2speechBufferStruct Text2speechBuffer;
typedef struct Text2speechBufferStruct
{
  cst_wave* lpData; /* pointer to locked data buffer */
  unsigned long dwBufferLength; /* length of data buffer */
  unsigned long dwBufferPos; /* current position audio stram */
  Text2speechBuffer* nextBuffer; /* next buffer */
} Text2speechBuffer;

typedef struct Text2speechDataStruct
{
  PaStream* stream;
  Text2SpeechVoice voice;
  std::atomic<Text2SpeechStreamStatus> status;
  int in_reset;
  void** fq;
  Text2speechBuffer* currentBuffer;
  Text2speechBuffer* lastBuffer;
  std::atomic_int queueLen;
  std::atomic_int queueMaxLen;
  Text2speechDataStruct()
      : stream(NULL), voice(Text2SpeechVoice::female), status(Text2SpeechStreamStatus::uninitialized), in_reset(0), fq(NULL), currentBuffer(NULL), lastBuffer(NULL)
  {
  }
} Text2speechData;

Text2SpeechSetup text2speechSetup = Text2SpeechSetup();

static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
{
  /* Cast data passed through stream to our structure. */
  Text2speechData* data = (Text2speechData*)userData;
  unsigned short* out16Bit = (unsigned short*)outputBuffer;
  unsigned long i;
  (void)inputBuffer; /* Prevent unused variable warning. */

  if (data->currentBuffer == NULL)
  {
    //this should not happen, stop stream.
    for (i = 0; i < frameCount; i++)
    {
      out16Bit[i] = (unsigned short)0;
    }
    return paComplete;
  }

  unsigned long numSamples = data->currentBuffer->dwBufferLength - data->currentBuffer->dwBufferPos;
  unsigned long n = frameCount < numSamples ? frameCount : numSamples;

  unsigned short* lpData16 = (unsigned short*)data->currentBuffer->lpData->samples;

  for (i = 0; i < n; i++)
  {
    out16Bit[i] = lpData16[data->currentBuffer->dwBufferPos + i];
  }
  for (i = n; i < frameCount; i++)
  {
    out16Bit[i] = (unsigned short)0;
  }
  data->currentBuffer->dwBufferPos += n;
  if (data->currentBuffer->dwBufferPos >= data->currentBuffer->dwBufferLength)
  {
    data->queueLen--;
    if (data->currentBuffer->nextBuffer == NULL)
    {
      Text2speechBuffer* finishedBuffer = data->currentBuffer;
      data->currentBuffer = NULL;

      free(finishedBuffer->lpData->samples);
      free(finishedBuffer->lpData);
      delete finishedBuffer;
      return paComplete;
    }
    else
    {
      Text2speechBuffer* finishedBuffer = data->currentBuffer;
      data->currentBuffer = data->currentBuffer->nextBuffer;

      free(finishedBuffer->lpData->samples);
      free(finishedBuffer->lpData);
      delete finishedBuffer;
      return paContinue;
    }
  }
  else
  {
    return paContinue;
  }
}

static void paFinishedCallback(void* userData)
{
  Text2speechData* data = (Text2speechData*)userData;
  data->status = Text2SpeechStreamStatus::idle;
  //TRACE("Finished stream %p", userData);
}

Text2Speech& Text2Speech::getInstance()
{
  //if you'd like to have one stream per robot in simulator you need to return a new instance per robot
  //return *(new Text2Speech());

  //otherwise use a single instance, text2speech output will then be queued
  if (instance == NULL)
  {
    //instance = std::make_unique<Text2Speech>();
    instance = new Text2Speech();
  }
  //return *instance.get();
  return *instance;
}

Text2Speech* Text2Speech::instance;

void Text2Speech::text2Speech(std::string text)
{
#ifdef USE_ESPEAK
  SystemCall::text2SpeechESpeak(text.c_str());
#else

  //TRACE("Start text2speech: %s", text);

  //init if not already done
  text2speechSetup.init();

  //if more than 10 sentences are queued, deny to add more.
  if (data->queueLen >= data->queueMaxLen)
  {
#ifdef TARGET_ROBOT
    OUTPUT_ERROR("Text2Speech: too many sentences queued.");
#else
    std::cerr << "Text2Speech: too many sentences queued." << std::endl;
#endif
    return;
  }

  //in order to limit the memory footprint, limit the length of output to 200 chars
  if (text.length() > 200)
  {
#ifdef TARGET_ROBOT
    OUTPUT_WARNING("Text2Speech: output too long.");
#else
    std::cerr << "Text2Speech: output too long." << std::endl;
#endif
    text.erase(200, std::string::npos);
  }

  if (data->voice == Text2SpeechVoice::output2console)
  {
#ifdef TARGET_ROBOT
    OUTPUT_TEXT("Say: " << text);
#else
    std::cout << "Say: " << text;
#endif
    return;
  }

  const char* textStr = text.c_str();
  cst_voice* voice = (data->voice == Text2SpeechVoice::male) ? Text2SpeechSetup::male_voice : Text2SpeechSetup::female_voice;
  cst_wave* w = flite_text_to_wave(textStr, voice);

  //normalize wave data
  short* lpData16 = (short*)w->samples;
  int i;
  short max = 0;
  for (i = 0; i < w->num_samples; i++)
  {
    //detect max value
    short v = lpData16[i] > 0 ? lpData16[i] : -lpData16[i];
    if (v > max)
      max = v;
  }
  if (max > 0)
  {
    //calculate factor 90% of max volume
    float factor = ((float)SHRT_MAX / (float)max) * 0.9f;
    if (factor > 10.f)
      factor = 10;
    for (i = 0; i < w->num_samples; i++)
    {
      //normalize
      short v = (short)lpData16[i];
      short v_new = (short)((float)v * factor);
      lpData16[i] = (unsigned short)v_new;
    }
  }

  Text2SpeechSetup::outputParameters->channelCount = w->num_channels;
  Text2speechBuffer* buffer = new Text2speechBuffer();
  buffer->dwBufferPos = 0;
  buffer->dwBufferLength = w->num_samples;
  buffer->lpData = w;
  buffer->nextBuffer = NULL;

  PaError paerr;

  Text2SpeechStreamStatus expectedUninit = Text2SpeechStreamStatus::uninitialized;
  Text2SpeechStreamStatus expectedIdle = Text2SpeechStreamStatus::idle;
  if (data->status.compare_exchange_strong(expectedUninit, Text2SpeechStreamStatus::initializing))
  {
    data->currentBuffer = buffer;
    data->lastBuffer = buffer;
    data->queueLen++;

    paerr = Pa_OpenStream(&data->stream, NULL, Text2SpeechSetup::outputParameters, w->sample_rate, paFramesPerBufferUnspecified, 0, &paCallback, data);
    if (paerr != paNoError)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Failed to open portaudio stream:" << paerr);
      OUTPUT_ERROR("Error message:" << Pa_GetErrorText(paerr));
#else
      std::cerr << "Failed to open portaudio stream:" << paerr << std::endl;
      std::cerr << "Error message:" << Pa_GetErrorText(paerr) << std::endl;
#endif
      return;
    }
    paerr = Pa_SetStreamFinishedCallback(data->stream, &paFinishedCallback);
    if (paerr != paNoError)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Failed to set portaudio finished callback:" << paerr);
      OUTPUT_ERROR("Error message:" << Pa_GetErrorText(paerr));
#else
      std::cerr << "Failed to set portaudio finished callback:" << paerr << std::endl;
      std::cerr << "Error message:" << Pa_GetErrorText(paerr) << std::endl;
#endif
      return;
    }

    paerr = Pa_StartStream(data->stream);
    if (paerr != paNoError)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Failed to start portaudio stream:" << paerr);
      OUTPUT_ERROR("Error message:" << Pa_GetErrorText(paerr));
#else
      std::cerr << "Failed to start portaudio stream:" << paerr << std::endl;
      std::cerr << "Error message:" << Pa_GetErrorText(paerr) << std::endl;
#endif
      return;
    }
    data->status = Text2SpeechStreamStatus::streaming;
  }
  else if (data->status.compare_exchange_strong(expectedIdle, Text2SpeechStreamStatus::streaming))
  {
    //we need to stop the stream before starting it again
    paerr = Pa_StopStream(data->stream);
    if (paerr != paNoError)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Failed to stop portaudio stream:" << paerr);
      OUTPUT_ERROR("Error message:" << Pa_GetErrorText(paerr));
#else
      std::cerr << "Failed to stop portaudio stream:" << paerr << std::endl;
      std::cerr << "Error message:" << Pa_GetErrorText(paerr) << std::endl;
#endif
      return;
    }
    data->currentBuffer = buffer;
    data->lastBuffer = buffer;
    data->queueLen++;

    paerr = Pa_StartStream(data->stream);
    if (paerr != paNoError)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Failed to start portaudio stream:" << paerr);
      OUTPUT_ERROR("Error message:" << Pa_GetErrorText(paerr));
#else
      std::cerr << "Failed to start portaudio stream:" << paerr << std::endl;
      std::cerr << "Error message:" << Pa_GetErrorText(paerr) << std::endl;
#endif
      return;
    }
  }
  else if (data->status == Text2SpeechStreamStatus::streaming)
  {
    data->lastBuffer->nextBuffer = buffer;
    data->lastBuffer = buffer;
    data->queueLen++;
  }
  else if (data->status == Text2SpeechStreamStatus::initializing)
  {
    //wait for a different thread to complete init
    while (data->status.load() == Text2SpeechStreamStatus::initializing)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    data->lastBuffer->nextBuffer = buffer;
    data->lastBuffer = buffer;
    data->queueLen++;
  }
#endif
}

Text2Speech::Text2Speech()
{
#ifndef USE_ESPEAK
  data = new Text2speechData();
  data->status = Text2SpeechStreamStatus::uninitialized;
  data->currentBuffer = NULL;
  data->lastBuffer = NULL;
  data->stream = NULL;
  data->voice = Text2SpeechVoice::male;

  data->queueLen = 0;
  data->queueMaxLen = 10;
#endif
}

Text2Speech::Text2Speech(Text2SpeechVoice voice)
{
#ifndef USE_ESPEAK
  data = new Text2speechData();
  data->status = Text2SpeechStreamStatus::uninitialized;
  data->currentBuffer = NULL;
  data->lastBuffer = NULL;
  data->stream = NULL;
  data->voice = voice;

  data->queueLen = 0;
  data->queueMaxLen = 10;
#endif
}

Text2Speech::~Text2Speech()
{
#ifndef USE_ESPEAK
  //TRACE("Destructor ~Text2Speech called %p", data);
  if (data->status == Text2SpeechStreamStatus::streaming)
    Pa_StopStream(data->stream);
  delete data;
#endif
}


Text2SpeechSetup::Text2SpeechSetup() {}


void Text2SpeechSetup::init()
{
#ifndef USE_ESPEAK
  if (initialized == Text2SpeechInitStatus::ready)
    return;

  Text2SpeechInitStatus expected = Text2SpeechInitStatus::uninitialized;
  if (!initialized.compare_exchange_strong(expected, Text2SpeechInitStatus::initializing))
  {
    //wait for a different thread to complete init
    while (initialized.load() != Text2SpeechInitStatus::ready)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  else
  {
    // init portaudio
    PaError paerr;
    paerr = Pa_Initialize();
    if (paerr != paNoError)
    {
      fprintf(stderr, "Failed to initialize portaudio: %d\n", paerr);
      fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(paerr));
      return;
    }
    outputParameters = new PaStreamParameters();
    outputParameters->device = Pa_GetDefaultOutputDevice(); /* default output device */
    if (outputParameters->device == paNoDevice)
    {
      fprintf(stderr, "Failed to open portaudio default output device : % d\n", paerr);
      fprintf(stderr, "Error message : %s\n", Pa_GetErrorText(paerr));
      return;
    }
    outputParameters->suggestedLatency = Pa_GetDeviceInfo(outputParameters->device)->defaultLowOutputLatency;
    outputParameters->hostApiSpecificStreamInfo = NULL;
    outputParameters->channelCount = 1;
    outputParameters->sampleFormat = paInt16;

    // init flite
    flite_init();
    flite_add_lang("eng", usenglish_init, cmulex_init);
    flite_add_lang("usenglish", usenglish_init, cmulex_init);

    male_voice = flite_voice_load((std::string(File::getBHDir()) + "/Config/Voices/cmu_us_bdl.flitevox").c_str());
    if (male_voice == NULL)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Error: Unable to load flite male voice for text2speech");
#else
      std::cerr << "Error: Unable to load flite male voice for text2speech" << std::endl;
#endif
      return;
    }

    female_voice = flite_voice_load((std::string(File::getBHDir()) + "/Config/Voices/cmu_us_clb.flitevox").c_str());
    if (female_voice == NULL)
    {
#ifdef TARGET_ROBOT
      OUTPUT_ERROR("Error: Unable to load flite female voice for text2speech");
#else
      std::cerr << "Unable to load flite female voice for text2speech" << std::endl;
#endif
      return;
    }

    initialized = Text2SpeechInitStatus::ready;
  }
#endif
}

Text2SpeechSetup::~Text2SpeechSetup(){
#ifndef USE_ESPEAK
//TRACE("Destructor ~Text2SpeechSetup called");

//fixme: pa_terminate hangs most of the times
//fixme: it should be called, when is terminated but currently this is called when loading a different setting, too
//Pa_Terminate();
//delete outputParameters;
#endif
}

std::atomic<Text2SpeechInitStatus> Text2SpeechSetup::initialized(Text2SpeechInitStatus::uninitialized);
PaStreamParameters* Text2SpeechSetup::outputParameters = NULL;
cst_voice* Text2SpeechSetup::male_voice = NULL;
cst_voice* Text2SpeechSetup::female_voice = NULL;
