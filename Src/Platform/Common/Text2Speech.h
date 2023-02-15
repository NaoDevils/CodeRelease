/**
* @file Platform/Common/Text2Speech.h
* Declaration of class Text2Speech for Windows and Linux.
*/

#pragma once

#include <string>
#include <memory>

typedef struct Text2speechDataStruct Text2speechData;
typedef struct PaStreamParameters PaStreamParameters;
typedef struct cst_voice_struct cst_voice;

enum class Text2SpeechStreamStatus
{
  uninitialized,
  initializing,
  idle,
  streaming,
};

enum class Text2SpeechInitStatus
{
  uninitialized,
  initializing,
  ready,
};

enum class Text2SpeechVoice
{
  male,
  female,
  output2console
};

/**
 * This class provides text2speech capabilities.
 */
class Text2Speech
{
public:
  Text2Speech();
  Text2Speech(Text2SpeechVoice voice);
  ~Text2Speech();

  static Text2Speech& getInstance();


  /**
   * NOTE: Do not call this directly, better use SystemCall::text2Speech(std::string text) for background processing.
   * The function converts the given text to speech and outputs the resulting audio stream.
	 */
  void text2Speech(std::string text);

private:
  Text2speechData* data;
  static Text2Speech* instance;
};


/**
 * This class provides init and shutdown methods for portaudio and flite.
 */
class Text2SpeechSetup
{
public:
  Text2SpeechSetup();
  ~Text2SpeechSetup();

  static void init();
  static std::atomic<Text2SpeechInitStatus> initialized;
  static PaStreamParameters* outputParameters;

  static cst_voice* male_voice;
  static cst_voice* female_voice;
};
