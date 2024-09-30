/**
* @file Platform/Common/Text2Speech.h
* Declaration of class Text2Speech for Windows and Linux.
*/

#pragma once

#include <string>
#include <memory>
#include <array>

struct Text2SpeechData;
struct PaStreamParameters;
typedef struct cst_voice_struct cst_voice;
class Text2Speech;


/**
 * This class provides init and shutdown methods for portaudio and flite.
 */
class Text2SpeechSetup
{
public:
  Text2SpeechSetup();
  ~Text2SpeechSetup();

  enum class Voice
  {
    male,
    female,
    output2console // must be last element
  };

  cst_voice* getVoice(Voice voice) const
  {
    const size_t i = static_cast<size_t>(voice);
    return i < static_cast<size_t>(Voice::output2console) ? voices[i].get() : nullptr;
  }
  const PaStreamParameters* getOutputParameters() const { return outputParameters.get(); }

private:
  static constexpr const char* voiceDirectory{"/Config/Voices/"};
  static constexpr std::array<const char*, static_cast<size_t>(Voice::output2console)> voicePaths{"cmu_us_bdl.flitevox", "cmu_us_clb.flitevox"};

  using voice_ptr = std::unique_ptr<cst_voice, void (*)(cst_voice*)>;
  std::array<voice_ptr, static_cast<size_t>(Voice::output2console)> voices{voice_ptr(nullptr, nullptr), voice_ptr(nullptr, nullptr)};

  std::unique_ptr<PaStreamParameters> outputParameters;

  // Create one global TTS instance, also see comment for Text2Speech::getInstance()
  std::unique_ptr<Text2Speech> tts;
};


/**
 * This class provides text2speech capabilities.
 */
class Text2Speech
{
public:
  Text2Speech(const Text2SpeechSetup& setup, Text2SpeechSetup::Voice voice);
  ~Text2Speech();

  // Currently, there is only one Text2Speech instance at all.
  // We need a RobotLocal<> class to have one instance per (simulated) robot.
  static Text2Speech& getInstance() { return *instance; };

  /**
   * NOTE: Do not call this directly, better use SystemCall::text2Speech(std::string text) for background processing.
   * The function converts the given text to speech and outputs the resulting audio stream.
	 */
  void text2Speech(std::string text);

private:
  static Text2Speech* instance;

  std::unique_ptr<Text2SpeechData> data;
};
