/**
 * @file AudioData.h
 * The file declares a struct that stores audio data of up to four channels.

 * On a V6, the four channels are:
 * 0: back left microphone
 * 1: back right microphone
 * 2: front left microphone
 * 3: front right microphone

 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(AudioData,
  void draw() const;
  ,
  (unsigned)(0) channels, /** Number of channels */
  (unsigned)(0) sampleRate, /** Chosen sample rate */
  (std::string)("") device, /** Selected audio device name */
  (std::string)("") api, /** Selected audio device API */
  (double)(0.0) latency, /** Chosen suggested stream latency */
  (bool)(false) isValid, /** Record state */
  (std::vector<float>) samples /** Interleaved samples */
);
