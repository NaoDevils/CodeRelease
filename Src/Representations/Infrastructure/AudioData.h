/**
 * @file AudioData.h
 * The file declares a struct that stores audio data of up to four channels.
 * On a V4, the four channels are:
 * 0: left microphone
 * 1: right microphone
 * 2: front microphone
 * 3: rear microphone
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

// deprecated!
//STREAMABLE(AudioData,
//{,
//  (unsigned)(2) channels,
//  (unsigned)(48000) sampleRate,
//  (std::vector<short>) samples, /**< Samples are interleaved. */
//});

STREAMABLE(AudioData,
{ ,
  (unsigned)(0) channels, /**< will be overwritten by AudioProviderDortmund*/
  (unsigned)(0) sampleRate, /**< will be overwritten by AudioProviderDortmund*/
  (std::vector<float>) samples, /**< Samples are interleaved. */
  (bool)(false) isValid, /** set by AudioProviderDortmund to indicate the record state*/
});
