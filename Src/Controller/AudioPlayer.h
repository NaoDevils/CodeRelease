/**
 * @file AudioPlayer.cpp
 * This file declares a class that plays audio samples using PortAudio.
 * @author Aaron Larisch
 */

#pragma once

#include <memory>

struct AudioData;

class AudioPlayer
{
  struct Pimpl;
  std::unique_ptr<Pimpl> p;

public:
  AudioPlayer();
  ~AudioPlayer();
  void play(const AudioData& audioData);
};
