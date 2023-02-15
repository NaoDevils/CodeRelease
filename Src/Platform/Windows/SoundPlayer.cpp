/**
* @file Platform/Windows/SoundPlayer.cpp
* Implementation of class SoundPlayer.
* @attention this is the Windows implementation
* @author Colin Graf
*/

#include <windows.h>

#include "SoundPlayer.h"
#include "Platform/File.h"
#include <Platform/Common/Text2Speech.h>

int SoundPlayer::play(const std::string& name)
{
  if (name.substr(0, 4).compare("t2s:") == 0)
  {
    Text2Speech::getInstance().text2Speech(name.substr(3));
    return 1;
  }
  else
  {
    std::string filePath(File::getBHDir());
    filePath += "/Config/Sounds/";
    filePath += name;
    PlaySound(filePath.c_str(), nullptr, SND_ASYNC | SND_FILENAME);
    return 1;
  }
}
