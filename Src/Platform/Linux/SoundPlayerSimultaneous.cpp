/**
* @file  Platform/linux/SoundPlayerSimultaneous.cpp
* Implementation of class SoundPlayerSimulataneous.
* @attention this is the Linux implementation
*/

#include <sys/wait.h>

#include "SoundPlayerSimultaneous.h"
#include "Platform/File.h"

#include <algorithm>
#include <cstring>
#include <thread>
#include <vector>

SoundPlayerSimultaneous SoundPlayerSimultaneous::soundPlayer;

SoundPlayerSimultaneous::SoundPlayerSimultaneous() :
  Thread<SoundPlayerSimultaneous>()
{}

SoundPlayerSimultaneous::~SoundPlayerSimultaneous()
{}

/*
void SoundPlayer::start()
{
  Thread<SoundPlayer>::start(this, &SoundPlayer::main);
}

void SoundPlayer::main()
{
  while(isRunning() && !closing)
  {
    flush();
    VERIFY(sem.wait());
  }
}

void SoundPlayer::playDirect(const std::string& basename)
{
  std::string fileName(filePrefix);
  fileName += basename;

  int r = vfork();
  if(r == -1)
    perror("SoundPlayer: fork() failed");
  else if(r != 0) //parent
  {
    int status;
    waitpid(r, &status, 0);
  }
  else //child
  {
    if(execlp("aplay", "aplay", "-q", fileName.c_str(), (char*)0) == -1)
      perror("SoundPlayer: exec failed");
  }
}

void SoundPlayer::flush()
{
  for(;;)
  {
    std::string first;
    {
      SYNC;
      if(0 == queue.size())
        break;
      first = queue.front();
      queue.pop_front();
    }

    playDirect(first);
  }
}
int SoundPlayer::play(const std::string& name)
{
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
    queuelen = static_cast<int>(soundPlayer.queue.size());
    if(!soundPlayer.started)
    {
      soundPlayer.started = true;
      soundPlayer.filePrefix = File::getBHDir();
      soundPlayer.filePrefix += "/Config/Sounds/";
      soundPlayer.start();
    }
    else
      soundPlayer.sem.post();
  }
  return queuelen;
}
*/

static std::vector<std::string> currentSounds;

void playSoundFile(std::string fileName) {
	//if requested sound is not being played at the moment:
	if (std::find(currentSounds.begin(), currentSounds.end(), fileName) == currentSounds.end()) {
		#ifdef TARGET_ROBOT
		  fprintf(stderr, "Playing %s\n", fileName.c_str());
		#endif
		currentSounds.push_back(fileName);
		std::string cmd = "aplay -q ";
		std::string dir = File::getBHDir();
		dir.append("/Config/Sounds/");
		cmd.append(dir);
		cmd.append(fileName);
		char * cstr = new char[cmd.length() + 1];
		std::strcpy(cstr, cmd.c_str());
		std::system(cstr);
		currentSounds.erase(std::remove(currentSounds.begin(), currentSounds.end(), fileName));
	}
	else {
		#ifdef TARGET_ROBOT
		  fprintf(stderr, "%s is already being played\n", fileName.c_str());
		#endif
	}
}

int SoundPlayerSimultaneous::play(std::string name) {
    std::thread soundThread(playSoundFile, name);
    soundThread.detach();
	return 0;
}
