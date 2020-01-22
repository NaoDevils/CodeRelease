/**
* @file  Platform/linux/SoundPlayerSimultaneous.h
*
* Declaration of class SoundPlayerSimultaneous.
*/

#pragma once

#include <string>
#include "Thread.h"

class SoundPlayerSimultaneous : Thread<SoundPlayerSimultaneous>
{
public:
	/**
	* If you want to play Config/Sounds/bla.wav use play("bla.wav");
	* Several different sounds can be played simultaneously but not the same ones.
	* @param name The filename of the sound file.
	* @return The amound of files in play sound queue.
	*/
	static int play(const std::string name);

private:
	static SoundPlayerSimultaneous soundPlayer; /**< The only instance of this class. */

	/**
	* Constructor.
	*/
	SoundPlayerSimultaneous();

	/**
	* Destructor.
	*/
	~SoundPlayerSimultaneous();
};
