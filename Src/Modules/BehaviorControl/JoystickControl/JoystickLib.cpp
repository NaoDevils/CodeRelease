// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>
#include "JoystickLib.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>


// The joystick connection only compiles on linux, thus on windows some methods has to be modified.
#ifdef LINUX
#include "unistd.h"
#endif

JoystickLib::JoystickLib()
{
}

JoystickLib::JoystickLib(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

JoystickLib::JoystickLib(std::string devicePath)
{
  openPath(devicePath);
}

bool JoystickLib::connect()
{
  // Close connection if it is already established.
  if (isFound())
    closeJoystick();

  // Open connection to the first joystick
  openPath("/dev/input/js0");

  return isFound();
}

void JoystickLib::openPath(std::string devicePath)
{
#ifdef LINUX
  std::cout << "Open joystick connection: " << devicePath << " ... ";
  m_fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
  // Check if connection is established.
  if (!isFound())
  {
    // Conecting to joystick failed.
    std::cout << "Failed" << std::endl;
    return;
  }
  std::cout << "Succeeded" << std::endl;
  // Empty message buffer.
  JoystickEvent event;
  while (sample(&event));
#endif
}

void JoystickLib::closeJoystick()
{
#ifdef LINUX
  close(m_fd);
  std::cout << "Closed joystick connection." << std::endl;
#endif
}

bool JoystickLib::sample(JoystickEvent* event)
{
#ifdef LINUX
  int bytes = read(m_fd, event, sizeof(*event));

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
#else
  return false;
#endif
}

bool JoystickLib::isFound()
{
#ifdef LINUX
  return m_fd >= 0;
#else
  return false;
#endif
}

JoystickLib::~JoystickLib()
{
  closeJoystick();
}
