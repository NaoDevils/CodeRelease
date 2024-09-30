/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Settings.h"
#include "Tools/Streams/InStreams.h"
#ifdef TARGET_ROBOT
#include "Platform/Linux/NaoBodyV6.h"
#include <iostream>
#include <chrono>
#include <thread>
#endif

#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Streams/AutoStreamable.h"
#include <stdexcept>

STREAMABLE(Robots,,
  (std::vector<RobotConfig>) robotsIds
);

void Settings::read(In& in)
{
  unsigned char version;
  in >> version;
  if (version != this->version)
    throw std::runtime_error("unexpected settings version");

  unsigned char overlaysSize;
  in >> robotName >> bodyName >> naoVersion >> overlaysSize;

  overlays.resize(overlaysSize);
  for (std::string& overlay : overlays)
    in >> overlay;
}

void Settings::write(Out& out) const
{
  out << version << robotName << bodyName << naoVersion << static_cast<unsigned char>(overlays.size());
  for (const std::string& overlay : overlays)
    out << overlay;
}

bool Settings::load()
{
  std::string bhdir = File::getBHDir();
#ifdef TARGET_ROBOT
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if (!robotsStream.exists())
  {
    TRACE("Could not load robots.cfg");
    return false;
  }
  else
  {
    Robots robots;
    robotsStream >> robots;

    std::string bodyId, headId;

    // wait for LoLA
    NaoBodyV6 naoBody;
    while (!naoBody.init())
    {
      std::cerr << "Waiting for LoLA via ndevilsbase...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (*NaoBodyV6().getBodyId() == 0)
    {
      std::cerr << "Waiting for body id...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    bodyId = std::string(NaoBodyV6().getBodyId());

    while (*NaoBodyV6().getHeadId() == 0)
    {
      std::cerr << "Waiting for head id...\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    headId = std::string(NaoBodyV6().getHeadId());

    for (const RobotConfig& robot : robots.robotsIds)
    {
      if (robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        naoVersion = robot.naoVersion;
      }
      if (robot.headId == headId)
      {
        robotName = robot.name;
      }
    }

    if (bodyName.empty())
    {
      bodyName = bodyId;
      TRACE("Could not find bodyId in robots.cfg.");
    }

    if (robotName.empty())
    {
      robotName = headId;
      TRACE("Could not find headId in robots.cfg.");
    }
  }
#endif

  // the default config hierarchy does not work here because
  // the global settings object is not initialized at this point
  const std::list<std::string> names = {bhdir + "/Config/settings.cfg", bhdir + "/Config/Robots/" + robotName + "/Head/settings.cfg", bhdir + "/Config/Robots/" + bodyName + "/Body/settings.cfg"};
  InMapFile stream(names);
  if (stream.exists())
    stream >> *this;
  else
  {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robotName.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  std::cout << "Hi, I am " << robotName;
  if (robotName == bodyName)
    std::cout << ".\n";
  else
    std::cout << " (using body of " << bodyName << ").\n";

  std::cout << "overlays:";
  for (const std::string& overlay : overlays)
    std::cout << " " << overlay;
  std::cout << "\n";
#endif

  return true;
}
