/**
* @file RobotMap.h
* Declaration of a class that represents the map of tracked robots in this robot's environment.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#pragma once
#include "Representations/Perception/RobotsPercept.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include <vector>

/**
* @class RobotMap
* A class represents the map of tracked robots in this robot's environment.
*/

STREAMABLE(RobotMapEntry,
{,
  (Pose2f) pose, /**< The position in absolute field coordinates. */
  ((RobotEstimate) RobotType) robotType,
});

STREAMABLE(RobotMap,
{
  RobotMap() {robots.reserve(10); }

  /** Reset the RobotMap */
  void reset() {robots.clear();}

  RobotMap& operator=(const RobotMap &other)
  {
    if (this == &other)
      return *this;
    reset();
    for (unsigned int i = 0; i < other.robots.size(); i++)
    {
      RobotMapEntry re;
      re.robotType = other.robots[i].robotType;
      re.pose = other.robots[i].pose;
      robots.push_back(re);
    }
    return *this;
  }

  void draw() const,
  (std::vector<RobotMapEntry>) robots, /**< The robots in absolute field coordinates. */

});


struct GroundTruthRobotMap : public RobotMap 
{
  /** Draws something*/
  void draw() const;
};


struct LocalRobotMap : public RobotMap 
{
  void draw() const;
};


struct RemoteRobotMap : public RobotMap 
{
  /** Draws something*/
  void draw() const;
};

STREAMABLE(RobotMapCompressed,
{
  RobotMapCompressed() = default;
  RobotMapCompressed(const RobotMap &other)
  {
    robots.clear();
    for (unsigned int i = 0; i < other.robots.size(); i++)
    {
      robots.push_back(RobotMapEntryCompressed((short)other.robots[i].pose.translation.x(),
        (short)other.robots[i].pose.translation.y(),other.robots[i].robotType));
    }
  }

  operator RobotMap() const
  {
    RobotMap robotMap;
    robotMap.robots.clear();
    for (unsigned int i = 0; i < robots.size(); i++)
    {
      RobotMapEntry re;
      re.robotType = robots[i].robotType;
      re.pose.translation.x() = (float)robots[i].pose.x();
      re.pose.translation.y() = (float)robots[i].pose.y();
      re.pose.rotation = 0;
      robotMap.robots.push_back(re);
    }
    return robotMap;
  }

  STREAMABLE(RobotMapEntryCompressed,
  {
    RobotMapEntryCompressed() = default;
    RobotMapEntryCompressed(short x, short y, RobotEstimate::RobotType type)
    {
      pose.x() = x;
      pose.y() = y;
      robotType = type;
    },
    (Vector2s) pose,
    ((RobotEstimate) RobotType) robotType,
  }),

  (std::vector<RobotMapEntryCompressed>) robots,

});
