/**
* @file RobotMap.cpp
* Implements drawing methods.
*/

#include "RobotMap.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose3f.h"

void RobotMap::draw() const
{
  DEBUG_DRAWING("representation:RobotMap", "drawingOnField")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      CIRCLE("representation:RobotMap", i->pose.translation.x(), i->pose.translation.y(), 100, 10,
        Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
    }
  }

  // now in 3D
  DEBUG_DRAWING3D("representation:RobotMap", "field")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      float x = (float)i->pose.translation.x();
      float y = (float)i->pose.translation.y();
      CIRCLE3D("representation:RobotMap", Pose3f(x, y, 1),
        200,
        5,
        color);
    }
  }
}

void GroundTruthRobotMap::draw() const
{
  DEBUG_DRAWING("representation:GroundTruthRobotMap", "drawingOnField")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      CIRCLE("representation:GroundTruthRobotMap", i->pose.translation.x(), i->pose.translation.y(), 100, 10,
        Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
    }
  }

  // now in 3D
  DEBUG_DRAWING3D("representation:GroundTruthRobotMap", "field")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      float x = (float)i->pose.translation.x();
      float y = (float)i->pose.translation.y();
      CIRCLE3D("representation:GroundTruthRobotMap", Pose3f(x, y, 1),
        200,
        5,
        color);
    }
  }
}

void LocalRobotMap::draw() const
{
  DEBUG_DRAWING("representation:LocalRobotMap", "drawingOnField")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      CIRCLE("representation:LocalRobotMap", i->pose.translation.x(), i->pose.translation.y(), 100, 10,
        Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
    }
  }

  // now in 3D
  DEBUG_DRAWING3D("representation:LocalRobotMap", "field")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      float x = (float)i->pose.translation.x();
      float y = (float)i->pose.translation.y();
      CIRCLE3D("representation:LocalRobotMap", Pose3f(x, y, 1),
        200,
        5,
        color);
    }
  }
}

void RemoteRobotMap::draw() const
{
  DEBUG_DRAWING("representation:RemoteRobotMap", "drawingOnField")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      CIRCLE("representation:RemoteRobotMap", i->pose.translation.x(), i->pose.translation.y(), 100, 10,
        Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
    }
  }

  // now in 3D
  DEBUG_DRAWING3D("representation:RemoteRobotMap", "field")
  {
    for (std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
    {
      ColorRGBA color = (i->robotType == RobotEstimate::teammateRobot) ? ColorRGBA(150, 255, 0) : ColorRGBA::black;
      color = (i->robotType == RobotEstimate::opponentRobot) ? ColorRGBA::red : color;
      float x = (float)i->pose.translation.x();
      float y = (float)i->pose.translation.y();
      CIRCLE3D("representation:RemoteRobotMap", Pose3f(x, y, 1),
        200,
        5,
        color);
    }
  }
}