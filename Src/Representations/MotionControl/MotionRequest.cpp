/**
 * @file Representations/MotionControl/MotionRequest.cpp
 * Implementation of a struct that represents the motions that can be requested from the robot.
 */

#include <cstdio>
#include <cstring>

#include "MotionRequest.h"
#include "Tools/Debugging/DebugDrawings.h"

void MotionRequest::printOut(char* destination) const
{
  strcpy(destination, getName(motion));
  destination += strlen(destination);
  switch(motion)
  {
    case walk:
      if(walkRequest.requestType == WalkRequest::destination)
        sprintf(destination, ": %.0lfmm %.0lfmm %.0lf°",
                walkRequest.request.translation.x(), walkRequest.request.translation.y(),
                walkRequest.request.rotation.toDegrees());
      else
        sprintf(destination, ": %.0lfmm/s %.0lfmm/s %.0lf°/s",
                walkRequest.request.translation.x(), walkRequest.request.translation.y(),
                walkRequest.request.rotation.toDegrees());
      break;
    case specialAction:
      sprintf(destination, ": %s", SpecialActionRequest::getName(specialActionRequest.specialAction));
      break;
    case kick:
      sprintf(destination, ": %s", KickRequest::getName(kickRequest.kickMotionType));
      break;
  }
}

void MotionRequest::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
  if(motion == walk)
  {
    switch(walkRequest.requestType)
    {
      case WalkRequest::destination:
      {
        LINE("representation:MotionRequest", 0, 0, walkRequest.request.translation.x(), walkRequest.request.translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        CROSS("representation:MotionRequest", walkRequest.request.translation.x(), walkRequest.request.translation.y(), 50, 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        Vector2f rotation(500.f, 0.f);
        rotation.rotate(walkRequest.request.rotation);
        ARROW("representation:MotionRequest", walkRequest.request.translation.x(), walkRequest.request.translation.y(), 
          walkRequest.request.translation.x() + rotation.x(), walkRequest.request.translation.y() + rotation.y(), 
          0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        break;
      }
      case WalkRequest::speed:
      {
        Vector2f translation = walkRequest.request.translation * 10.f; // TODO -> check
        ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        if(walkRequest.request.rotation != 0.0f)
        {
          translation.x() = translation.norm();
          translation.y() = 0;
          translation.rotate(walkRequest.request.rotation);
          ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        }
        break;
      }
    }
  }
}


void SpeedRequest::draw() const
{
  DEBUG_DRAWING("representation:SpeedRequest", "drawingOnField") // drawing of the speed vector
  {
    ARROW("representation:SpeedRequest", 0, 0, translation.x(), translation.y(), 5, Drawings::solidPen, ColorRGBA(200, 0, 0));
    DRAW_ROBOT_POSE("representation:SpeedRequest", (*this), ColorRGBA(0, 200, 0));
  }
}
