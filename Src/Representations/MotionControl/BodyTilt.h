/**
* @class BodyTilt 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"


STREAMABLE(BodyTilt,
  void watch()
  {
    WATCH(x);
    WATCH(y);
  }

  BodyTilt& operator=(const Point &p)
  {
    this->x=(float)p.x;
    this->y=(float)p.y;
    return *this;
  }
  BodyTilt& operator=(const BodyTilt &b)
  {
    this->x=(float)b.x;
    this->y=(float)b.y;
		return *this;
  }
	,
	(float)(0.f) x,
	(float)(0.f) y
);
