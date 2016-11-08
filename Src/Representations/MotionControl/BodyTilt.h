/**
* @class BodyTilt 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"


class BodyTilt : public Streamable
{
public :

	// angle around x and y
	float x, y;

	void watch()
	{
		WATCH(x);
		WATCH(y);
	}

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN;
		STREAM(x)
		STREAM(y)
		STREAM_REGISTER_FINISH;
	};
	BodyTilt()
	{
		x = y = 0;
	};

	~BodyTilt(){};

	void operator =(Point &p)
	{
		this->x=(float)p.x;
		this->y=(float)p.y;
	}
  void operator =(BodyTilt &b)
  {
    this->x=(float)b.x;
    this->y=(float)b.y;
  }
};
