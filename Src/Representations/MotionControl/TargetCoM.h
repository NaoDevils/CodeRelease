/**
* @file TargetCoM 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif

/**
* @class TargetCoM 
* Representing the target position of center of mass.
*/
class TargetCoM : public Streamable, public Point, public Watch
{
public :
	void watch()
	{
		WATCH(x);
		WATCH(y);
		WATCH(z);
	}

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN;
		STREAM(x)
		STREAM(y)
		STREAM(z)
    STREAM(state_x)
   STREAM(state_y)
		STREAM_REGISTER_FINISH;
	};

    float state_x[6], state_y[6];
    bool isRunning;

	/** Constructor */
	TargetCoM()
	{
	};

	/** Desctructor */
	~TargetCoM(){};
};