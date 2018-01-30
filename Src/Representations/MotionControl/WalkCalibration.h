/**
* @file WalkCalibration.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"


/**
* @class WalkCalibration 
* 
*/
struct WalkCalibration : public Streamable
{
  StreamPoint fixedOffset;
  bool calibrationDone;

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN;
    STREAM(fixedOffset)
    STREAM(calibrationDone)
		STREAM_REGISTER_FINISH;
	};

	/** Constructor */
  WalkCalibration()
	{
	};

	/** Desctructor */
	~WalkCalibration(){};
};
