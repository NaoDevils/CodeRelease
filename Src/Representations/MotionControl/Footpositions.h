/**
* @file Footpositions.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class Robot
 * Representation to transfer the target foot (including the swing leg) positions to other modules.
 */
class Footpositions : public Footposition, public Streamable
{
public :

	/** Is the controller running? */
	bool running;

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN;
		STREAM_REGISTER_FINISH;
	};

	/** Constructor */
	Footpositions()
	{
	};

	/** Desctructor */
	~Footpositions(){};

  void operator = (const StepData &p)
	{
    this->Footpositions::StepData::operator =(p);
	}

  void operator = (const Footposition &p)
	{
    this->Footposition::operator =(p);
	}

};
