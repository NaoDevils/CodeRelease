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
  DWE::State currentState;

	void serialize(In* in,Out* out)
	{
    Vector3f leftFootPos(this->footPos[0].x, this->footPos[0].y, this->footPos[0].z);
    Vector3f leftFootRot(this->footPos[0].rx, this->footPos[0].ry, this->footPos[0].r);
    Vector3f rightFootPos(this->footPos[1].x, this->footPos[1].y, this->footPos[1].z);
    Vector3f rightFootRot(this->footPos[1].rx, this->footPos[1].ry, this->footPos[1].r);
    STREAM_REGISTER_BEGIN;
    STREAM(leftFootPos)
    STREAM(leftFootRot)
    STREAM(rightFootPos)
    STREAM(rightFootRot)
    STREAM(running)
    STREAM(currentState, DWE)
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
