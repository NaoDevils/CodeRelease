/**
* @file Footpositions.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class Robot
 * Representation to transfer the target foot (including the swing leg) positions to other modules.
 */
STREAMABLE_WITH_BASE(Footpositions, Footposition,
  Footpositions& operator=(const StepData &p)
	{
    this->Footpositions::StepData::operator=(p);
		return *this;
	}

	Footpositions& operator=(const Footposition &p)
	{
    this->Footposition::operator=(p);
		return *this;
	}
	,
  /** Is the controller running? */
	(bool)(false) running,
	((DWE) State)(standby) currentState
);
