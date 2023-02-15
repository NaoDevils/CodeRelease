/**
* @file TargetCoM 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif

/**
* @class TargetCoM 
* Representing the target position of center of mass.
*/
STREAMABLE_WITH_BASE(TargetCoM, TranslationPoint,

	void watch()
	{
		WATCH(x);
		WATCH(y);
		WATCH(z);
	}

	bool isRunning = false;

	/** Constructor */
	TargetCoM()
	{
    x = 0;
    y = 0;
    for (int i = 0; i < 6; i++) {
      state_x[i] = 0.0;
      state_y[i] = 0.0;
    }
    
	};
	,
	(double[6]) state_x,
	(double[6]) state_y
);

//struct TargetCoMFLIPM : public TargetCoM {};
//struct TargetCoMLIPM : public TargetCoM {};