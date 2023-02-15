/**
* @class FootSteps 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#ifndef _FOOTSTEPS_H
#define _FOOTSTEPS_H

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include <vector>
#include "Platform/BHAssert.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot steps in buffer */
#define MAX_STEPS 300

/**
 * @class Robot
 * Representation to transfer the target foot steps to other modules.
 */
STREAMABLE(FootSteps,
	/** When the controller is not running this step is used. */
	StepData suggestedStep;

  DWE::State walkState;

	/** Immediate stop in case of emergency. */
	bool emergencyStop = false;

  /** Position of the robot after the whole preview phase is executed */
  Point robotPoseAfterStep;

	/** Initialize the data. */
	void reset()
	{
		running=false;
		emergencyStop=false;
		steps.clear();
	}

	int getNumOfSteps() const
	{
		return static_cast<int>(steps.size());
	}

  bool empty() const
  {
    return steps.empty();
  }

	void addStep(Footposition newstep)
	{
		steps.push_back(newstep);
	}

	Footposition getStep(unsigned int i) const
	{
    ASSERT(!steps.empty());
		return steps[i];
	}

  void popStep()
  {
    steps.pop_back();
  }

  void popFront()
  {
    steps.erase(steps.begin());
  }

	/** 
	 *	Buffer for target foot steps. There might be more than one step
	 *	per frame to fill the preview buffer needed by the preview controller
	 *	ZMP/IP-Controller 
	 */

	std::vector<Footposition> steps{ MAX_STEPS };
	,

	/** Is the controller running? */
	(bool)(false) running,

	/**
	 * The provided time is the number of the frame when the currently created foot step
	 * will be executed
	 */
	(unsigned)(0) time
);
#endif
