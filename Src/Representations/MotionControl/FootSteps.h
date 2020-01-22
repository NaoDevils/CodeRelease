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
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot steps in buffer */
#define MAX_STEPS	300

/**
 * @class Robot
 * Representation to transfer the target foot steps to other modules.
 */
class FootSteps : public Streamable
{
public :

	/** When the controller is not running this step is used. */
	StepData suggestedStep;

	/** Is the controller running? */
	bool running;

  DWE::State walkState;

	/** Immediate stop in case of emergency. */
	bool emergencyStop;

  /** Position of the robot after the whole preview phase is executed */
  Point robotPoseAfterStep;

 /**
  * The provided time is the number of the frame when the currently created foot step
  * will be executed
  */
  unsigned time; 


	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN;
		STREAM(running)
    STREAM(time)
		STREAM_REGISTER_FINISH;
	};

	/** Constructor */
	FootSteps() : steps(MAX_STEPS)
	{
		running=false;
		emergencyStop=false;

	};

	/** Initialize the data. */
	void reset()
	{
		running=false;
		emergencyStop=false;
		steps.clear();
	}

	/** Destructor */
	~FootSteps(){};

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

	std::vector<Footposition> steps;
};
#endif
