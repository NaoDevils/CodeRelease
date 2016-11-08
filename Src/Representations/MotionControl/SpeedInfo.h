/**
* @file SpeedInfo 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Representations/MotionControl/WalkRequest.h"
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class SpeedInfo
 * Contains information about the actual speed.
 */
struct SpeedInfo : public Streamable
{
	Pose2f speed; /**< The< actual speed. */
  Pose2f speedBeforePreview; /**< Future speed in ControllerParams::N frames */
  bool deceleratedByAcc; /** Decelerated due to limited acceleration */
	int timestamp; /** Timestamp of the foot executing this speed */
  
  /** Currently executed custom step file. If this is not "none" the speed above
   is invalid. */
  WalkRequest::StepRequest currentCustomStep;
  
  int delay; /**< The current speed was requested delay frames before. */
	
	/** Constructor */
	SpeedInfo(){}

	/** Destructor */
	~SpeedInfo(){}

protected:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_REGISTER_FINISH;
  }
};
