/**
* @file WalkingInformations.h
* This file contains some structures used by the walking engine to describe the walk.
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#define MAX_PARAM_FILENAME_LEN 1000
#include "Point.h"
#include <stdio.h>
#include <iostream>
#include <cstring>
#include "Tools/Enum.h"

#define LEFT_FOOT		0
#define RIGHT_FOOT		1
#define ROBOT_POSE		2

struct WalkingParams
{
	unsigned int 
		crouchingDownPhaseLen, /**< Duration for the crouching down phase */
		stoppingPhaseLen, /**< Duration for the stopping phase */
		startingPhaseLen; /**< Duration for the starting phase */

	double
		doubleSupportRatio; /**< The double support ration */

};

/** Current state of PatternGenerator. */
enum State
{
	standby,		/**< Standing, no ZMP/IP-Controller needed. */
	ready,			/**< Standing, ZMP/IP-Controller running and ready to go. */
	walking,		/**< Walking using ZMP/IP-Controller. */
  customSteps,/**< Executing step from a step file. */
  stopping,		/**< Stopping but still walking. */
  goingToReady,	/**< Going to standing with ZMP/IP-Controller. */
	goingToStandby, /**< Going to standing without ZMP/IP-Controller. */
	NA				/**< Unknown state. */
};

/** Current walking phase. */
enum WalkingPhase
{
	firstSingleSupport,		/**< Standing on left foot. */
	firstDoubleSupport,		/**< Phase after firstSingleSupport. */
	secondSingleSupport,	/**< Standing on right foot. */
	secondDoubleSupport,	/**< Phase after secondSingleSupport. */
	unlimitedDoubleSupport, /**< Double support with unknown end. */
	unlimitedSingleSupport, /**< Single support with unknown end. */
        unlimitedSingleSupportLeft,
	unlimitedSingleSupportRight,
	numOfWalkingPhases
};

inline bool isDS(WalkingPhase ph)
{
  return ph == firstDoubleSupport || ph == secondDoubleSupport;
}

inline bool isSS(WalkingPhase ph)
{
  return ph == firstSingleSupport || ph == secondSingleSupport;
}

enum FreeLegPhase { starting, ongoing, ending, freeLegNA, numOfFreeLegPhases };

class MovementInformation
{
public:
	Pose2f speed; /**< The new speed (x, y translation and rotation) */
	/**
	 * Equality check.
	 * @param other The other MovementInformation.
	 * @return true, if the instances are equal, false otherwise.
	 */
	bool operator != (MovementInformation other)
	{
		return speed!=other.speed;
	}
  
  int timestamp; /**< The time when this was requested. */
};

class RobotInformation
{
public:
	/** destructor */
	RobotInformation()
	{
		std::strcpy(paramFile, "parameters.dat");
	}

	/** 
	* Sets the path to the parameter file
	* @param path Path to the file
	* @return false, if failed
	*/
	bool setParamFile(char *path)
	{
		strncpy(paramFile, path, MAX_PARAM_FILENAME_LEN);
		return true;
	}

	char paramFile[MAX_PARAM_FILENAME_LEN]; /**< The path to the parameter file */
};


