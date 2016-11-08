/**
* @file CoMProvider.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once
#include <list>
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "StepData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/ActualCoM.h"

/**
 * @class CoMProvider
 * Determines the target orientation of the body.
 */


class CoMProvider
{
public:
	/** Constructor with all needed source data structures.
	 * @param theSensorData Measured data.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
	CoMProvider(
		const JointAngles				&theJointAngles,
		const WalkingEngineParams	&theWalkingEngineParams,
		const JointRequest			&theJointRequest,
		const FootSteps				&theFootSteps,
    const RobotModel &theRobotModel,
		const ActualCoMRCS			&theActualCoMProvider);

	/** Destructor */
	~CoMProvider(void);

	/** 
	 * Calculates the next target orientation.
	 * @param bodyTilt Target data structure.
	 */
	void updateActualCoM(ActualCoM &theActualCoM);

private:
    //const JointAngles				&theJointAngles; unused
    //const WalkingEngineParams	&theWalkingEngineParams; unused
    //const JointRequest			&theJointRequest;			/**< Set by constructor. */ unused
		const FootSteps				&theFootSteps;
    const RobotModel &theRobotModel;
		const ActualCoMRCS			&theActualCoMRCS;

		typedef std::list<Footposition> FootList;

		/** List of target foot positions in world coordinate system filled with the
			foot positions found in theFootpositions. No more needed positions
			are deleted in every step cycle. */	
		FootList footPositions;
};

