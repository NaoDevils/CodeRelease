/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/

/** 
* @file ZMPIPController.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "StepData.h"
#include "Point.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Tools/RingBuffer.h"
#include "Representations/Sensing/ZMPModel.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Math/Filter/FIRFilter.h"
#include "Tools/Module/Module.h"
#else
#include "Filter/FIRFilter.h"
#endif

#define MAX_SENSOR_DELAY 100

/**
 * @class ZMPIPController
 * Controller translation the desired ZMP position to a target CoM position. See
 *
 * Stefan Czarnetzki, Sören Kerner, Oliver Urbann, 
 * Observer-based dynamic walking control for biped robots, 
 * Robotics and Autonomous Systems, Volume 57, Issue 8, Humanoid Soccer Robots, 31 July 2009, Pages 839-845, ISSN 0921-8890
 * doi:10.1016/j.robot.2009.03.007. http://linkinghub.elsevier.com/retrieve/pii/S0921889009000608.
 * 
 */
class ZMPIPController 
{

public:

	/** Constructor with all needed source data structures.
	 * @param theRobotModel Model of the robot.
	 * @param theRefZMP The desired ZMP.
	 * @param theSensorData Data from sensors.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 * @param thePatternGenRequest The request received by the PatternGenerator.
	 * @param theFallDownState Information about the current state. If the robt has fallen down stop walking engine.
	 * @param theControllerParams Some controller parameters used here.
	 * @param theZMPModel The measured ZMP.
	 * @param theWalkingInfo Informations about the walk from the last frame.
	 */
	ZMPIPController(
		const RobotModel			&theRobotModel,
		const RefZMP				&theRefZMP,
		const InertialSensorData			&theInertialSensorData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const PatternGenRequest		&thePatternGenRequest,
		const FallDownState			&theFallDownState,
		const ControllerParams		&theControllerParams,
		const ZMPModel				&theZMPModel,
		const WalkingInfo			&theWalkingInfo);

	~ZMPIPController() { freeMem(); } /**< Destructor */

	/** Start the controller. */
	void Start() { useSensor(theWalkingEngineParams.sensorControlRatio[0]); }

	/** Tells the controller to stop moving after the last added step. */
	void End() { reset(); } 

	/** Use the measured ZMP */
	void setZMP();

	/** 
	 * Returns the current observation
	 * @param x Obersavtion vector for x axis (position, speed, ZMP)
	 * @param y Obersavtion vector for y axis (position, speed, ZMP)
	 */
	void getObservations(Vector3f &x, Vector3f &y);


	/** 
	 * Use the sensor with given factor.
	 * @param factor Scaling factor between 0 and 1.
	 */
	void useSensor(float factor) { sensorScale = factor; }
	
	/** 
	 * Calculate one target position of the CoM.
	 * @param targetCoM The representation to fill with the position.
	 */
	void updateKinematicRequest(TargetCoM & targetCoM);

private:
  //const RobotModel &          theRobotModel;			/**< Set by constructor. */ unused
	const RefZMP &				theRefZMP;				/**< Set by constructor. */
	const InertialSensorData &          theInertialSensorData;			/**< Set by constructor. */
	const WalkingEngineParams & theWalkingEngineParams;	/**< Set by constructor. */
	const PatternGenRequest	&	thePatternGenRequest;	/**< Set by constructor. */
  //const FallDownState	&		theFallDownState;		/**< Set by constructor. */ unused
	const ControllerParams &    theControllerParams;	/**< Set by constructor. */
	const ZMPModel &            theZMPModel;			/**< Set by constructor. */
	const WalkingInfo			&theWalkingInfo;		/**< Set by constructor. */


	typedef std::list<ZMP *> ZMPList;
	/** Current element in the list. */
	ZMPList::iterator kElement;
	/** List of reference ZMP. */
	ZMPList pRef;

	RingBuffer<Point, MAX_SENSOR_DELAY> delayBuffer, /**< Buffer to deal with the sensor delay. */
		positionDelayBuffer;	/**< Buffer to deal with the sensor delay. */
	float localSensorScale;

	float cur_x[3],		/**< Controller state vector for x axis. */
		cur_y[3],			/**< Controller state vector for y axis. */
		obs_x[3],			/**< Observer state vector for x axis. */
		obs_y[3];			/**< Observer state vector for y axis. */

	float cont_x,			/**< Controller internal value. */
		cont_y;				/**< Controller internal value. */

	float vx,				/**< Controller internal value. */
		vy;					/**< Controller internal value. */

	bool isRunning,			/**< Is the controller running? */
		kElementEmpty;		/**< Is there a valid current ZMPList element? */
	Point robotPosition,	/**< Current robot position. */
		realZMP;			/**< Current measured ZMP. */
	float sensorScale;		/**< Scaling factor for the measured ZMP error. */

	void reset();			/**< Resets the controller. */
	void Shrink();			/** Deletes no more needed elements in lists. */
	void addRefZMP(ZMP zmp);/** Add a ZMP position to the list. */
	void freeMem();			/** Free all memory. */
	Point controllerStep(); /** Calculate one step of the system. */


};
