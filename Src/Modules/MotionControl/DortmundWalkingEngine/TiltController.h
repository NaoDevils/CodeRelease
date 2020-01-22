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
* @file TiltController.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once


#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
//#include "Representations/MotionControl/SpeedInfo.h"
//#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
//#include "Representations/Configuration/JointCalibration.h"
//#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/PIDController.h"

/**
 * @class TiltController
 * Determines the target orientation of the body.
 */
class TiltController
{
public:
	/** Constructor with all needed source data structures.
	 * @param theInertialSensorData Measured data.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
	TiltController(
		const InertialSensorData			&theInertialSensorData,
		const WalkingEngineParams	&theWalkingEngineParams/*,
		const JointCalibration		&theJointCalibration,
		const JointRequest			&theJointRequest,
		const SpeedInfo				&theSpeedInfo,
		const TargetCoM				&theTargetCoM*/);

	/** Destructor */
	~TiltController(void);

	/** 
	 * Calculates the next target orientation.
	 * @param bodyTilt Target data structure.
	 */
	void updateBodyTilt(BodyTilt &bodyTilt);

private:
	const InertialSensorData  &theInertialSensorData;/**< Set by constructor. */
	const WalkingEngineParams	&theWalkingEngineParams;/**< Set by constructor. */
	//const JointCalibration		&theJointCalibration;	/**< Set by constructor. */
	//const JointRequest				&theJointRequest;			/**< Set by constructor. */
	//const SpeedInfo           &theSpeedInfo;        /**< Set by constructor. */
	//const TargetCoM				&theTargetCoM;
	Point angleSum; /**< This is the sum of the angle over max angle and under the min angle respectively */
  PIDController<float> tiltPIDController,rollPIDController;
  BodyTilt lastBodyTilt;
};

