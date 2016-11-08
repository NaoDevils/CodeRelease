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
* @file ZMPGenerator.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#include <list>
#include "StepData.h"

#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"


/**
 * @class ZMPGenerator
 * Calculates the reference ZMP using the given foot steps.
 */
class ZMPGenerator
{
public:
	
	/** Constructor with all needed source data structures.
	 * @param theFootSteps The target foot positions (positions on ground only).
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 * @param theControllerParams Some controller parameters used here.
	 */
	ZMPGenerator(
		const FootSteps				&theFootSteps,
		const WalkingEngineParams	&theWalkingEngineParams,
		const ControllerParams		&theControllerParams,
		const FreeLegPhaseParams	&theFreeLegPhaseParams,
		const WalkingInfo			&theWalkingInfo,
        const ReferenceModificator  &theReferenceModificator);
	/** Destructor */
	~ZMPGenerator(void);

	/** 
	 * Calculates on target position 
	 * @param refZMP Structure to be filled with the reference ZMP.
	 */
	void updateRefZMP(RefZMP& refZMP);

  Point plotZMP;
private:
	const FootSteps &           theFootSteps;			/**< Set by constructor. */
	const WalkingEngineParams & theWalkingEngineParams; /**< Set by constructor. */
	const ControllerParams &    theControllerParams;	/**< Set by constructor. */
	const FreeLegPhaseParams	&theFreeLegPhaseParams;
//	const WalkingInfo			&theWalkingInfo; unused
    const ReferenceModificator  &theReferenceModificator;

	typedef std::list<Footposition *> FootList;

	FootList footPositions;								/**< List of foot steps. */
	FootList::iterator currentFootpos, lastPlannedZMP;
	float lpxss;										/**< Last position of ZMP along the x axis */
  //FIXME: Value is used uninitialized
	bool isRunning;										/**< Is the ZMP/IP-Controller running? */
	ZMP zmp, lastZMPRCS;
	/** 
	 * Calculates on target position 
	 * @param refZMP Structure to be filled with the reference ZMP.
	 */
	void planZMP(RefZMP &refZMP);
	void addFootsteps(const Footposition &fp);			/**< Add a foot position to the footPositions list. */
	void reset();										/**< Resets the generator. */
	void Shrink();										/**< Deletes no more needed elements in list. */
	void freeMem();										/**< Free all memory. */
};

