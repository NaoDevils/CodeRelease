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

#include "ZMPIPController.h"
#include "LimbCombinator.h"

//#define LOGGING
#ifndef WALKING_SIMULATOR
//#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

ZMPIPController::ZMPIPController(
		const RobotModel			&theRobotModel,
		const RefZMP				&theRefZMP,
		const InertialSensorData			&theInertialSensorData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const PatternGenRequest		&thePatternGenRequest,
		const FallDownState			&theFallDownState,
		const ControllerParams		&theControllerParams,
		const ZMPModel				&theZMPModel,
		const WalkingInfo			&theWalkingInfo):
  theRefZMP(theRefZMP),
  theInertialSensorData(theInertialSensorData),
  theWalkingEngineParams(theWalkingEngineParams),
  thePatternGenRequest(thePatternGenRequest),
  theControllerParams(theControllerParams),
  theZMPModel(theZMPModel),
  theWalkingInfo(theWalkingInfo)
{
	sensorScale=1.0f;
	reset();																				    
}

void ZMPIPController::setZMP()
{
	Point realZMP(theZMPModel.zmp_acc.x()/1000, theZMPModel.zmp_acc.y()/1000);
	Point CoM(0, 0, theControllerParams.z_h, 0);
	CoM.rotateAroundX(theInertialSensorData.angle.x()*theWalkingEngineParams.sensorControl.rollFactor);
	CoM.rotateAroundY(theInertialSensorData.angle.y()*theWalkingEngineParams.sensorControl.tiltFactor);

	positionDelayBuffer.push_front(robotPosition);

	realZMP+=CoM;
	if (positionDelayBuffer.size()>theWalkingEngineParams.sensorControl.sensorDelay)
	{
		realZMP.rotate2D(positionDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].r);
		realZMP+=positionDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay];
	}
	realZMP.z=0;
	this->realZMP=realZMP;
}


void ZMPIPController::freeMem()
{
	int size = static_cast<int>(pRef.size());
	for (int i=0; i<size; i++)
	{
		ZMP *zmp=pRef.front();
		pRef.pop_front();
		delete zmp;		
	}
}


void ZMPIPController::Shrink()
{
	while(pRef.front()!=(*kElement))
	{
		ZMP *zmp=pRef.front();
		pRef.pop_front();
		delete zmp;
	}
}

void ZMPIPController::reset()
{
	cur_x[0]=cur_y[0]=obs_x[0]=obs_y[0]=0;
	cur_x[1]=cur_y[1]=obs_x[1]=obs_y[1]=0;
	cur_x[2]=cur_y[2]=obs_x[2]=obs_y[2]=0;
	vx = vy = 0;

	cont_x=cont_y=0;
	freeMem();
	pRef.clear();

	kElementEmpty=true;

	isRunning=false;
	positionDelayBuffer.clear();
	delayBuffer.clear();
	realZMP=0;
}

Point ZMPIPController::controllerStep()
{
	static int delayCounter=0;

	Point target(cur_x[2], cur_y[2]);
	delayBuffer.push_front(target);

	Point ZMPdiff;
	if (delayBuffer.size()>theWalkingEngineParams.sensorControl.sensorDelay)
	{
		ZMPdiff.x=delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].x-realZMP.x;
		ZMPdiff.y=delayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].y-realZMP.y;
	}

	bool sensorOn=false;

	if (thePatternGenRequest.newState==PatternGenRequest::walking)
		sensorOn=true;

	if (sensorOn && localSensorScale<1)
		localSensorScale+=1.0f/theWalkingEngineParams.walkTransition.zmpSmoothPhase;

	if (!sensorOn)
		delayCounter++;
	else
		delayCounter=0;

	if (delayCounter>theControllerParams.N && localSensorScale>0)
		localSensorScale-=1.0f/theWalkingEngineParams.walkTransition.zmpSmoothPhase;

	float _obs_x[3], _obs_y[3], _cur_x[3], _cur_y[3];

	ZMPdiff.rotate2D(-robotPosition.r);
  ZMPdiff *= (localSensorScale * theWalkingEngineParams.sensorControl.sensorControlRatio[0]);
	ZMPdiff.rotate2D(robotPosition.r);

	_obs_x[0]=theControllerParams.A0(0,0)*obs_x[0]+theControllerParams.A0(0,1)*obs_x[1]+theControllerParams.A0(0,2)*obs_x[2]-theControllerParams.L(0,0)*ZMPdiff.x;
	_obs_x[1]=theControllerParams.A0(1, 0)*obs_x[0]+theControllerParams.A0(1, 1)*obs_x[1]+theControllerParams.A0(1, 2)*obs_x[2]-theControllerParams.L(1, 0)*ZMPdiff.x;
	_obs_x[2]=theControllerParams.A0(2,0)*obs_x[0]+theControllerParams.A0(2,1)*obs_x[1]+theControllerParams.A0(2,2)*obs_x[2]+theControllerParams.dt*cont_x-theControllerParams.L(2,0)*ZMPdiff.x;

	_obs_y[0]=theControllerParams.A0(0,0)*obs_y[0]+theControllerParams.A0(0,1)*obs_y[1]+theControllerParams.A0(0,2)*obs_y[2]-theControllerParams.L(0,0)*ZMPdiff.y;
	_obs_y[1]=theControllerParams.A0(1,0)*obs_y[0]+theControllerParams.A0(1,1)*obs_y[1]+theControllerParams.A0(1,2)*obs_y[2]-theControllerParams.L(1,0)*ZMPdiff.y;
	_obs_y[2]=theControllerParams.A0(2,0)*obs_y[0]+theControllerParams.A0(2,1)*obs_y[1]+theControllerParams.A0(2,2)*obs_y[2]+theControllerParams.dt*cont_y-theControllerParams.L(2,0)*ZMPdiff.y;

	obs_x[0]=_obs_x[0];
	obs_x[1]=_obs_x[1];
	obs_x[2]=_obs_x[2];

	obs_y[0]=_obs_y[0];
	obs_y[1]=_obs_y[1];
	obs_y[2]=_obs_y[2];

	vx += obs_x[2] - (*kElement)->x();
	vy += obs_y[2] - (*kElement)->y();

	kElement++;

	Point u;
	ZMPList::iterator _pRef;
	_pRef=kElement;

	for(int j=0; j<theControllerParams.N; j++)
	{
		ASSERT(_pRef!=pRef.end());
		u.x+=theControllerParams.Gd[j]*(*_pRef)->x();
		u.y+=theControllerParams.Gd[j]*(*_pRef)->y();	
		++_pRef;
	}

	cont_x = -theControllerParams.Gi*vx - (theControllerParams.Gx(0,0) * obs_x[0] + theControllerParams.Gx(0,1) * obs_x[1] + theControllerParams.Gx(0, 2) * obs_x[2]) - u.x;
	cont_y = -theControllerParams.Gi*vy - (theControllerParams.Gx(0, 0) * obs_y[0] + theControllerParams.Gx(0, 1) * obs_y[1] + theControllerParams.Gx(0, 2) * obs_y[2]) - u.y;

	_cur_x[0]=theControllerParams.A0(0, 0)*obs_x[0]+theControllerParams.A0(0, 1)*obs_x[1]+theControllerParams.A0(0, 2)*obs_x[2];
	_cur_x[1]=theControllerParams.A0(1, 0)*obs_x[0]+theControllerParams.A0(1, 1)*obs_x[1]+theControllerParams.A0(1, 2)*obs_x[2];
	_cur_x[2]=theControllerParams.A0(2, 0)*obs_x[0]+theControllerParams.A0(2, 1)*obs_x[1]+theControllerParams.A0(2, 2)*obs_x[2]+theControllerParams.dt*cont_x;

	_cur_y[0]=theControllerParams.A0(0, 0)*obs_y[0]+theControllerParams.A0(0, 1)*obs_y[1]+theControllerParams.A0(0, 2)*obs_y[2];
	_cur_y[1]=theControllerParams.A0(1, 0)*obs_y[0]+theControllerParams.A0(1, 1)*obs_y[1]+theControllerParams.A0(1, 2)*obs_y[2];
	_cur_y[2]=theControllerParams.A0(2, 0)*obs_y[0]+theControllerParams.A0(2, 1)*obs_y[1]+theControllerParams.A0(2, 2)*obs_y[2]+theControllerParams.dt*cont_y;

	cur_x[0]=_cur_x[0];
	cur_x[1]=_cur_x[1];
	cur_x[2]=_cur_x[2];

	cur_y[0]=_cur_y[0];
	cur_y[1]=_cur_y[1];
	cur_y[2]=_cur_y[2];

	return Point(cur_x[0], cur_y[0], theControllerParams.z_h);
}


void ZMPIPController::addRefZMP(ZMP zmp)
{
	pRef.push_back(new ZMP(zmp));

	if (kElementEmpty)
	{
		kElement=pRef.begin();
		kElementEmpty=false;
	}
}

void ZMPIPController::getObservations(Vector3f &x, Vector3f &y)
{
	x[0]=obs_x[0];
	x[1]=obs_x[1];
	x[2]=obs_x[2];

	y[0]=obs_y[0];
	y[1]=obs_y[1];
	y[2]=obs_y[2];
}


void ZMPIPController::updateKinematicRequest(TargetCoM & targetCoM)
{
	for (int i=0; i<theRefZMP.numOfZMP; i++)
		addRefZMP(theRefZMP.getZMP(i));
	targetCoM.x=targetCoM.y=0;
	
	robotPosition.x=theWalkingInfo.robotPosition.translation.x();
	robotPosition.y=theWalkingInfo.robotPosition.translation.y();
	robotPosition.r=theWalkingInfo.robotPosition.rotation;

	if (!isRunning && theRefZMP.running)
	{
		Start();
		isRunning=true;
	}
	setZMP();
	if (isRunning && !theRefZMP.running)
	{
		End();
		isRunning=false;
	}
	if (isRunning)
	{
		ASSERT(pRef.size()>0);
		(Point &)targetCoM=controllerStep();
		Shrink();
		/*LOG("WalkingEngine", "real ZMP x", realZMP.x);
		LOG("WalkingEngine", "real ZMP y", realZMP.y);
		LOG("WalkingEngine", "calc ZMP x", cur_x[2]);
		LOG("WalkingEngine", "calc ZMP y", cur_y[2]);
		LOG("WalkingEngine", "ref ZMP x", (*kElement)->x);
		LOG("WalkingEngine", "ref ZMP y", (*kElement)->y);
		LOG("WalkingEngine", "Acc x", theSensorData.data[SensorData::accX]);
		LOG("WalkingEngine", "Acc y", theSensorData.data[SensorData::accY]);
		LOG("WalkingEngine", "Acc z", theSensorData.data[SensorData::accZ]);
		LOG("WalkingEngine", "Roll", theSensorData.data[SensorData::angleX]);
		LOG("WalkingEngine", "Tilt", theSensorData.data[SensorData::angleY]);
		LOG("WalkingEngine", "Target CoM x", targetCoM.x);
		LOG("WalkingEngine", "Target CoM y", targetCoM.y);
		LOG("WalkingEngine", "Target CoM z", targetCoM.z);
		LOG("WalkingEngine", "Walking Engine Time", LimbCombinator::walkingEngineTime);*/
	}
}
