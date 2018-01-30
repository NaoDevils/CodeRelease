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
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.cpp
* Generator for foot steps
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "PatternGenerator.h"
#include <iostream>

//#define LOGGING

#ifndef WALKING_SIMULATOR
#include "Tools/Math/Eigen.h"

#include "Tools/Debugging/Modify.h"

//#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

using namespace std;


PatternGenerator::PatternGenerator( 
                                   const WalkingEngineParams   &theWalkingEngineParams,
                                   const PatternGenRequest     &thePatternGenRequest,
                                   const RobotModel            &theRobotModel,
                                   const RobotDimensions       &theRobotDimensions,
                                   const FallDownState         &theFallDownState,
                                   const ControllerParams      &theControllerParams,
                                   const MotionSelection       &theMotionSelection,
                                   const WalkingInfo           &theWalkingInfo,
                                  
                                   const ReferenceModificator  &theReferenceModificator,
                                   const FrameInfo             &theFrameInfo,
                                   const BallModel             &theBallModel,
                                   const BallModelAfterPreview &theBallModelAfterPreview):
theWalkingEngineParams(theWalkingEngineParams),
thePatternGenRequest(thePatternGenRequest),
theRobotModel(theRobotModel),
theRobotDimensions(theRobotDimensions),
theFallDownState(theFallDownState),
theControllerParams(theControllerParams),
theMotionSelection(theMotionSelection),
theWalkingInfo(theWalkingInfo),
theReferenceModificator(theReferenceModificator),
theFrameInfo(theFrameInfo),
theBallModel(theBallModel),
theBallModelAfterPreview(theBallModelAfterPreview),
speedBuffer(Pose2f())
{
  wasReady = false;
  reset();
  baseRot=0; 
  curPitch=targetPitch=0;
}

void PatternGenerator::setPitch(float pitch)
{
  targetPitch=pitch;
}

float PatternGenerator::getCurrentPitch()
{
  return curPitch;
}

float PatternGenerator::getStepLength(float speed)
{
  return speed*curStepDuration/2;
}

int PatternGenerator::getOptimalPreviewLength()
{
  return std::max((unsigned int)theControllerParams.N+1, doubleSupportDuration()+singleSupportDuration()+1);
}

void PatternGenerator::setStepLength()
{

  Point forward, sidestep;
  float r=0;

  forward.x=(currentMovement.speed.translation.x()*curStepDuration/2) * cos(robotPose.r);
  forward.y=(currentMovement.speed.translation.x()*curStepDuration/2) * sin(robotPose.r);

  sidestep.x=-(currentMovement.speed.translation.y()*curStepDuration) * sin(robotPose.r);
  sidestep.y=(currentMovement.speed.translation.y()*curStepDuration) * cos(robotPose.r);

  r=currentMovement.speed.rotation*curStepDuration;

  deltaDirection[firstSingleSupport]=deltaDirection[secondSingleSupport]=r/(2*singleSupportDuration());
  deltaDirection[firstDoubleSupport]=deltaDirection[secondDoubleSupport]=0;

  float z=0.02f;

  Point p1(0,0,0,0);
  Point p2(0, 0, z, 0);

  footModifier[unlimitedDoubleSupport][RIGHT_FOOT]=p1;
  footModifier[unlimitedDoubleSupport][LEFT_FOOT]=p1;

  footModifier[firstSingleSupport][LEFT_FOOT] =p1;
  footModifier[firstSingleSupport][RIGHT_FOOT]=p2;
  footModifier[secondDoubleSupport][LEFT_FOOT]=p1;

  footModifier[firstDoubleSupport][LEFT_FOOT]=p1;
  footModifier[firstDoubleSupport][RIGHT_FOOT].x=forward.x;
  footModifier[firstDoubleSupport][RIGHT_FOOT].y=forward.y;
  footModifier[firstSingleSupport][RIGHT_FOOT]=footModifier[firstDoubleSupport][RIGHT_FOOT]+p2;


  footModifier[secondSingleSupport][RIGHT_FOOT]=p1;
  footModifier[secondSingleSupport][LEFT_FOOT]=p2;

  footModifier[secondDoubleSupport][LEFT_FOOT].x+=forward.x;
  footModifier[secondDoubleSupport][LEFT_FOOT].y+=forward.y;
  footModifier[secondDoubleSupport][LEFT_FOOT].z=0;
  footModifier[secondDoubleSupport][RIGHT_FOOT]=p1; 
  footModifier[secondSingleSupport][LEFT_FOOT]=footModifier[secondDoubleSupport][LEFT_FOOT]+p2;

  if (currentMovement.speed.translation.y()<0)
  {
    Point p1(forward.x, forward.y, 0, 0);
    Point p2(forward.x+sidestep.x, forward.y+sidestep.y, 0, 0);

    footModifier[secondSingleSupport][ROBOT_POSE]=p2;
    footModifier[firstSingleSupport][ROBOT_POSE]=p1;
  

    footModifier[firstDoubleSupport][RIGHT_FOOT].y+=sidestep.y;
    footModifier[firstDoubleSupport][RIGHT_FOOT].x+=sidestep.x;
  }
  else
  {
    Point p1(forward.x, forward.y, 0, 0);
    Point p2(forward.x+sidestep.x, forward.y+sidestep.y, 0, 0);

    footModifier[secondSingleSupport][ROBOT_POSE]= p1;
    footModifier[firstSingleSupport][ROBOT_POSE]=p2;
  
    footModifier[secondDoubleSupport][LEFT_FOOT].y+=sidestep.y;
    footModifier[secondDoubleSupport][LEFT_FOOT].x+=sidestep.x;
  }

  if (r<0)
  {
    footModifier[secondDoubleSupport][RIGHT_FOOT].r=r/2;
    footModifier[secondDoubleSupport][LEFT_FOOT].r=r/2;
    footModifier[firstDoubleSupport][RIGHT_FOOT].r=r;
    footModifier[secondSingleSupport][RIGHT_FOOT].r=r/2;
    footModifier[firstSingleSupport][ROBOT_POSE].r=r/2;
    footModifier[secondSingleSupport][ROBOT_POSE].r=r/2;
  }
  else
  {
    footModifier[firstDoubleSupport][RIGHT_FOOT].r=r/2;
    footModifier[firstDoubleSupport][LEFT_FOOT].r=r/2;
    footModifier[secondDoubleSupport][LEFT_FOOT].r=r;
    footModifier[firstSingleSupport][LEFT_FOOT].r=r/2;
    footModifier[secondSingleSupport][ROBOT_POSE].r=r/2;
    footModifier[firstSingleSupport][ROBOT_POSE].r=r/2;
  }



  //Charlie Chapling walk
  for (int i=0; i<4; i++)
  {
    footModifier[i][LEFT_FOOT].r+=baseRot;
    footModifier[i][RIGHT_FOOT].r-=baseRot;
  }	
}


PatternGenerator::~PatternGenerator(void)
{
}


inline unsigned int PatternGenerator::getPhaseLength(float ratio, float stepDur)
{
  return (int)floor(((ratio*stepDur*(1/theControllerParams.dt))/2)+0.5f);
}

void PatternGenerator::initWalkingPhase()
{
  if (currentWalkingPhase!=unlimitedDoubleSupport)
    return;

  if (currentWalkingPhase==unlimitedDoubleSupport)
  {
    currentWalkingPhase=secondSingleSupport;

    if (currentMovement.speed.translation.y()<0 || currentMovement.speed.rotation<0)
      currentWalkingPhase=firstSingleSupport;
  }
  stateCounter=singleSupportDuration() - 1;
}

int PatternGenerator::changeState(State newState, MovementInformation &moveInf)
{
  Footposition newStep;
  MovementInformation stopInf;
  switch(currentState)
  {
  case standby:
    if (newState==ready || newState==walking)
    {
      stateCounter=wasReady ? theWalkingEngineParams.walkTransition.crouchingDownPhaseLength : 30;
      currentState=goingToReady;
      currentMovement=moveInf;
    }
    else return TRANSITION_NOT_POSSIBLE;
    break;
  case ready:
    switch(newState)
    {
    case standby:
      stateCounter=theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
      currentState=goingToStandby;
      running=false;
      break;
    case walking:
    {
      currentState = walking;
      currentMovement = moveInf;
      initWalkingPhase();
      calcWalkParams();
      setStepLength();
      break;
    }
    case ready:
      applyStandType();
      break;

    default:
      return TRANSITION_NOT_POSSIBLE;
    }
    break;
  case walking:
    switch (newState)
    {
      // This command stops the robot, but is only requests the stopping phase
      // (by settings newState to stopping). The stopping phase will be started by
      // stopRobot (which sets currentState to stopping)
    case ready:
      this->newState=stopping;
      this->newMovement=stopInf;
      break;

      // This command changes the speed, if robot is walking
    case walking:
      if (moveInf!=currentMovement)
      {
        initWalkingPhase();
        this->newState=walking;
        newMovement=moveInf;
      }
      break;

      // Nothing else possible during walking
    default:
      return TRANSITION_NOT_POSSIBLE;
    }
    break;
  default:
    break;
  }

  return OK;
}

#define MAX_DURATION	4
#define MIN_DURATION	1

void PatternGenerator::applyStandType()
{
  switch(theMotionSelection.walkRequest.standType)
  {
  case WalkRequest::doubleSupport:
    stateCounter=1;
    currentWalkingPhase=unlimitedDoubleSupport;
    break;
  default:
    break;
  }
}

void PatternGenerator::calcWalkParams()
{
	if (dynamicDuration)
  {
    if (theMotionSelection.walkRequest.kickStrength == 0)
    {
      float distToMax = std::abs(theWalkingEngineParams.speedLimits.xForward / 1000 - currentMovement.speed.translation.x()) / (theWalkingEngineParams.speedLimits.xForward / 1000.f);
      curStepDuration = distToMax * 0.3f + (1 - distToMax) * 0.5f;
    }
    else
    {
      curStepDuration = 1.f;
    }
	}
}

void PatternGenerator::updateRobotPose()
{
  robotPose+=footModifier[currentWalkingPhase][ROBOT_POSE];
  direction=robotPose.r;
  distanceLeft-=footModifier[currentWalkingPhase][ROBOT_POSE];

  int footNum = (currentWalkingPhase == secondSingleSupport);
  robotPose += sideStepSum[footNum];
  sideStepSum[RIGHT_FOOT] = 0;
  sideStepSum[LEFT_FOOT] = 0;

}

bool PatternGenerator::isWalking()
{
  return !(currentState==ready || currentState==standby);
}


void PatternGenerator::resetDelays()
{
  for (int i=0; i<3; i++)
    speedApplyDelay[i]=0;
}

void PatternGenerator::decreaseDelays()
{
  for (int i=0; i<3; i++)
    if (speedApplyDelay[i]>0)
      speedApplyDelay[i]--;
}

void PatternGenerator::applySpeed(bool x, bool y, bool r_start, bool r_stop)
{

  if (x && speedApplyDelay[0]==0)
  {
    if (currentMovement.speed.translation.x() !=newMovement.speed.translation.x())
      speedApplyDelay[0]=theWalkingEngineParams.acceleration.speedApplyDelay;

    currentMovement.speed.translation.x() =newMovement.speed.translation.x();
  }
  if (y && speedApplyDelay[1]==0)
  {
    if (currentMovement.speed.translation.y() !=newMovement.speed.translation.y())
    {
      speedApplyDelay[1]=theWalkingEngineParams.acceleration.speedApplyDelay;
    }
    currentMovement.speed.translation.y() =newMovement.speed.translation.y();
  }
  if (speedApplyDelay[2]==0)
  {

    if (currentMovement.speed.rotation*newMovement.speed.rotation<0 || newMovement.speed.rotation == 0)
    {
      if (r_stop)
        currentMovement.speed.rotation =0;
    }
    else
    {
      if (r_start)
        currentMovement.speed.rotation =newMovement.speed.rotation;
    }
  }
}

void PatternGenerator::updateCounter()
{
  MovementInformation standInf;
  bool stopPossible=false;
  //int previewLength;
  if (currentMovement.speed.translation.x() ==0 &&
    currentMovement.speed.translation.y() ==0 &&
    currentMovement.speed.rotation ==0)
    stopPossible=true;

  if (stateCounter==0)
  {
    switch(currentState)
    {
    case ready:
    case standby:								
      stateCounter=1;
      currentWalkingPhase=unlimitedDoubleSupport;
      break;

    case goingToReady:
      stateCounter=1;
      currentState=ready;
      //running=true;
      currentMovement=standInf;
      calcWalkParams();
      setStepLength();
      currentWalkingPhase=unlimitedDoubleSupport;
      //previewDelta=theControllerParams.N+1;
      previewDelta = getOptimalPreviewLength();
      walkHeight = theControllerParams.z_h-CoM.z;
      wasReady = true;
      //if (previewDelta<(int)(doubleSupportDuration()+singleSupportDuration()+1))
      //  previewDelta=doubleSupportDuration()+singleSupportDuration()+1;
      handlePreviewDelta(theWalkingEngineParams.walkTransition.startingPhaseLength);
      break;

    case goingToStandby:
      stateCounter=1;
      currentState=standby;
      reset();
      break;

    case stopping:	
      if (newState==ready)
      {
        currentState=ready;
        stateCounter=1;
        newState=NA;
      }
    case walking:
      switch (currentWalkingPhase)
      {
      case firstSingleSupport:

        currentWalkingPhase=firstDoubleSupport;
        calcWalkParams();
        //updateRobotPose(); // Important for side steps.s
        stateCounter=doubleSupportDuration();
        if (newState==stopping && stopPossible && theMotionSelection.walkRequest.standType!=WalkRequest::rightSingleSupport)
        {
          calcWalkParams();
          stopRobot(robotPose);
        }
        break;
      case firstDoubleSupport:
        currentWalkingPhase=secondSingleSupport;
        updateRobotPose();
        if (newState==walking || newState==stopping)
          applySpeed(true, !(newMovement.speed.translation.y()<0), !(newMovement.speed.rotation<0), (currentMovement.speed.rotation >=0));

        calcWalkParams();
        
        setStepLength();

        robotPoseAfterStep.x = robotPose.x + 
          footModifier[secondDoubleSupport][ROBOT_POSE].x + 
          footModifier[firstSingleSupport][ROBOT_POSE].x +
          footModifier[firstDoubleSupport][ROBOT_POSE].x;
        ASSERT(robotPoseAfterStep.x == robotPoseAfterStep.x);
        robotPoseAfterStep.y = robotPose.y + 
          footModifier[secondDoubleSupport][ROBOT_POSE].y + 
          footModifier[firstSingleSupport][ROBOT_POSE].y +
          footModifier[firstDoubleSupport][ROBOT_POSE].y;
        ASSERT(robotPoseAfterStep.y == robotPoseAfterStep.y);
        if (currentMovement.speed.rotation<0)
          robotPoseAfterStep.r = robotPose.r + 
          footModifier[secondDoubleSupport][ROBOT_POSE].r + 
          footModifier[firstSingleSupport][ROBOT_POSE].r + 
          footModifier[firstDoubleSupport][ROBOT_POSE].r;

        if (currentMovement.speed.rotation>0)
          robotPoseAfterStep.r = robotPose.r + 
          footModifier[secondDoubleSupport][ROBOT_POSE].r + 
          footModifier[firstSingleSupport][ROBOT_POSE].r +
          footModifier[firstDoubleSupport][ROBOT_POSE].r + 
          footModifier[secondSingleSupport][ROBOT_POSE].r;
        ASSERT(robotPoseAfterStep.r == robotPoseAfterStep.r);
        if (theMotionSelection.walkRequest.standType==WalkRequest::rightSingleSupport && isSpeed0()) // ToDo: Fixme
          stopRobot(robotPose);
        else
        {
          stateCounter=singleSupportDuration();
        }

        break;
      case secondSingleSupport:

        currentWalkingPhase=secondDoubleSupport;
        calcWalkParams();
        //updateRobotPose();
        stateCounter=doubleSupportDuration();
        if (newState==stopping && stopPossible && theMotionSelection.walkRequest.standType!=WalkRequest::leftSingleSupport)
        {
          calcWalkParams();
          stopRobot(robotPose);	
        }
        break;
      case secondDoubleSupport:
        currentWalkingPhase=firstSingleSupport;
        updateRobotPose();
        if (newState==walking || newState==stopping)
          applySpeed(true, !(newMovement.speed.translation.y()>0), !(newMovement.speed.rotation>0), (currentMovement.speed.rotation <=0));
       
        calcWalkParams();
        
        setStepLength();

        robotPoseAfterStep.x = robotPose.x + 
          footModifier[firstDoubleSupport][ROBOT_POSE].x + 
          footModifier[secondSingleSupport][ROBOT_POSE].x +
          footModifier[secondDoubleSupport][ROBOT_POSE].x;
        robotPoseAfterStep.y = robotPose.y + 
          footModifier[firstDoubleSupport][ROBOT_POSE].y + 
          footModifier[secondSingleSupport][ROBOT_POSE].y +
          footModifier[secondDoubleSupport][ROBOT_POSE].y;

        if (currentMovement.speed.rotation < 0)
          robotPoseAfterStep.r = robotPose.r + 
          footModifier[firstDoubleSupport][ROBOT_POSE].r + 
          footModifier[secondSingleSupport][ROBOT_POSE].r + 
          footModifier[secondDoubleSupport][ROBOT_POSE].r +
          footModifier[firstSingleSupport][ROBOT_POSE].r;

        if (currentMovement.speed.rotation > 0)
          robotPoseAfterStep.r = robotPose.r + 
          footModifier[firstDoubleSupport][ROBOT_POSE].r + 
          footModifier[secondSingleSupport][ROBOT_POSE].r + 
          footModifier[secondDoubleSupport][ROBOT_POSE].r;
        
        if (theMotionSelection.walkRequest.standType== WalkRequest::leftSingleSupport && isSpeed0()) // ToDo: Fixme
          stopRobot(robotPose);
        else
        {
          stateCounter=singleSupportDuration();
        }
        break;
      default:
        if (currentState==walking)
        {
          cout << "Illegal support phase during walking" << endl;
          OUTPUT(idText, text, "Illegal support phase during walking");
        }
      }
      break;
    default:
      break;
    }
  }
  stateCounter--;
}

void PatternGenerator::reset()
{
  Point p(0,0,0,0);
  robotPose=p;
  newState=NA;
  currentState=standby;
  direction=0;
  previewDelta=0;
  running=false;
  resetDelays();
  currentTimeStamp=0;
  curPreviewLength=0;
}


StepData PatternGenerator::getNextStep() 
{
  Footposition step;
  // #ifndef WALKING_SIMULATOR
  float startZ = wasReady ? walkHeight : 0.3042051f-0.085f;
  float factor = 0;
  // #else
  // 	double startZ=0.331, factor=0;
  // #endif
  int crouchingLength = wasReady ? theWalkingEngineParams.walkTransition.crouchingDownPhaseLength : 30; // Was ist das für ein Hack?!

  switch(currentState)
  {
  case ready:
    //currentWalkingPhase=unlimitedDoubleSupport;
    //addStep();
    handlePreviewDelta();
    updateCounter();
    wasReady = true;
    break;

  case standby:
    //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
    step.direction=0;
    step.footPos[LEFT_FOOT]=0;
    step.footPos[RIGHT_FOOT]=0;
    step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ;
    step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
    step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
    step.footPos[LEFT_FOOT].r=baseRot;
    step.footPos[RIGHT_FOOT].r=-baseRot;
    robotPoseAfterStep = robotPose; // does not change...
    updateCounter();
    break;

  case stopping:
    addStep();
    updateCounter();
    break;

  case goingToStandby:
    if (crouchingLength>0)
    {
      factor = ((float)(stateCounter) / crouchingLength);
      //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
      step.direction=0;
      step.footPos[LEFT_FOOT]=0;
      step.footPos[RIGHT_FOOT]=0;
      step.footPos[RIGHT_FOOT].x=CoM.x*factor;
      step.footPos[LEFT_FOOT].x=CoM.x*factor;
      step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ+(startZ-theControllerParams.z_h+CoM.z)*factor;
      step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].r=baseRot;
      step.footPos[RIGHT_FOOT].r=-baseRot;
      robotPoseAfterStep = robotPose; // does not change...
    }
    updateCounter();
    break;

  case goingToReady:
    if (crouchingLength>0)
    {
      factor = 1 - ((float)(stateCounter) / crouchingLength);
      //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
      step.direction=0;
      step.footPos[LEFT_FOOT]=0;
      step.footPos[RIGHT_FOOT]=0;
      step.footPos[RIGHT_FOOT].x=CoM.x*factor;
      step.footPos[LEFT_FOOT].x=CoM.x*factor;
      step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ+(startZ-theControllerParams.z_h+CoM.z)*factor;
      step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].r=baseRot;
      step.footPos[RIGHT_FOOT].r=-baseRot;
      robotPoseAfterStep = robotPose; // does not change...
    }
    updateCounter();
    break;

  case walking:
    handlePreviewDelta();
    break;


  default:
    break;
  }

  return step;
}

void PatternGenerator::handlePreviewDelta(int max)
{

  // meaning of skip: skip only every second frame when previewDelta<0

  if ((previewDelta==0 /*|| !skip*/) && running)
  {
    addStep();
    updateCounter();
    skip=!skip;
  }

  while (previewDelta>0 && max>0) // Add more steps to get a higher preview
  {
    addStep();
    max--;
    previewDelta--;
    curPreviewLength++;
    updateCounter();
    if (previewDelta == 0)
      running = true;
  }

  if (/*skip &&*/ previewDelta<0) // Skip this frame, cause we need a lower preview
  {
    previewDelta++;
    curPreviewLength--;
    skip=!skip;
  }
}

unsigned int PatternGenerator::singleSupportDuration()
{
  return getPhaseLength(1-theWalkingEngineParams.footMovement.doubleSupportRatio, curStepDuration);
}

unsigned int PatternGenerator::doubleSupportDuration()
{
  return getPhaseLength(theWalkingEngineParams.footMovement.doubleSupportRatio, curStepDuration);
}

void PatternGenerator::calcFootRotation(int footNum)
{

}

State& PatternGenerator::getCurrentState()
{
  return currentState;
}

void PatternGenerator::addStep()
{
  Footposition newStep;
  newStep.footPos[LEFT_FOOT]=0;
  newStep.footPos[RIGHT_FOOT]=0;

  newStep.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
  newStep.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
 
  newStep.footPos[LEFT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][LEFT_FOOT].r);
  newStep.footPos[RIGHT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][RIGHT_FOOT].r);
  
  for (int i=0; i<2; i++)
    for (int j=0; j<3; j++)
      ASSERT(newStep.footPos[i].v[j] == newStep.footPos[i].v[j]);

  newStep.onFloor[LEFT_FOOT]=(footModifier[currentWalkingPhase][LEFT_FOOT].z==0);
  newStep.onFloor[RIGHT_FOOT]=(footModifier[currentWalkingPhase][RIGHT_FOOT].z==0);

  newStep.direction=direction;

  newStep.phase=currentWalkingPhase;
  newStep.singleSupportLen=singleSupportDuration();
  newStep.doubleSupportLen=doubleSupportDuration();
  newStep.stepDuration=curStepDuration;
  newStep.timestamp=currentTimeStamp; // time is the frame number when it is executed after preview
  currentTimeStamp++;

  newStep.frameInPhase=0;
  if (currentWalkingPhase==firstSingleSupport || currentWalkingPhase==secondSingleSupport)
    newStep.frameInPhase=newStep.singleSupportLen-stateCounter-1;
  if (currentWalkingPhase==firstDoubleSupport || currentWalkingPhase==secondDoubleSupport)
    newStep.frameInPhase=newStep.doubleSupportLen-stateCounter-1;

  ASSERT(newStep.frameInPhase < newStep.doubleSupportLen || newStep.frameInPhase < newStep.singleSupportLen);

  newStep.speed=currentMovement.speed;
  // Add to buffer for ZMP/IP-Controller (sent via FootSteps)

  if (steps!=NULL)
  {
    steps->addStep(newStep);
  }

  bool ok=false;
  if (newStep.onFloor[LEFT_FOOT])
  {
    if (deltaDirection[currentWalkingPhase]<0 || newStep.footPos[LEFT_FOOT].r>=direction)
      ok=true;
  }

  if (newStep.onFloor[RIGHT_FOOT])
  {
    if (deltaDirection[currentWalkingPhase]>0 || newStep.footPos[RIGHT_FOOT].r<=direction)
      ok=true;
  }

  if (ok)
    direction+=deltaDirection[currentWalkingPhase];

  lastStep=newStep;


  /*LOG("PatternGenerator_addStep", "step.footPos[0].x", newStep.footPos[0].x);
  LOG("PatternGenerator_addStep", "step.footPos[0].y", newStep.footPos[0].y);
  LOG("PatternGenerator_addStep", "step.footPos[0].z", newStep.footPos[0].z);
  LOG("PatternGenerator_addStep", "step.footPos[0].r", newStep.footPos[0].r);
  LOG("PatternGenerator_addStep", "step.footPos[1].x", newStep.footPos[1].x);
  LOG("PatternGenerator_addStep", "step.footPos[1].y", newStep.footPos[1].y);
  LOG("PatternGenerator_addStep", "step.footPos[1].z", newStep.footPos[1].z);
  LOG("PatternGenerator_addStep", "step.footPos[1].r", newStep.footPos[1].r);
  LOG("PatternGenerator_addStep", "step.direction", newStep.direction);
  LOG("PatternGenerator_addStep", "currentWalkingPhase", currentWalkingPhase);*/
}

void PatternGenerator::stopRobot(Point &pos)
{
  Footposition newStep;
  currentState=stopping;
  newState=ready;

  applyStandType();

  // That's not exact. Maybe there are more steps to do ...
  stateCounter=theWalkingEngineParams.walkTransition.stoppingPhaseLength;
  resetDelays();
}

void PatternGenerator::updateCoM(Point CoM)
{
  this->CoM=CoM;
}

void PatternGenerator::updateFootSteps(FootSteps & steps) 
{
  steps.reset();

  if (theFallDownState.state!=FallDownState::upright)
	  reset();

  this->steps=&steps;

  if (theWalkingEngineParams.footMovement.minStepDuration == theWalkingEngineParams.footMovement.maxStepDuration)
  {
    dynamicDuration=false;
    curStepDuration=theWalkingEngineParams.footMovement.minStepDuration;
  }
  else
    dynamicDuration=true;

  Point CoM(theRobotModel.centerOfMass.x()/1000, theRobotModel.centerOfMass.y()/1000, (theRobotModel.centerOfMass.z())/1000, 0);
  updateCoM(CoM);
  setPitch(thePatternGenRequest.pitch);

  if (thePatternGenRequest.newState!=PatternGenRequest::NA)
  {
    MovementInformation moveInf;
    moveInf.speed = thePatternGenRequest.speed;

    State newState=NA; // Selber Name wie Klassenmember! A: Tatsache! 100 Gummipunkte!

    // Translate public states to internal
    switch (thePatternGenRequest.newState)
    {
    case standby:
      newState=standby;
      break;

    case ready:
      newState=ready;
      break;

    case walking:
      newState=walking;
      break;
    default:
      OUTPUT(idText, text,"PatternGenerator: switch (thePatternGenRequest.newState) hit default. Should not happen...");
    }

    changeState(newState, moveInf);
  }
  
  static int timediff;
  timediff = theReferenceModificator.creationTime - currentTimeStamp;
  MODIFY("module:PatternGenerator:timediff", timediff);

  if (running)
  {
    //ASSERT(theReferenceModificator.creationTime == currentTimeStamp);  

    for (int dim = 0; dim < 2; dim++)
    {
      Point mod = Point(theReferenceModificator[dim].x, theReferenceModificator[dim].y);
      for (int footNum = 0; footNum < 2; footNum++)
      {
        if (theReferenceModificator.aTime[dim].startFoot[footNum] != -1)
        {
          sideStepSum[footNum] += mod;
        }
      }
    }
  }

  steps.pitch=curPitch;
  steps.suggestedStep=getNextStep();
  steps.running=running;

  speedBuffer.push_front(currentMovement.speed);

  steps.time = currentTimeStamp;
  steps.robotPoseAfterStep=robotPoseAfterStep;
  steps.cycleLength=(int)(curStepDuration*100);
  if (running)
  {
    cyclePosCounter++;
    if (cyclePosCounter==steps.cycleLength)
      cyclePosCounter=0;
  }
  else
    cyclePosCounter=0;
  steps.positionInCycle=cyclePosCounter;

  this->steps=NULL;

  decreaseDelays();
  
  PLOT("module:PatternGenerator:PreviewLength", curPreviewLength);
}


void PatternGenerator::updateSpeedInfo(SpeedInfo & speedInfo)
{
  if (ControllerParams::N!=0 && speedBuffer.size()>=ControllerParams::N-1)
  {
    speedInfo.speed=speedBuffer[ControllerParams::N-1];
    speedInfo.speedBeforePreview = speedBuffer[0];
    speedInfo.timestamp = currentTimeStamp - ControllerParams::N;
  }
  else
	  speedInfo.speed = Point();
  speedInfo.deceleratedByAcc = decelByAcc;
}