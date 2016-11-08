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
 * @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator2016.cpp
 * Generator for foot steps
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#include "PatternGenerator2016.h"
#include <iostream>
#include "Platform/File.h"

#define LOGGING

#ifndef WALKING_SIMULATOR
#include "Tools/Math/BHMath.h"

#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

using namespace std;


PatternGenerator2016::PatternGenerator2016() : speedBuffer(Pose2f())
{
  wasReady = false;
  reset();
  baseRot=0;
  currentStepFileIndex = -1;
  curPitch=targetPitch=0;
  loadStepFiles();
  //saveStepFiles();
}

void PatternGenerator2016::setPitch(float pitch)
{
  targetPitch=pitch;
}

float PatternGenerator2016::getCurrentPitch()
{
  return curPitch;
}

float PatternGenerator2016::getStepLength(float speed)
{
  return speed*curStepDuration/2;
}

int PatternGenerator2016::getOptimalPreviewLength()
{
  return std::max((unsigned int)theControllerParams.N+1, doubleSupportDuration()+singleSupportDuration()+1);
}

void PatternGenerator2016::setStepLength()
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
  
  footModifier[unlimitedSingleSupportLeft][RIGHT_FOOT]=p2;
  footModifier[unlimitedSingleSupportLeft][LEFT_FOOT]=p1;
  footModifier[unlimitedSingleSupportRight][RIGHT_FOOT]=p1;
  footModifier[unlimitedSingleSupportRight][LEFT_FOOT]=p2;
  
  
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
    footModifier[unlimitedSingleSupportRight][ROBOT_POSE]=p2;
    footModifier[firstSingleSupport][ROBOT_POSE]=p1;
    footModifier[unlimitedSingleSupportLeft][ROBOT_POSE]=p1;
    
    
    footModifier[firstDoubleSupport][RIGHT_FOOT].y+=sidestep.y;
    footModifier[firstDoubleSupport][RIGHT_FOOT].x+=sidestep.x;
  }
  else
  {
    Point p1(forward.x, forward.y, 0, 0);
    Point p2(forward.x+sidestep.x, forward.y+sidestep.y, 0, 0);
    
    footModifier[secondSingleSupport][ROBOT_POSE]= p1;
    footModifier[unlimitedSingleSupportRight][ROBOT_POSE]= p1;
    footModifier[firstSingleSupport][ROBOT_POSE]=p2;
    footModifier[unlimitedSingleSupportLeft][ROBOT_POSE]=p2;
    
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


PatternGenerator2016::~PatternGenerator2016(void)
{
}


inline unsigned int PatternGenerator2016::getPhaseLength(float ratio, float stepDur)
{
  return (int)floor(((ratio*stepDur*(1/theControllerParams.dt))/2)+0.5f);
}

void PatternGenerator2016::initWalkingPhase()
{
  if (currentWalkingPhase!=unlimitedDoubleSupport &&
      currentWalkingPhase!=unlimitedSingleSupportLeft &&
      currentWalkingPhase!=unlimitedSingleSupportRight)
    return;
  
  if (currentWalkingPhase==unlimitedDoubleSupport)
  {
    currentWalkingPhase=secondSingleSupport;
    
    if (currentMovement.speed.translation.y()<0 || currentMovement.speed.rotation<0)
      currentWalkingPhase=firstSingleSupport;
  }
  else
  {
    if (currentWalkingPhase==unlimitedSingleSupportLeft)
    {
      currentWalkingPhase=firstSingleSupport;
      currentMovement.speed.rotation=0;
    }
    else
    {
      currentWalkingPhase=secondSingleSupport;
      currentMovement.speed.rotation=0;
    }
  }
  stateCounter=singleSupportDuration() - 1;
}

Point PatternGenerator2016::getPoseAfterCustomStep()
{
  Point lS = executedStep->onFloor[LEFT_FOOT] ? executedStep->footPos[LEFT_FOOT] : executedStep->footPos[RIGHT_FOOT];
  int sign = executedStep->onFloor[LEFT_FOOT] ? -1 : 1;
  Point yDist(0, sign * theWalkingEngineParams.footYDistance);
  yDist.rotate2D(lS.r);
  return toCustomCS(lS + yDist);
}

void PatternGenerator2016::transitionToWalk(MovementInformation &moveInf)
{
  if (currentStepFileIndex != -1)
    robotPose = getPoseAfterCustomStep();
  
  currentState=walking;
  currentMovement.speed = Pose2f();
  newMovement = moveInf;
  applySpeed(true,
             (currentWalkingPhase == firstSingleSupport &&
              newMovement.speed.translation.y()>=0) ||
             (currentWalkingPhase == secondSingleSupport &&
              (newMovement.speed.translation.y()<0)) ||
             currentWalkingPhase == unlimitedDoubleSupport,
             (currentWalkingPhase == firstSingleSupport &&
              newMovement.speed.rotation<0) ||
             (currentWalkingPhase == secondSingleSupport &&
              newMovement.speed.rotation>0) ||
             currentWalkingPhase == unlimitedDoubleSupport, true);
  if (currentMovement.speed.translation.y() * currentMovement.speed.rotation < 0)
    currentMovement.speed.translation.y() = 0;
  currentStepFileIndex = -1;
  initWalkingPhase();
  calcWalkParams();
  setStepLength();
  currentCustomStep = WalkRequest::none;
}

int PatternGenerator2016::changeState(State newState, MovementInformation &moveInf)
{
  Footposition newStep;
  MovementInformation stopInf;
  switch(currentState)
  {
    case standby:
      if (newState==ready || newState==walking)
      {
        stateCounter=wasReady ? theWalkingEngineParams.crouchingDownPhaseLength : 30;
        currentState=goingToReady;
        currentMovement=moveInf;
      }
      else return TRANSITION_NOT_POSSIBLE;
      break;
    case ready:
      switch(newState)
      {
      case standby:
        stateCounter=theWalkingEngineParams.crouchingDownPhaseLength;
        currentState=goingToStandby;
        running=false;
        break;
      case walking:
        if (curPitch<=theWalkingEngineParams.maxWalkPitch)
          transitionToWalk(moveInf);
        break;
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
          moveInf.timestamp = currentTimeStamp;
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

void PatternGenerator2016::applyStandType()
{
  switch(theMotionSelection.walkRequest.standType)
  {
  case WalkRequest::doubleSupport:
      stateCounter=1;
      if (currentWalkingPhase==unlimitedSingleSupportLeft)
      {
        currentWalkingPhase=firstSingleSupport;
        currentState=walking;
        stateCounter=singleSupportDuration();
      }
      else if(currentWalkingPhase==unlimitedSingleSupportRight)
      {
        currentWalkingPhase=secondSingleSupport;
        currentState=walking;
        stateCounter=singleSupportDuration();
      }
      else
        currentWalkingPhase=unlimitedDoubleSupport;
      break;
      
    default:
      break;
  }
}

bool PatternGenerator2016::transitionToCustomStepsNoCheck()
{
  if (theMotionSelection.walkRequest.stepRequest >=
      WalkRequest::StepRequest::beginScript //&&
//      deltaDirection[0] == 0 &&
//      deltaDirection[1] == 0 &&
//      deltaDirection[2] == 0 &&
//      deltaDirection[3] == 0
      )
  {
    
    int idx = (int)(theMotionSelection.walkRequest.stepRequest - WalkRequest::StepRequest::beginScript - 1);
    
    if ((currentWalkingPhase == secondSingleSupport &&
         stepFiles[idx].steps.begin()->onFloor[1]) ||
        (currentWalkingPhase == firstSingleSupport &&
         stepFiles[idx].steps.begin()->onFloor[0]))
    {
      executedStep = stepFiles[idx].steps.begin();
      int footIndex = executedStep->onFloor[LEFT_FOOT] ? LEFT_FOOT : RIGHT_FOOT;
      Point CStoFoot = executedStep->footPos[footIndex];
      float coordRot = lastStep.footPos[footIndex].r;
      CStoFoot.rotate2D(coordRot);
      
      coordinateSystem = lastStep.footPos[footIndex] - CStoFoot;
      coordinateSystem.r = coordRot;

      stateCounter = executedStep->duration;
      currentStepFileIndex = idx;
      setCustomWalkingPhase();
      currentState = customSteps;
      currentMovement.speed = Point();
      currentCustomStep = theMotionSelection.walkRequest.stepRequest;
      setCustomStepDeltaDirection();
    }
  }
  
  return currentStepFileIndex != -1;
}

bool PatternGenerator2016::transitionToCustomSteps()
{
  
  // Nur aktivieren, wenn der nächste Schritt kleiner werden muss, um
  // Zielpunkt zu erreichen. Muss er größer werden, liegt es außerhalb
  // der Reichweite. Um diese Fälle zu vermeiden muss die aktuelle
  // Geschwindigkeit hier automatisch gewählt werden, so dass der Fall
  // früher oder später eintritt.
  
  Vector2f ballPos = theBallModelAfterPreview.estimate.position;
  coordinateSystem = theWalkingInfo.ballCStoWalkingCS(ballPos);
  coordinateSystem.r = robotPose.r;
  
  if (theMotionSelection.walkRequest.stepRequest >=
      WalkRequest::StepRequest::beginScript)
  {

    int idx = (int)(theMotionSelection.walkRequest.stepRequest - WalkRequest::StepRequest::beginScript - 1);
    
    if ((currentWalkingPhase == firstDoubleSupport &&
         stepFiles[idx].steps.begin()->onFloor[1]) ||
        (currentWalkingPhase == secondDoubleSupport &&
         stepFiles[idx].steps.begin()->onFloor[0]))
    {
      executedStep = stepFiles[idx].steps.begin();
      int footIndex = executedStep->onFloor[LEFT_FOOT] ? LEFT_FOOT : RIGHT_FOOT;
      Point firstStepBallCS = executedStep->footPos[footIndex];
      firstStepBallCS.rotate2D(robotPose.r);
      /* Position of the first foot of te custom steps in the
       coordinate system of the ball, converted to the CS of
       the walking engine */
      Point firstStepWECS = coordinateSystem + firstStepBallCS;
      Point firstStepDiff = firstStepWECS - lastStep.footPos[footIndex];
      Point footDist = firstStepWECS - lastStep.footPos[!footIndex];
      footDist.rotate2D(-robotPose.r);
      float firstStepLength = firstStepDiff.euklidDistance2D();
      float maxRadius = static_cast<float>(sqrt(theWalkingEngineParams.maxSpeedXForwardOmni *
                             theWalkingEngineParams.maxSpeedXForwardOmni +
                             theWalkingEngineParams.maxSpeedY *
                             theWalkingEngineParams.maxSpeedY)) *
                             theControllerParams.dt *
                             singleSupportDuration() / 1000;
      if (firstStepLength < maxRadius && fabs(footDist.y) > 0.1)
      {
        stateCounter = executedStep->duration;
        currentStepFileIndex = idx;
        setCustomWalkingPhase();
        currentState = customSteps;
        currentMovement.speed = Point();
        currentCustomStep = theMotionSelection.walkRequest.stepRequest;
      }
    }
  }
  
  return currentStepFileIndex != -1;
}

void PatternGenerator2016::calcWalkParams()
{
  if (dynamicDuration)
  {
    if (theMotionSelection.walkRequest.kickStrength == 0)
    {
      float distToMax = fabs(theWalkingEngineParams.maxSpeedXForward / 1000 - (float)currentMovement.speed.translation.x()) / (theWalkingEngineParams.maxSpeedXForward / 1000.f);
      curStepDuration = distToMax * 0.3f + (1 - distToMax) * 0.5f;
    }
    else
    {
      curStepDuration = 1.f;
    }
  }
}

void PatternGenerator2016::updateRobotPose()
{
  robotPose+=footModifier[currentWalkingPhase][ROBOT_POSE];
  direction=robotPose.r;
  distanceLeft-=footModifier[currentWalkingPhase][ROBOT_POSE];
  
  int footNum = (currentWalkingPhase == secondSingleSupport);
  robotPose += sideStepSum[footNum];
  sideStepSum[RIGHT_FOOT] = 0;
  sideStepSum[LEFT_FOOT] = 0;
  
}

bool PatternGenerator2016::isWalking()
{
  return !(currentState==ready || currentState==standby);
}


void PatternGenerator2016::resetDelays()
{
  for (int i=0; i<3; i++)
    speedApplyDelay[i]=0;
}

void PatternGenerator2016::decreaseDelays()
{
  for (int i=0; i<3; i++)
    if (speedApplyDelay[i]>0)
      speedApplyDelay[i]--;
}

void PatternGenerator2016::applySpeed(bool x, bool y, bool r_start, bool r_stop)
{
  
  if (x && speedApplyDelay[0]==0)
  {
    if (currentMovement.speed.translation.x()!=newMovement.speed.translation.x())
      speedApplyDelay[0]=theWalkingEngineParams.speedApplyDelay;
    
    currentMovement.speed.translation.x() =newMovement.speed.translation.x();
    PLOT("module:PatternGenerator2016:AcceptedX", currentMovement.speed.translation.x()*1000);
    acceptedDrawn[0] = true;
  }
  if (y && speedApplyDelay[1]==0)
  {
    if (currentMovement.speed.translation.y() !=newMovement.speed.translation.y())
    {
      speedApplyDelay[1]=theWalkingEngineParams.speedApplyDelay;
    }
    currentMovement.speed.translation.y() =newMovement.speed.translation.y();
    PLOT("module:PatternGenerator2016:AcceptedY", currentMovement.speed.translation.y()*1000);
    acceptedDrawn[1] = true;
  }
  if (speedApplyDelay[2]==0)
  {
    
    if (currentMovement.speed.rotation*newMovement.speed.rotation<0 || newMovement.speed.rotation == 0)
    {
      if (r_stop)
      {
        currentMovement.speed.rotation=0;
        acceptedDrawn[2] = true;
      }
    }
    else
    {
      if (r_start)
      {
        currentMovement.speed.rotation=newMovement.speed.rotation;
        acceptedDrawn[2] = true;
      }
    }
    if (acceptedDrawn[2])
      PLOT("module:PatternGenerator2016:AcceptedR", currentMovement.speed.translation.y()*1000);
  }
}

void PatternGenerator2016::setCustomWalkingPhase()
{
  if (executedStep->onFloor[LEFT_FOOT] && !executedStep->onFloor[RIGHT_FOOT])
    currentWalkingPhase = firstSingleSupport;
  else if (!executedStep->onFloor[LEFT_FOOT] &&
           executedStep->onFloor[RIGHT_FOOT])
    currentWalkingPhase = secondSingleSupport;
  else if (executedStep->onFloor[LEFT_FOOT] &&
           executedStep->onFloor[RIGHT_FOOT] &&
           currentWalkingPhase == firstSingleSupport)
    currentWalkingPhase = firstDoubleSupport;
  else if (executedStep->onFloor[LEFT_FOOT] &&
           executedStep->onFloor[RIGHT_FOOT] &&
           currentWalkingPhase == secondSingleSupport)
    currentWalkingPhase = secondDoubleSupport;
}

void PatternGenerator2016::setCustomStepDeltaDirection()
{
  float targetRot, r;
  StepsFile::StepsIterator _nextStep(executedStep);
  _nextStep++;
  if (_nextStep != stepFiles[currentStepFileIndex].steps.end() &&
      _nextStep->onFloor[LEFT_FOOT] &&
      _nextStep->onFloor[RIGHT_FOOT])
  {
    targetRot = (_nextStep->footPos[LEFT_FOOT].rotation - _nextStep->footPos[RIGHT_FOOT].rotation) * 0.5f + _nextStep->footPos[RIGHT_FOOT].rotation;
    float currentRot = direction - coordinateSystem.r;
    r = (targetRot - currentRot) / executedStep->duration;
  }
  else if (_nextStep == stepFiles[currentStepFileIndex].steps.end())
  {
    r = newMovement.speed.rotation * theControllerParams.dt;
  }
  else
  {
    r = 0;
  }
  deltaDirection[firstSingleSupport]=deltaDirection[secondSingleSupport]=r;
  deltaDirection[firstDoubleSupport]=deltaDirection[secondDoubleSupport]=0;
}

void PatternGenerator2016::updateCounter()
{
  MovementInformation standInf;
  bool stopPossible=false;
  StepsFile::StepsIterator _nextStep;
  //int previewLength;
  if (currentMovement.speed.translation.x() == 0 &&
      currentMovement.speed.translation.y() == 0 &&
      currentMovement.speed.rotation == 0 &&
      currentStepFileIndex == -1)
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
        handlePreviewDelta(theWalkingEngineParams.startingPhaseLength);
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
        break;
        
      case customSteps:
        executedStep++;
        setCustomWalkingPhase();
        robotPoseAfterStep = getPoseAfterCustomStep();
        _nextStep = executedStep;
        _nextStep++;
        if (_nextStep == stepFiles[currentStepFileIndex].steps.end())
        {
          setCustomWalkingPhase();
          //currentWalkingPhase = (WalkingPhase)((currentWalkingPhase + 1) % 4);
          MovementInformation m;
          m.speed = thePatternGenRequest.speed;
          transitionToWalk(m);
          //stateCounter = doubleSupportDuration();
          stateCounter = singleSupportDuration();
        }
        else
        {
          setCustomStepDeltaDirection();
          stateCounter = executedStep->duration;
        }
        break;
        
      case walking:
        switch (currentWalkingPhase)
      {
        case firstSingleSupport:
          currentWalkingPhase=firstDoubleSupport;
          calcWalkParams();
          stateCounter=doubleSupportDuration();
          if (newState==stopping && stopPossible &&
              theMotionSelection.walkRequest.standType!=WalkRequest::rightSingleSupport)
          {
            calcWalkParams();
            stopRobot(robotPose);
          }
          break;
        case firstDoubleSupport:
          currentWalkingPhase=secondSingleSupport;
          updateRobotPose();
          if (transitionToCustomStepsNoCheck())
            break;
          if (newState==walking || newState==stopping)
            applySpeed(true,
                       !(newMovement.speed.translation.y()<0),
                       !(newMovement.speed.rotation<0), (currentMovement.speed.rotation>=0));
          
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
          stateCounter=doubleSupportDuration();
          if (newState==stopping && stopPossible &&
              theMotionSelection.walkRequest.standType!=WalkRequest::leftSingleSupport)
          {
            calcWalkParams();
            stopRobot(robotPose);
          }
          break;
        case secondDoubleSupport:
          currentWalkingPhase=firstSingleSupport;
          updateRobotPose();
          if (transitionToCustomStepsNoCheck())
            break;
          if (newState==walking || newState==stopping)
            applySpeed(true, 
              !(newMovement.speed.translation.y()>0), 
              !(newMovement.speed.rotation>0), 
              (currentMovement.speed.rotation<=0));
          
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
          
          if (theMotionSelection.walkRequest.standType==WalkRequest::leftSingleSupport && isSpeed0()) // ToDo: Fixme
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

void PatternGenerator2016::reset()
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
  currentStepFileIndex = -1;
  delay = 0;
  lastStep.onFloor[0] = lastStep.onFloor[1] = false;
}

float PatternGenerator2016::getPitchSpeed()
{
  if (targetPitch-curPitch>0)
    return theWalkingEngineParams.pitchSpeed;
  else
    return -theWalkingEngineParams.pitchSpeed;
}

void PatternGenerator2016::handlePitch()
{
  // interpolate and set the pitch
  
  if (curPitch>theWalkingEngineParams.maxWalkPitch && currentState!=ready && targetPitch-curPitch>=0)
    return;
  
  if (std::abs(targetPitch-curPitch)>theWalkingEngineParams.pitchSpeed*theControllerParams.dt)
  {
    curPitch+=getPitchSpeed()*theControllerParams.dt;
  }
  else
    curPitch=targetPitch;
}


StepData PatternGenerator2016::getNextStep()
{
  Footposition step;
  // #ifndef WALKING_SIMULATOR
  float startZ = wasReady ? walkHeight : 0.3042051f-0.085f;
  float factor = 0;
  // #else
  // 	double startZ=0.331, factor=0;
  // #endif
  int crouchingLength = wasReady ? theWalkingEngineParams.crouchingDownPhaseLength : 30; // Was ist das für ein Hack?!
  
  switch(currentState)
  {
    case ready:
      //currentWalkingPhase=unlimitedDoubleSupport;
      //addStep();
      handlePreviewDelta();
      handlePitch();
      updateCounter();
      wasReady = true;
      break;
      
    case standby:
      //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
      step.direction=0;
      step.footPos[LEFT_FOOT]=0;
      step.footPos[RIGHT_FOOT]=0;
      step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ;
      step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
      step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
      step.footPos[LEFT_FOOT].r=baseRot;
      step.footPos[RIGHT_FOOT].r=-baseRot;
      robotPoseAfterStep = robotPose; // does not change...
      updateCounter();
      break;
      
    case stopping:
      addStep();
      handlePitch();
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
        step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
        step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
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
        step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
        step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
        step.footPos[LEFT_FOOT].r=baseRot;
        step.footPos[RIGHT_FOOT].r=-baseRot;
        robotPoseAfterStep = robotPose; // does not change...
      }
      updateCounter();
      break;
      
    case walking:
    case customSteps:
      handlePreviewDelta();
      break;
      
      
    default:
      break;
  }
  
  return step;
}

void PatternGenerator2016::handlePreviewDelta(int max)
{
  
  // meaning of skip: skip only every second frame when previewDelta<0
  
  if ((previewDelta==0 /*|| !skip*/) && running)
  {
    addStep();
    updateCounter();
    handlePitch();
    skip=!skip;
  }
  
  while (previewDelta>0 && max>0) // Add more steps to get a higher preview
  {
    addStep();
    max--;
    previewDelta--;
    curPreviewLength++;
    updateCounter();
    handlePitch();
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

unsigned int PatternGenerator2016::singleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (!executedStep->onFloor[LEFT_FOOT] || !executedStep->onFloor[RIGHT_FOOT])
      return executedStep->duration;
    else if (std::next(executedStep) !=
             stepFiles[currentStepFileIndex].steps.end())
      return std::next(executedStep)->duration;
  }
  return getPhaseLength(1-theWalkingEngineParams.doubleSupportRatio, curStepDuration);
}

unsigned int PatternGenerator2016::doubleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (executedStep->onFloor[LEFT_FOOT] && executedStep->onFloor[RIGHT_FOOT])
      return executedStep->duration;
    else if (std::next(executedStep) !=
             stepFiles[currentStepFileIndex].steps.end())
      return std::next(executedStep)->duration;
  }
  
  return getPhaseLength(theWalkingEngineParams.doubleSupportRatio, curStepDuration);
}


State& PatternGenerator2016::getCurrentState()
{
  return currentState;
}

Point PatternGenerator2016::toCustomCS(Point p)
{
  return coordinateSystem + p.rotate2D(coordinateSystem.r);
}

void PatternGenerator2016::addStep()
{
  Footposition newStep;
  newStep.footPos[LEFT_FOOT]=0;
  newStep.footPos[RIGHT_FOOT]=0;
  
  newStep.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
  newStep.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
  
  newStep.footPos[LEFT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][LEFT_FOOT].r);
  newStep.footPos[RIGHT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][RIGHT_FOOT].r);
  
  
  /////////////////////////////////////////////////////////////////////////////
  
  if (currentStepFileIndex != -1)
  {
    for (int i = 0; i < 2; i++)
    {
      
      newStep.footPos[i] = toCustomCS(executedStep->footPos[i]);
      newStep.onFloor[i] = executedStep->onFloor[i];
      if (!newStep.onFloor[i])
        newStep.footPos[i].z = theWalkingEngineParams.stepHeight[0];
      else
        newStep.footPos[i].z = 0;
    }
    newStep.stepDuration = executedStep->duration;
  }
  else
  {
    newStep.footPos[LEFT_FOOT] += robotPose + footModifier[currentWalkingPhase][LEFT_FOOT];
    newStep.footPos[RIGHT_FOOT] += robotPose + footModifier[currentWalkingPhase][RIGHT_FOOT];
    newStep.stepDuration=curStepDuration;
    newStep.onFloor[LEFT_FOOT]=(footModifier[currentWalkingPhase][LEFT_FOOT].z==0);
    newStep.onFloor[RIGHT_FOOT]=(footModifier[currentWalkingPhase][RIGHT_FOOT].z==0);
    
  }
  
  newStep.footPos[LEFT_FOOT] += sideStepSum[LEFT_FOOT];
  newStep.footPos[RIGHT_FOOT] += sideStepSum[RIGHT_FOOT];
  
  /////////////////////////////////////////////////////////////////////////////
  
  for (int i=0; i<2; i++)
  {
    ASSERT(newStep.footPos[i] == newStep.footPos[i]);
    
    // This makes boom if a foot on the floor is repositioned by more than 1 mm
    // Check transitions from phase to phase including rotation, also custom
    // steps. Same for rotation.
    ASSERT(!(lastStep.onFloor[i] && newStep.onFloor[i]) || (lastStep.footPos[i].euklidDistance3D(newStep.footPos[i]) < 0.001 && fabs(lastStep.footPos[i].r - newStep.footPos[i].r) < 0.01));
  }
  
  newStep.direction=direction;
  newStep.customStep = currentCustomStep;
  newStep.phase=currentWalkingPhase;
  newStep.singleSupportLen=singleSupportDuration();
  newStep.doubleSupportLen=doubleSupportDuration();
  newStep.frameInPhase=0;
  if (currentWalkingPhase==firstSingleSupport || currentWalkingPhase==secondSingleSupport)
    newStep.frameInPhase=newStep.singleSupportLen-stateCounter-1;
  if (currentWalkingPhase==firstDoubleSupport || currentWalkingPhase==secondDoubleSupport)
    newStep.frameInPhase=newStep.doubleSupportLen-stateCounter-1;
  newStep.timestamp=currentTimeStamp; // time is the frame number when it is executed after preview
  currentTimeStamp++;
  

  ASSERT(newStep.frameInPhase < newStep.doubleSupportLen || newStep.frameInPhase < newStep.singleSupportLen);
  
  newStep.speed=currentMovement.speed;
  // Add to buffer for ZMP/IP-Controller (sent via FootSteps)
  
  if (steps!=NULL)
  {
    steps->addStep(newStep);
  }
  
  bool ok = false;
  if (currentStepFileIndex == -1)
  {
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
  }
  else
    ok = true;
  
  if (ok)
    direction+=deltaDirection[currentWalkingPhase];
  
  LOG("PatternGenerator2016_addStep", "step.footPos[0].x", newStep.footPos[0].x);
  LOG("PatternGenerator2016_addStep", "step.footPos[0].y", newStep.footPos[0].y);
  LOG("PatternGenerator2016_addStep", "step.footPos[0].z", newStep.footPos[0].z);
  LOG("PatternGenerator2016_addStep", "step.footPos[0].r", newStep.footPos[0].r);
  LOG("PatternGenerator2016_addStep", "step.footPos[1].x", newStep.footPos[1].x);
  LOG("PatternGenerator2016_addStep", "step.footPos[1].y", newStep.footPos[1].y);
  LOG("PatternGenerator2016_addStep", "step.footPos[1].z", newStep.footPos[1].z);
  LOG("PatternGenerator2016_addStep", "step.footPos[1].r", newStep.footPos[1].r);
  LOG("PatternGenerator2016_addStep", "step.direction", newStep.direction);
  LOG("PatternGenerator2016_addStep", "timestamp", newStep.timestamp);
  LOG("PatternGenerator2016_addStep", "Ball x", coordinateSystem.x);
  LOG("PatternGenerator2016_addStep", "Ball y", coordinateSystem.y);

  for (int i = 0; i < 2; i++)
  {
    if (newStep.onFloor[i])
      lastStep.footPos[i] = newStep.footPos[i];
    lastStep.onFloor[i] = newStep.onFloor[i];
  }
}

void PatternGenerator2016::stopRobot(Point &pos)
{
  Footposition newStep;
  currentState=stopping;
  newState=ready;
  
  applyStandType();
  
  // That's not exact. Maybe there are more steps to do ...
  stateCounter=theWalkingEngineParams.stoppingPhaseLength;
  resetDelays();
}

void PatternGenerator2016::updateCoM(Point CoM)
{
  this->CoM=CoM;
}

void PatternGenerator2016::update(FootSteps & steps)
{
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2016:saveCustomSteps")
    saveStepFiles();
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2016:loadCustomSteps")
    loadStepFiles();
  
  DECLARE_PLOT("module:PatternGenerator2016:AcceptedX");
  DECLARE_PLOT("module:PatternGenerator2016:AcceptedY");
  DECLARE_PLOT("module:PatternGenerator2016:AcceptedR");
  steps.reset();
  
  acceptedDrawn[0] = false;
  acceptedDrawn[1] = false;
  acceptedDrawn[2] = false;
  
  if (theFallDownState.state!=FallDownState::upright)
    reset();
  
  this->steps=&steps;
  
  if (theWalkingEngineParams.stepDuration!=0)
  {
    dynamicDuration=false;
    if (currentStepFileIndex == -1)
      curStepDuration=theWalkingEngineParams.stepDuration;
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
        OUTPUT(idText, text,"PatternGenerator2016: switch (thePatternGenRequest.newState) hit default. Should not happen...");
    }
    
    changeState(newState, moveInf);
  }
  
  static int timediff;
  timediff = theReferenceModificator.creationTime - currentTimeStamp;
  MODIFY("module:PatternGenerator2016:timediff", timediff);
  
  if (running)
  {

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
    
    LOG("PatternGenerator2016_addStep", "currentWalkingPhase", currentWalkingPhase);
  }
  else
    cyclePosCounter=0;
  steps.positionInCycle=cyclePosCounter;
  
  this->steps=NULL;
  
  decreaseDelays();
  
  PLOT("module:PatternGenerator2016:PreviewLength", curPreviewLength);
  if (!acceptedDrawn[0])
    PLOT("module:PatternGenerator2016:AcceptedX", 0);
  if (!acceptedDrawn[1])
    PLOT("module:PatternGenerator2016:AcceptedY", 0);
  if (!acceptedDrawn[2])
    PLOT("module:PatternGenerator2016:AcceptedR", 0);
}


void PatternGenerator2016::update(SpeedInfo & speedInfo)
{
  if (ControllerParams::N!=0 && speedBuffer.size()>=ControllerParams::N-1)
  {
    speedInfo.speed = speedBuffer[ControllerParams::N-1];
    speedInfo.speedBeforePreview = speedBuffer[0];
    speedInfo.timestamp = currentTimeStamp - ControllerParams::N;
  }
  else
    speedInfo.speed = Point();
  speedInfo.deceleratedByAcc = decelByAcc;
  speedInfo.currentCustomStep = currentCustomStep;

  PLOT("representation:SpeedInfo:speed.x", speedInfo.speed.translation.x() * 1000);
}

void PatternGenerator2016::Step::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(footPos);
  STREAM(duration);
  STREAM(onFloor);
  STREAM(free);
  STREAM_REGISTER_FINISH;
}

void PatternGenerator2016::StepsFile::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(steps);
  STREAM_REGISTER_FINISH;
}

void PatternGenerator2016::loadStepFile(std::string file, int idx)
{
  stepFiles[idx].steps.clear();
  
  InMapFile s(file);
  if(s.exists())
    s >> stepFiles[idx];
  else
  {
    InMapFile s(file);
    if(s.exists())
      s >> stepFiles[idx];
    else
    {
      OUTPUT(idText, text, std::string("Unable to load ") << file << ". Creating new.");
      stepFiles[idx].steps.push_back(Step());
      saveStepFile(file, idx);
    }
  }
  
  // Must begin and end with single support
  ASSERT(!(stepFiles[idx].steps.front().onFloor[0] &&
           stepFiles[idx].steps.front().onFloor[1]) &&
         !(stepFiles[idx].steps.back().onFloor[0] &&
           stepFiles[idx].steps.back().onFloor[1]));
  
  // First steps have zero rotation by definition
  ASSERT(stepFiles[idx].steps.front().footPos[0].rotation == 0 &&
         stepFiles[idx].steps.front().footPos[1].rotation == 0);
}

void PatternGenerator2016::saveStepFile(std::string file, int idx, bool robot)
{
  
  string fullPath;
  if (robot)
  {
    std::list<std::string> names = File::getFullNames(file);
    std::list<std::string>::const_iterator i;
    for (i = names.begin(); i != names.end(); ++i)
    {
      File path(*i, "r", false);
      if (path.exists())
        break;
    }

    fullPath = (i == names.end()) ? file : *i;
  }
  else
  {
    fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + file;
  }
  
  OutMapFile stream(fullPath);
  
  if(stream.exists())
  {
    stream << stepFiles[idx];
    OUTPUT(idText, text, std::string("Saved ") << file);
  }
  else
    OUTPUT(idText, text, std::string("Failed to save ") << file);
}

// To add a new file, add a new member to ENUM StepRequest in WalkRequest.h

void PatternGenerator2016::loadStepFiles()
{
  stepFiles.clear();
  stepFiles.resize(WalkRequest::numOfStepRequests);
  for (int i = 0; i < WalkRequest::numOfStepRequests - WalkRequest::StepRequest::beginScript - 1; i++)
  {
    loadStepFile(WalkRequest::getName((WalkRequest::StepRequest)(WalkRequest::StepRequest::beginScript + 1 + i)), i);
  }
  
}

void PatternGenerator2016::saveStepFiles()
{
  for (int i = 0; i < WalkRequest::numOfStepRequests - WalkRequest::StepRequest::beginScript - 1; i++)
  {
    saveStepFile(WalkRequest::getName((WalkRequest::StepRequest)(WalkRequest::StepRequest::beginScript + 1 + i)), i);
  }
}

MAKE_MODULE(PatternGenerator2016, dortmundWalkingEngine)