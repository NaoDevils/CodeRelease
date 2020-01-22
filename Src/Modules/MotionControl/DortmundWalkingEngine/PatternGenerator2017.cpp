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
 * @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator2017.cpp
 * Generator for foot steps
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#include "PatternGenerator2017.h"
#include <iostream>
#include "Platform/File.h"
#include "Tools/Math/Bspline.h"
#include "Tools/Math/interpolator.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Bezier.h"

using namespace DWE;

#define TRANSITION_NOT_POSSIBLE -1
#define OK 0

#define FOOT_LEN	0.16f

#define LOGGING

#define POLYNOM_DEGREE 4

 // Starts a new foot reset polygon
 // pointcount is the number of points between start and end
#define START_POLYGON(pointcount, start, end) { \
  int counter = 1; \
  int pc = pointcount; \
  Point *polygon = new Point[pc + 2]; \
  polygon[0] = start; \
  polygon[pc + 1] = end;

 // Defines the next point
 // translation is the rate of the full translation from the start to the target
 // height is the rate of the full step height
#define POINT(translation, height_factor, max_height) \
  polygon[counter] = polygon[0] + (polygon[pc + 1] - polygon[0])*translation; \
  polygon[counter].z = polygon[0].z + height_factor*max_height; \
  polygon[counter].ry = polygon[0].ry + height_factor*footPitch; \
  polygon[counter].rx = polygon[0].rx + height_factor*footRoll; \
  counter++;

 // Writes the curve to output 
#define END_POLYGON(output, count, degree) \
  BSpline<Point>::bspline(pc + 1, degree, polygon, output, count); } \


#ifndef WALKING_SIMULATOR
#include "Tools/Math/BHMath.h"

#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

using namespace std;


PatternGenerator2017::PatternGenerator2017() : speedBuffer(Pose2f())
{
  wasReady = false;
  walkHeight = initialStandbyHeight;
  reset();
  baseRot=0;
  loadStepFiles();
  fallDownAngleReductionFactor = Vector2f(1.f, 1.f);

  InMapFile stream("csConverter2019.cfg");
  if (stream.exists())
  {
    stream >> csConverterParams;
  }
}

int PatternGenerator2017::getOptimalPreviewLength()
{
  return std::max((unsigned int)PREVIEW_LENGTH+1, doubleSupportDuration()+singleSupportDuration()+1);
  //return doubleSupportDuration() + singleSupportDuration() + 1;
}

void PatternGenerator2017::setStepLength()
{
  stepsInPreview++;
  // setting direction to current center of feet rotation
  // this is done to remove inaccuracies from adding floats via deltaDirection
  direction = (leftFootPose2f.rotation + rightFootPose2f.rotation) / 2;

  if (currentState == customSteps)
  {
    // in custom steps, translation is directly taken from step definition
    currentExecutedStep = executedStep;
    int frameForNextTwoPhases = executedStep->duration;
    if (executedStep->onFloor[LEFT_FOOT]) // left foot stays
    {
      lastRightFootPose2f = rightFootPose2f;
      robotPose2f = leftFootPose2f;
      robotPose2f.translate(executedStep->footPos[RIGHT_FOOT].translation.x(), 
        -theWalkingEngineParams.footMovement.footYDistance + executedStep->footPos[RIGHT_FOOT].translation.y());
      robotPose2f.rotation += executedStep->footPos[RIGHT_FOOT].rotation;
      rightFootPose2f = robotPose2f;
      rightFootPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
      deltaDirection[firstSingleSupport] = (rightFootPose2f.rotation - lastRightFootPose2f.rotation) / (2 * executedStep->duration);
      deltaDirection[firstDoubleSupport] = 0.f;
      currentMovement.speed = rightFootPose2f - lastRightFootPose2f;
      // bc of our speed y definition (we want continuous speed):
      currentMovement.speed.translation.y() = executedStep->footPos[RIGHT_FOOT].translation.y() / 2.f; 
    }
    else // right foot stays
    {
      lastLeftFootPose2f = leftFootPose2f;
      robotPose2f = rightFootPose2f;
      robotPose2f.translate(executedStep->footPos[LEFT_FOOT].translation.x(),
        theWalkingEngineParams.footMovement.footYDistance + executedStep->footPos[LEFT_FOOT].translation.y());
      robotPose2f.rotation += executedStep->footPos[LEFT_FOOT].rotation;
      leftFootPose2f = robotPose2f;
      leftFootPose2f.translate(0, theWalkingEngineParams.footMovement.footYDistance);
      deltaDirection[secondSingleSupport] = (leftFootPose2f.rotation - lastLeftFootPose2f.rotation) / (2 * executedStep->duration);
      deltaDirection[secondDoubleSupport] = 0.f;
      currentMovement.speed = (leftFootPose2f - lastLeftFootPose2f);
      // bc of our speed y definition (we want continuous speed):
      currentMovement.speed.translation.y() = executedStep->footPos[LEFT_FOOT].translation.y() / 2.f;
    }
    if (!useRealSpeedInCustomSteps)
      currentMovement.speed = Pose2f();
    executedStep++;
    if (executedStep == currentSteps.steps.end())
    {
      frameForNextTwoPhases++; //TODO: assuming a DS length of 1, should call method here
    }
    else
      frameForNextTwoPhases += executedStep->duration;

    // translate speed from stepSpeed to speed per second
    float speedFactor = 1.f / (frameForNextTwoPhases * theFrameInfo.cycleTime);
    currentMovement.speed.translation *= speedFactor;
    currentMovement.speed.rotation *= (speedFactor / 2.f);

    curStepDuration = frameForNextTwoPhases * 2 * theFrameInfo.cycleTime;
    return;
  }
  /*if (theBallchaser.kickType == MotionRequest::dribble)
  {
    return;
  }*/
  // calculating distance for next step
  float stepTime = curStepDuration / 2;
  Pose2f stepSpeed = currentMovement.speed;
  stepSpeed.translation *= stepTime;
  stepSpeed.rotation *= stepTime;
  // since rotation is done in one step
  if ((stepSpeed.rotation >= 0 && currentWalkingPhase == secondSingleSupport) 
    || (stepSpeed.rotation < 0 && currentWalkingPhase == firstSingleSupport))
    stepSpeed.rotation *= 2.f;
  else
    stepSpeed.rotation = 0.f;
  // same for y translation
  if ((stepSpeed.translation.y() >= 0 && currentWalkingPhase == secondSingleSupport)
    || (stepSpeed.translation.y() < 0 && currentWalkingPhase == firstSingleSupport))
    stepSpeed.translation.y() *= 2.f;
  else
    stepSpeed.translation.y() = 0.f;

  // set new foot position and update robot pose (next to new foot position)
  if (currentWalkingPhase == firstSingleSupport) // left foot stays
  {
    lastRightFootPose2f = rightFootPose2f;
    robotPose2f = leftFootPose2f;
    robotPose2f.translate(stepSpeed.translation.x(), -theWalkingEngineParams.footMovement.footYDistance + stepSpeed.translation.y());
    robotPose2f.rotation += stepSpeed.rotation;
    rightFootPose2f = robotPose2f;
    rightFootPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
    deltaDirection[firstSingleSupport] = (rightFootPose2f.rotation - lastRightFootPose2f.rotation) / (2*singleSupportDuration());
    deltaDirection[firstDoubleSupport] = 0.f;
  }
  else // right foot stays
  {
    lastLeftFootPose2f = leftFootPose2f;
    robotPose2f = rightFootPose2f;
    robotPose2f.translate(stepSpeed.translation.x(), theWalkingEngineParams.footMovement.footYDistance + stepSpeed.translation.y());
    robotPose2f.rotation += stepSpeed.rotation;
    leftFootPose2f = robotPose2f;
    leftFootPose2f.translate(0, theWalkingEngineParams.footMovement.footYDistance);
    deltaDirection[secondSingleSupport] = (leftFootPose2f.rotation - lastLeftFootPose2f.rotation) / (2*singleSupportDuration());
    deltaDirection[secondDoubleSupport] = 0.f;
  }

  stateCounter = singleSupportDuration();
}

PatternGenerator2017::~PatternGenerator2017(void)
{
}


inline unsigned int PatternGenerator2017::getPhaseLength(float ratio, float stepDur)
{
  return (int)floor(((ratio*stepDur*(1/theFrameInfo.cycleTime))/2)+0.5f);
}

void PatternGenerator2017::initWalkingPhase()
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

  //stateCounter=singleSupportDuration() - 1;
}

void PatternGenerator2017::transitionToWalk(MovementInformation &moveInf)
{
  currentMovement.speed = Pose2f();
  newMovement = moveInf;
  lastSpeed = Pose2f();
  
  applyAcceleration();
  
  if (currentMovement.speed.translation.y() * currentMovement.speed.rotation < 0)
    currentMovement.speed.translation.y() = 0;
  currentStepFileIndex = -1;

  initWalkingPhase();
  lastStep.footPos[LEFT_FOOT] = Point(leftFootPose2f);
  lastStep.footPos[RIGHT_FOOT] = Point(rightFootPose2f);
  setStepLength();
  sideStepSum[0] = Point();
  sideStepSum[1] = Point();
  currentCustomStep = WalkRequest::none;
  customStepKickInPreview = false;
}

void PatternGenerator2017::updateFallDownReduction() {
  Angle angleX;
  Angle angleY;
  if (useIMUModel)
  {
    angleX = theIMUModel.orientation.x();
    angleY = theIMUModel.orientation.y();
  }
  else
  {
    angleX = theInertialSensorData.angle.x();
    angleY = theInertialSensorData.angle.y();
  }

  fallDownAngleReductionFactor.x() = 1.f;
  fallDownAngleReductionFactor.y() = 1.f;
  if (angleY < (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0] + csConverterParams.legJointBalanceParams.targetAngleY))
  {
    //fallDownAngleReduction.x() = 1.f + fallDownFactor*std::abs((angleY-theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]);
    fallDownAngleReductionFactor.x() = fallDownFactor * std::pow(fallDownExponentialBasis, std::abs((angleY - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]));
  }
  else if (angleY > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1] + csConverterParams.legJointBalanceParams.targetAngleY))
  {
    //fallDownAngleReduction.x() = 1.f + fallDownFactor*std::abs((angleY-theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]);
    fallDownAngleReductionFactor.x() = fallDownFactor * std::pow(fallDownExponentialBasis, std::abs((angleY - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]));
  }

  if (angleX < (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0] + csConverterParams.legJointBalanceParams.targetAngleX))
  {
    //fallDownAngleReduction.y() = 1.f + fallDownFactor*std::abs((angleX-theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]);
    fallDownAngleReductionFactor.y() = fallDownFactor * std::pow(fallDownExponentialBasis, std::abs((angleX - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0]));
  }
  if (angleX > (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1] + csConverterParams.legJointBalanceParams.targetAngleX))
  {
    //fallDownAngleReduction.y() = 1.f + fallDownFactor*std::abs((angleX-theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]);
    fallDownAngleReductionFactor.y() = fallDownFactor * std::pow(fallDownExponentialBasis, std::abs((angleX - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]) / theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1]));
  }
}

bool PatternGenerator2017::updateWalkState()
{
  Point CoMspeed = CoM - lastCoM;
  lastCoM = CoM;
  int oldState = currentState;
  ASSERT(!running || stateCounter > 0);
  stateCounter--;
  static Rangef z_h_limits = Rangef(0.2f, 0.265f);
  if (theFallDownState.state != FallDownState::upright && currentState != State::standby)
  {
    reset();
    currentState = newState = State::standby;
    running = false;
    //walkHeight = z_h_limits.limit(-theRobotModel.soleLeft.translation.z() / 1000);
    return false;
  }
  switch (currentState)
  {
  case State::standby:
    // immediately start transition to walk
    stateCounter = 1;
    running = false;
    if (theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionRequest.motion == MotionRequest::walk)
    {
      currentState = newState = State::goingToReady;
      reset();
      stateCounter = theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
    }
    break;
  case State::goingToReady:
    if (stateCounter == 0)
    {
      currentState = newState = ready;
      curStepDuration = calcDynamicStepDuration(currentMovement.speed.translation.x(), currentMovement.speed.translation.y(), currentMovement.speed.rotation);
      PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams", curStepDuration);
      desiredPreviewLength = getOptimalPreviewLength();
      stateCounter = desiredPreviewLength;
      updatePreview();
      stateCounter = 1;
      walkHeight = z_h_limits.limit(-theRobotModel.soleLeft.translation.z() / 1000);
    }
    if (!(theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionRequest.motion == MotionRequest::walk))
    {
      stateCounter = theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
      currentState = newState = State::goingToStandby;
    }
    break;
  case State::ready:
    stateCounter = 1;
    newMovement.speed.translation = theSpeedRequest.translation/1000.f;
    newMovement.speed.rotation = theSpeedRequest.rotation;
    // if motion command is non-zero-walk, start walking
    if (!isSpeed0() && theFrameInfo.getTimeSince(timeStampLastZeroRequest) > 150)
    {
      currentState = newState = walking;
      currentWalkingPhase = unlimitedDoubleSupport;
      transitionToWalk(newMovement);
    }
    // if motion command is not walk, go to standBy
    if (!(theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionRequest.motion == MotionRequest::walk) &&
      std::abs(CoMspeed.x) < theWalkingEngineParams.walkTransition.stopSpeedThresholdX &&
      std::abs(CoMspeed.y) < theWalkingEngineParams.walkTransition.stopSpeedThresholdY &&
      theInertialSensorData.gyro.norm() < maxGyroSpeedForTransition &&
      localFootPositions.phase == unlimitedDoubleSupport)
    {
      stateCounter = theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
      currentState = newState = State::goingToStandby;
    }
    break;
  case State::walking:
    newMovement.speed.translation = theSpeedRequest.translation / 1000.f;
    newMovement.speed.rotation = theSpeedRequest.rotation;
    // if motion command is zero-walk or not walk, get to ready again
    if (!(theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionSelection.targetMotion == MotionRequest::walk)
      || isSpeed0())
    {
      timeStampLastZeroRequest = theFrameInfo.time;
      newState = State::stopping;
    }
    break;
  case State::stopping:
    newMovement.speed = Pose2f();
    if (currentMovement.speed.translation.y() == 0 &&
      currentMovement.speed.translation.x() == 0 &&
      currentMovement.speed.rotation == 0)
    {
      if (isSpeed0())
      {
        timeStampLastZeroRequest = theFrameInfo.time;
        newState = ready;
      }
      else
        currentState = newState = State::walking;
    }
    break;
  case State::goingToStandby:
    if (stateCounter == 0)
    {
      reset();
      currentState = newState = State::standby;
    }
    break;
  case State::customSteps:
    // transition done when updating walk phase and selecting next executedStep
    break;
  default:
    break;
  }
  
  return oldState != currentState;
}

void PatternGenerator2017::updateWalkPhase()
{
  if (currentState != walking && currentState != stopping && currentState != customSteps)
  {
    currentWalkingPhase = unlimitedDoubleSupport;
    return;
  }
    
  if (stateCounter == 0)
  {
    if (currentStepFileIndex != -1 && executedStep == currentSteps.steps.end())
      transitionFromCustomSteps();
    switch (currentWalkingPhase)
    {
    case unlimitedDoubleSupport:
      stateCounter = 1;
      if (currentState == State::walking)
      {
        reset();
        applyAcceleration();
        if (currentMovement.speed.translation.y() > 0 || currentMovement.speed.rotation > 0)
          currentWalkingPhase = secondSingleSupport;
        else
          currentWalkingPhase = firstSingleSupport;
        setStepLength();
      }
      break;
    case firstSingleSupport:
      if (currentState == customSteps)
      {
        stepsSinceLastCustomStep = 0;
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
      }
      else
      {
        stepsSinceLastCustomStep++;
        currentWalkingPhase = firstDoubleSupport;
        stateCounter = doubleSupportDuration();
        if (newState != currentState)
          currentState = newState;
      }
      break;
    case firstDoubleSupport:
      if (currentState == customSteps)
        setCustomWalkingPhase();
      else
      {
        currentWalkingPhase = secondSingleSupport;
        applyAcceleration();
        if (currentState != customSteps && transitionToCustomSteps())
          currentState = customSteps;
      }
      setStepLength();
      sideStepSum[0] = Point();
      sideStepSum[1] = Point();
      break;
    case secondSingleSupport:
      if (currentState == customSteps)
      {
        stepsSinceLastCustomStep = 0;
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
      }
      else
      {
        stepsSinceLastCustomStep++;
        currentWalkingPhase = secondDoubleSupport;
        stateCounter = doubleSupportDuration();
        if (newState != currentState)
          currentState = newState;
      }
      break;
    case secondDoubleSupport:
      if (currentState == customSteps)
        setCustomWalkingPhase();
      else
      {
        currentWalkingPhase = firstSingleSupport;
        applyAcceleration();
        if (currentState != customSteps && transitionToCustomSteps())
          currentState = customSteps;
      }
      setStepLength();
      sideStepSum[0] = Point();
      sideStepSum[1] = Point(); 
      break;
    default:
      currentWalkingPhase = unlimitedDoubleSupport;
      stateCounter = 1;
      break;
    }
  }
}

Vector2f PatternGenerator2017::prependCustomStep(std::list<PatternGenerator2017::Step>& steps, Vector2f distance, bool onFloorLeft)
{
  // step limits
  Vector2f min(-0.10f, -0.05f);
  Vector2f max(0.10f, 0.05f);
  float innerMin = -0.01f;
  float innerMax = 0.01f;

  if (onFloorLeft) max.y() = innerMax;
  if (!onFloorLeft) min.y() = innerMin;

  float scale = 1.f;

  // clip steps
  if (distance.x() > max.x())
    scale = distance.x() / max.x();
  else if (distance.x() < min.x())
    scale = distance.x() / min.x();
  if (distance.y() > max.y() && distance.y() / max.y() > scale)
    scale = distance.y() / max.y();
  else if (distance.y() < min.y() && distance.y() / min.y() > scale)
    scale = distance.y() / min.y();

  Vector2f distanceLeft(0.f, 0.f);

  distanceLeft = distance - (distance / scale);
  distance /= scale;

  Step step;
  step.duration = 1;
  step.onFloor[0] = true;
  step.onFloor[1] = true;
  step.footPos[0] = Vector2f(0.f, 0.f);
  step.footPos[1] = Vector2f(0.f, 0.f);
  step.swingFootTraj = {};

  steps.push_front(step);

  step.duration = getPhaseLength(1-theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(distance.x(), distance.y(), 0.f));
  step.onFloor[0] = onFloorLeft;
  step.onFloor[1] = !onFloorLeft;
  step.footPos[0] = step.onFloor[0] ? Vector2f(0.f, 0.f) : distance;
  step.footPos[1] = step.onFloor[1] ? Vector2f(0.f, 0.f) : distance;
  step.swingFootTraj = {};

  steps.push_front(step);

  return distanceLeft;
}

bool PatternGenerator2017::prependCustomSteps(std::list<PatternGenerator2017::Step>& steps, Pose2f pose, bool startFootLeft, bool endFootLeft, int maxSteps, const StepsFile &stepsFile)
{
  if (std::abs(pose.rotation) >= 90_deg)
    return false;

  bool footLeft = startFootLeft;

  // if translation or rotation is minimal, remove it from prepend steps to reduce steps
  // TODO: not a clean solution. This tries to only apply front threshold if robot is already in front of ball -> this avoids running sideways into the ball!
  if (std::abs(pose.translation.y()) < 50)
  {
    if (-pose.translation.x() < stepsFile.translationThresholdXFront && pose.translation.x() < stepsFile.translationThresholdXBack)
      pose.translation.x() = 0.f;
  }
  else if (std::abs(pose.translation.x()) < stepsFile.translationThresholdXBack)
    pose.translation.x() = 0.f;
  if (std::abs(pose.translation.y()) < stepsFile.translationThresholdY)
    pose.translation.y() = 0.f;
  if (std::abs(pose.rotation) < stepsFile.rotationThreshold)
    pose.rotation = 0.f;
  
  // is prepend of steps neccessary?
  if (pose.translation.x() != 0.f || pose.translation.y() != 0.f || pose.rotation != 0.f)
  {

    // rotation defines minimum number of steps
    int numLeftSteps = 0, numRightSteps = 0;
    if (pose.rotation > 0)
    {
      numLeftSteps = std::max(numLeftSteps, static_cast<int>(std::ceil(pose.rotation / prependStepsRotMax)));
      numRightSteps = numLeftSteps - 1;
      numRightSteps += !startFootLeft;
    }
    else if (pose.rotation < 0)
    {
      numRightSteps = std::max(numRightSteps, static_cast<int>(std::ceil(-pose.rotation / prependStepsRotMax)));
      numLeftSteps = numRightSteps - 1;
      numLeftSteps += startFootLeft;
    }
    else // if no rotation, you have at least the starting foot step
    {
      numRightSteps += !startFootLeft;
      numLeftSteps += startFootLeft;
    }

    // We want an equal distribution of the x and y translation in each step.
    // Since our coordinate system changes with each step, we cannot use pose.translation/numOfSteps
    // We build a system of linear equations to determine the x and y values of each step:
    // Let x value per step be pose.translation.x/a and y be pose.translation.y/b
    // now we get two equations (example for just two coordinate system changes):
    // a [+a] + a*2*cos(-rotInStep) + b*sin(-rotInStep) = pose.translation.x
    // [b] + b*cos(-rotInStep) - 2*a*sin(-rotInStep) = pose.translation.y
    // a and b at the beginning of the equation depend on the number of steps before the first rotation 
    // and are done in the original coordinate system.
    // The last coordinate system change can be ignored, since it happens after all translations are done.
    // In the implementation the second part of the equation is called a_fromY (b_fromX)
    // We get the matrix A(a,a_fromY,b_fromX,b) and solve it
    // If the solution does not satisfy our constraints (e.g. translation limits)
    // we add steps until it does.
    // TODO: instead of equal distribution of speed, try to smooth acceleration, concept should still work
    //       we just have some coefficient other than 1 in front if term for each step
    //       and while we are at it, keep in mind the current speed
    bool solutionFound = false;
    bool leftRightImbalance = (numLeftSteps != numRightSteps);
    bool startWithYTrans = (pose.translation.y() > 0) == startFootLeft;
    numLeftSteps--;
    numRightSteps--;
    float xPerStep = 0.f, yPerStep = 0.f, rotPerStep = 0.f;
    while (!solutionFound && numLeftSteps + numRightSteps < maxSteps)
    {
      numLeftSteps++;
      numRightSteps++;
      const int numOfRotations = std::max(numLeftSteps, numRightSteps);
      // not defined if numOfRotations = 0, but irrelevant, since for loop below is skipped
      rotPerStep = pose.rotation / numOfRotations; 
      float rotAtStep = rotPerStep;
      // first coordinate system change might be reached after just one step
      float a = 2.f - leftRightImbalance;
      float a_fromY = 0.f;
      // y can only be applied if two steps are used at the start or the first step is in the fitting direction
      float b = startWithYTrans || !leftRightImbalance;
      float b_fromX = 0.f;
      for (int i = 1; i < numOfRotations; i++, rotAtStep += rotPerStep)
      {
        // x always gets applied in both steps -> factor 2!
        a += 2 * cos(-rotAtStep);
        a_fromY += sin(-rotAtStep);
        b += cos(-rotAtStep);
        // x always gets applied in both steps -> factor 2!
        b_fromX -= 2 * sin(-rotAtStep);
      }
      Matrix2f A;
      A << a, a_fromY,
        b_fromX, b;
      Vector2f perStep = A.colPivHouseholderQr().solve(pose.translation);
      xPerStep = perStep.x();
      yPerStep = perStep.y();
      solutionFound = !std::isnan(xPerStep) && !std::isnan(yPerStep) && std::abs(xPerStep) < prependStepsTransMax.x() && std::abs(yPerStep) < prependStepsTransMax.y();
      // edge case: one rotation only at the end -> can lead to wrong solutions since yPerStep can be 0.
      if (numOfRotations == 1 && yPerStep == 0.f && pose.translation.y() != 0.f)
        solutionFound = false;
    }
    if (!solutionFound || numLeftSteps + numRightSteps > maxSteps)
    {
      // OUTPUT_WARNING("module:PatternGenerator2017:prependCustomSteps - Could not find find steps to prepend to custom step");
      return false;
    }
    
    for (int i = 0; i < numLeftSteps + numRightSteps; i++)
    {
      Pose2f footPos;

      footPos.translation.x() = xPerStep;
      footPos.translation.y() = (footLeft == (yPerStep > 0)) ? yPerStep : 0.f;
      footPos.rotation = (footLeft == (rotPerStep > 0)) ? rotPerStep : 0.f;

      Step step;
      step.duration = getPhaseLength(1-theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(xPerStep, yPerStep, rotPerStep));
      step.onFloor[0] = !footLeft;
      step.onFloor[1] = footLeft;
      step.footPos[0] = step.onFloor[0] ? Pose2f() : footPos;
      step.footPos[1] = step.onFloor[1] ? Pose2f() : footPos;
      step.kick = false;
      step.swingFootTraj = {};

      steps.push_back(step);

      step.duration = 1;
      step.onFloor[0] = true;
      step.onFloor[1] = true;
      step.footPos[0] = Pose2f();
      step.footPos[1] = Pose2f();
      step.kick = false;
      step.swingFootTraj = {};

      steps.push_back(step);

      footLeft = !footLeft;
    }
  }
  // another step to 'stop' before the actual custom step execution 
  // TODO: finish this
  // TODO: test, if needed -> slower, but makes for a consistent custom step start point
  if (endFootLeft == footLeft)
  {
    Step step;
    step.duration = getPhaseLength(1-theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(0.f, 0.f, 0.f));
    step.onFloor[0] = !footLeft;
    step.onFloor[1] = footLeft;
    step.footPos[0] = Pose2f();
    step.footPos[1] = Pose2f();
    step.kick = false;
    step.swingFootTraj = {};

    steps.push_back(step);
    step.duration = 1;
    step.onFloor[0] = true;
    step.onFloor[1] = true;
    step.footPos[0] = Pose2f();
    step.footPos[1] = Pose2f();
    step.kick = false;
    step.swingFootTraj = {};

    steps.push_back(step);
  }
  return true;
}

Pose2f PatternGenerator2017::calcKickPose(const StepsFile& steps, const Vector2f& kickTarget)
{
  Vector2f kickTargetRel = Transformation::fieldToRobot(theRobotPoseAfterPreview, kickTarget);
  Pose2f kickPose((kickTargetRel - theBallModelAfterPreview.estimate.position).angle(), theBallModelAfterPreview.estimate.position);
  kickPose.rotation -= steps.kickAngle;
  kickPose.translation /= 1000.f;
  kickPose.translate(-steps.ballOffset);
  return kickPose;
}

bool PatternGenerator2017::isStablePosition()
{
  Angle angleX;
  Angle angleY;
  if (useIMUModel)
  {
    angleX = theIMUModel.orientation.x();
    angleY = theIMUModel.orientation.y();
  }
  else
  {
    angleX = theInertialSensorData.angle.x();
    angleY = theInertialSensorData.angle.y();
  }

  float factor = 1.0;
  MODIFY("fallDownFactor", factor);
  bool xFrontStable = angleY > factor * (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0] + csConverterParams.legJointBalanceParams.targetAngleY);
  bool xBackStable = angleY < factor * (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1] + csConverterParams.legJointBalanceParams.targetAngleY);
  bool yLeftStable = angleX > factor * (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[0] + csConverterParams.legJointBalanceParams.targetAngleX);
  bool yRightStable = angleX < factor * (theWalkingEngineParams.walkTransition.fallDownAngleMinMaxX[1] + csConverterParams.legJointBalanceParams.targetAngleX);

  return xFrontStable && xBackStable && yLeftStable && yRightStable;

  return true;
}

bool PatternGenerator2017::transitionToCustomSteps()
{
  if (stepsInPreview > 1 && useResetPreview)
    return false;
  Vector2f kickTarget = theMotionRequest.kickRequest.kickTarget;

  WalkRequest::StepRequest requestedStep = theMotionSelection.walkRequest.stepRequest;
  if (requestedStep == WalkRequest::any)
  {
    requestedStep = selectCustomStepForKick(kickTarget);
  }
  bool doSafetyStep = false;
  if (useSafetySteps && angleSumBodyTiltBack > angleSumForSafetyStepBack && theFrameInfo.getTimeSince(timeWhenLastSafetyStepTriggered) > 3000)
  {
    requestedStep = WalkRequest::safetyStepBack;
    doSafetyStep = true;
    angleSumBodyTiltBack = 0;
    angleSumBodyTiltFront = 0;
    //SystemCall::playSound("doh.wav");
    timeWhenLastSafetyStepTriggered = theFrameInfo.time;
  }
  else if (useSafetySteps && angleSumBodyTiltFront > angleSumForSafetyStepFront && theFrameInfo.getTimeSince(timeWhenLastSafetyStepTriggered) > 3000)
  {
    requestedStep = WalkRequest::safetyStepFront;
    doSafetyStep = true;
    angleSumBodyTiltBack = 0;
    angleSumBodyTiltFront = 0;
    //SystemCall::playSound("doh.wav");
    timeWhenLastSafetyStepTriggered = theFrameInfo.time;
  }

  if (currentState == walking && requestedStep >=
      WalkRequest::StepRequest::beginScript && stepsSinceLastCustomStep > stepsBetweenCustomSteps && isStablePosition())
  {
    int idx = (int)(requestedStep - WalkRequest::StepRequest::beginScript - 1);

    // if type is ::any, currentSteps are already set
    if (theMotionSelection.walkRequest.stepRequest != WalkRequest::any || doSafetyStep)
    {
      // copy steps, may be modified according to ball position / mirror afterwards
      currentSteps = stepFiles[idx];
      if (theMotionRequest.kickRequest.mirror || (doSafetyStep && currentWalkingPhase == firstSingleSupport))
        currentSteps.mirror();
    }

    // initialize custom step
    int lastPhaseLength = doubleSupportDuration();

    if ((currentWalkingPhase == secondSingleSupport &&
        currentSteps.steps.begin()->onFloor[1]) ||
        (currentWalkingPhase == firstSingleSupport &&
         currentSteps.steps.begin()->onFloor[0]))
    {
      stepsSinceLastCustomStep = 0;
      executedStep = currentSteps.steps.begin();

      setCustomWalkingPhase();
      currentStepFileIndex = idx;
      currentCustomStep = requestedStep;
      // set preview to be long enough for custom steps (double supp + single supp)
      int maxPhaseLength = desiredPreviewLength;
      for (const auto& step : currentSteps.steps)
      {
        if (lastPhaseLength + step.duration > maxPhaseLength)
        {
          maxPhaseLength = lastPhaseLength + step.duration;
        }
        lastPhaseLength = step.duration;
      }
      if (maxPhaseLength > desiredPreviewLength)
        desiredPreviewLength = maxPhaseLength + 1;
    }
  }
  
  return currentStepFileIndex != -1;
}

WalkRequest::StepRequest PatternGenerator2017::selectCustomStepForKick(const Vector2f& kickTarget)
{
  WalkRequest::StepRequest bestKick = WalkRequest::StepRequest::none;
  float bestScore = INFINITY;
  Vector2f kickTargetRel = Transformation::fieldToRobot(theRobotPoseAfterPreview, kickTarget);
  const std::vector<WalkRequest::StepRequest> &kickList =
       theRoleSymbols.role == BehaviorData::keeper ? activeGoalieKicks : activeKicks;
  for (WalkRequest::StepRequest kick : kickList)
  {
    int idx = (int)(kick - WalkRequest::StepRequest::beginScript - 1);
    if (!stepFiles[idx].isApplicable(kickTargetRel.norm() / 1000.f)) continue;

    for (int i = 0; i < 2; i++)
    {
      StepsFile steps = stepFiles[idx];
      if (i)
        steps.mirror();
      std::list<PatternGenerator2017::Step> prependedSteps;
      Pose2f kickPose(calcKickPose(steps, kickTarget));
      bool prependSuccessful = 
        prependCustomSteps(prependedSteps, kickPose, currentWalkingPhase == secondSingleSupport, steps.steps.front().onFloor[LEFT_FOOT], maxPrependedSteps, steps);
      steps.steps.insert(steps.steps.begin(), prependedSteps.begin(), prependedSteps.end());

      kickPose = calcKickPose(steps, kickTarget);
      // TODO: better measurement or scrap it completely since maxPrependedSteps tells the story..
      // maybe use the distance and rotation of the prepended steps
      // TODO: distinguish between long and short kicks
      float score = getScore(prependedSteps);

      if (score < bestScore && prependSuccessful)
      {
        bestKick = kick;
        bestScore = score;
        currentSteps = steps;
      }
    }
  }

  return bestKick;
}

float PatternGenerator2017::getScore(const std::list<PatternGenerator2017::Step>& steps)
{
  float score = 0.f;
  for (const PatternGenerator2017::Step& step : steps)
  {
    score += 1.f;
    score += (step.footPos[LEFT_FOOT].rotation / prependStepsRotMax) * stepRotationWeight;
    score += (step.footPos[RIGHT_FOOT].rotation / prependStepsRotMax) * stepRotationWeight;
  }
  return score;
}

void PatternGenerator2017::transitionFromCustomSteps()
{
  currentState = walking;
  currentStepFileIndex = -1;
  currentCustomStep = WalkRequest::StepRequest::none;
  customStepKickInPreview = false;
  stepsSinceLastCustomStep = 0;
}

float PatternGenerator2017::calcDynamicStepDuration(float speedX, float speedY, float speedR)
{
  // TODO: acceleration dependent on step length, step duration dependent on step length. What about dep on acc?
  PLOT("module:PatternGenerator2017:stabilityErrorX", theWalkingInfo.stabilityError.x());
  PLOT("module:PatternGenerator2017:stabilityErrorY", theWalkingInfo.stabilityError.y());
  PLOT("module:PatternGenerator2017:stabilityError", theWalkingInfo.stabilityError.norm());
  bool robotStable = sensorErrorMinForStepCorrection > theWalkingInfo.stabilityError.norm();
  if (robotStable)
  {
    if (theWalkingEngineParams.footMovement.minStepDuration != theWalkingEngineParams.footMovement.maxStepDuration)
    {
      if (running)
      {
        float xLimit = speedX > 0 ? theWalkingEngineParams.speedLimits.xForward / 1000.f : theWalkingEngineParams.speedLimits.xBackward / 1000.f;
        float yLimit = theWalkingEngineParams.speedLimits.y * 2.f / 1000.f;
        float rLimit = theWalkingEngineParams.speedLimits.r * 1.5f;

        float factorX = std::abs(xLimit - std::abs(speedX)) / xLimit;
        float factorY = std::abs(yLimit - std::abs(speedY)) / yLimit;
        float factorR = std::abs(rLimit - std::abs(speedR)) / rLimit;
        
        float distToMax = std::min(factorX, std::min(factorR, factorY));
        float result = distToMax * theWalkingEngineParams.footMovement.maxStepDuration + (1.f - distToMax) * theWalkingEngineParams.footMovement.minStepDuration;
        // duration of first step of side walk can be adjusted in parameters, the faster we walk sideways, the more the parameter weighs in
        if (theWalkingEngineParams.footMovement.leadingSideStepSpeedUp > 0.f && std::abs(currentMovement.speed.translation.y()) > 0.01f)
        {
          float ySpeedPercentage = std::abs(currentMovement.speed.translation.y()) / (theWalkingEngineParams.speedLimits.y / 1000.f);
          // 1.f means no speed up, range is from 1.f to parameter
          float firstLegSpeedUp = 1.f * (1.f - ySpeedPercentage) + theWalkingEngineParams.footMovement.leadingSideStepSpeedUp * ySpeedPercentage;
          if (currentMovement.speed.translation.y() > 0 && currentWalkingPhase == WalkingPhase::secondSingleSupport)
            result /= firstLegSpeedUp;
          if (currentMovement.speed.translation.y() < 0 && currentWalkingPhase == WalkingPhase::firstSingleSupport)
            result /= firstLegSpeedUp;
        }
        return result;

      }
      else
      {
        return theWalkingEngineParams.footMovement.maxStepDuration; //1.f;
      }
    }
    else
      return theWalkingEngineParams.footMovement.minStepDuration;
  }
  else
  {
    // TODO: adjust speed
    return std::min(absoluteMaxStepDuration, absoluteMinStepDuration + theWalkingInfo.stabilityError.norm());
  }
}

bool PatternGenerator2017::isWalking()
{
  return !(currentState==ready || currentState==standby);
}

void PatternGenerator2017::applyAcceleration()
{
  PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams", curStepDuration);

  updateFallDownReduction();

  lastSpeed = currentMovement.speed;
  /************** rotation ****************/
  // apply rotation speed if feet were not rotated within boundaries
  float maxStepAccR = theWalkingEngineParams.acceleration.maxAccR * curStepDuration;
  const bool sameDirection = !(newMovement.speed.rotation*currentMovement.speed.rotation < 0);
  const bool feetParallel = std::abs(leftFootPose2f.rotation - rightFootPose2f.rotation) < 0.001f;
  const bool deceleration = std::abs(newMovement.speed.rotation) < std::abs(currentMovement.speed.rotation) || !sameDirection;
  // deceleration -> always full up to 0
  if (feetParallel)
  {
    if (deceleration)
    {
      // same direction
      if (sameDirection)
        currentMovement.speed.rotation = newMovement.speed.rotation;
      else
      {
        // if movement is minimal, direction change is allowed!
        if (currentMovement.speed.rotation > 0.f)
          currentMovement.speed.rotation = std::min(0.f,
            std::max(currentMovement.speed.rotation - maxStepAccR, static_cast<float>(newMovement.speed.rotation)));
        else
          currentMovement.speed.rotation = std::max(0.f,
            std::min(currentMovement.speed.rotation + maxStepAccR, static_cast<float>(newMovement.speed.rotation)));
      }
    }
    // acceleration r (here, the new and current speed direction is the same and new speed has higher absolute)
    else
    {
      float maxSpeedR = (newMovement.speed.translation.x() == 0.f && newMovement.speed.translation.y() == 0.f) ?
        theWalkingEngineParams.speedLimits.rOnly : theWalkingEngineParams.speedLimits.r;
      if (newMovement.speed.rotation < 0) // right
        currentMovement.speed.rotation = std::max<float>(std::max(static_cast<float>(newMovement.speed.rotation), currentMovement.speed.rotation - maxStepAccR),
          -maxSpeedR);
      else // left
        currentMovement.speed.rotation = std::min<float>(std::min(static_cast<float>(newMovement.speed.rotation), currentMovement.speed.rotation + maxStepAccR),
          maxSpeedR);
    }
  }
  
  /************** x ****************/
  float maxStepAccX = newMovement.speed.translation.x() > 0 ? theWalkingEngineParams.acceleration.maxAccXForward * curStepDuration / 2 : theWalkingEngineParams.acceleration.maxAccXBackward * curStepDuration / 2;
  const bool sameDirectionX = newMovement.speed.translation.x()*currentMovement.speed.translation.x() >= 0;
  // deceleration x -> always full up to 0
  if (std::abs(newMovement.speed.translation.x()) < std::abs(currentMovement.speed.translation.x()) || !sameDirectionX)
  {
    if (sameDirectionX)
      currentMovement.speed.translation.x() = newMovement.speed.translation.x();
    else
    {
      // if movement is minimal, direction change is allowed!
      if (currentMovement.speed.translation.x() > 0.f)
        currentMovement.speed.translation.x() = std::min(0.f, 
          std::max(currentMovement.speed.translation.x() - maxStepAccX, newMovement.speed.translation.x()));
      else
        currentMovement.speed.translation.x() = std::max(0.f, 
          std::min(currentMovement.speed.translation.x() + maxStepAccX, newMovement.speed.translation.x()));
    }
  }
  // acceleration x (here, the new and current speed direction is the same and new speed has higher absolute)
  else
  {
    float maxSpeedXForward = (std::abs(newMovement.speed.translation.y()) > 20.f) ? theWalkingEngineParams.speedLimits.xForwardOmni : theWalkingEngineParams.speedLimits.xForward;
    // slow down in Penalty Shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout) {
      maxSpeedXForward *= penaltyShootoutSlowDownFactor;
    }
    // slow down when hot
    if (theRobotHealth.maxJointTemperature > slowDownTemperature) {
      maxSpeedXForward /= theRobotHealth.maxJointTemperature / slowDownTemperature;
    }

    float maxSpeedXBackward = theWalkingEngineParams.speedLimits.xBackward;    
    // slow down in Penalty Shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout) {
      maxSpeedXBackward *= penaltyShootoutSlowDownFactor;
    }
    // slow down when hot
    if (theRobotHealth.maxJointTemperature > slowDownTemperature) {
      maxSpeedXBackward /= theRobotHealth.maxJointTemperature / slowDownTemperature;
    }

    maxSpeedXForward /= std::max<float>(fallDownAngleReductionFactor.x(), fallDownAngleReductionFactor.y());
    maxSpeedXBackward /=  std::max<float>(fallDownAngleReductionFactor.x(), fallDownAngleReductionFactor.y());

    if (newMovement.speed.translation.x() < 0) // backward
      currentMovement.speed.translation.x() = std::max(std::max(newMovement.speed.translation.x(), currentMovement.speed.translation.x() - maxStepAccX),
        -maxSpeedXBackward / 1000.f);
    else // forward
      currentMovement.speed.translation.x() = std::min(std::min(newMovement.speed.translation.x(), currentMovement.speed.translation.x() + maxStepAccX), 
        maxSpeedXForward / 1000.f);
  }

  /************** y ****************/
  // no acceleration in y at start if rotation is in different direction!
  if (currentWalkingPhase == unlimitedDoubleSupport &&
    currentMovement.speed.rotation * newMovement.speed.translation.y() < 0)
    return;
  float maxStepAccY = theWalkingEngineParams.acceleration.maxAccY * curStepDuration;
  float maxSpeedY = theWalkingEngineParams.speedLimits.y;
  // slow down in Penalty Shootout
  if (Global::getSettings().gameMode == Settings::penaltyShootout) {
    maxSpeedY *= penaltyShootoutSlowDownFactor;
  }
  // slow down when hot
  if (theRobotHealth.maxJointTemperature > slowDownTemperature) {
    maxSpeedY /= theRobotHealth.maxJointTemperature / slowDownTemperature;
  }

  maxSpeedY /= std::max<float>(fallDownAngleReductionFactor.x(), fallDownAngleReductionFactor.y());

  const bool sameDirectionY = newMovement.speed.translation.y()*currentMovement.speed.translation.y() >= 0;
  // if feet were not parallel, last support foot has to have a bigger than default y distance to the robotpose
  // (see setStepLength : new step is created like this: 
  // support foot -> translate with speed + footYDistance -> rotate -> translate footYDistance
  bool lastBaseFootRight = (currentWalkingPhase == firstSingleSupport);
  Pose2f lastBaseFoot = lastBaseFootRight ? rightFootPose2f : leftFootPose2f;
  Pose2f lastPose2f = robotPose2f;
  lastPose2f.rotation = lastBaseFoot.rotation;
  Pose2f poseDifference = lastPose2f - lastBaseFoot;
  const bool feetParallelY = std::abs(std::abs(poseDifference.translation.y()) - theWalkingEngineParams.footMovement.footYDistance) < 0.001f;
  const bool yStepPossible = (currentWalkingPhase == firstSingleSupport && newMovement.speed.translation.y() <= 0) ||
    (currentWalkingPhase == secondSingleSupport && newMovement.speed.translation.y() >= 0);
  if (feetParallelY && (yStepPossible || !sameDirectionY))
  {
    // deceleration y -> always full up to 0
    if (!yStepPossible && !sameDirectionY)
      currentMovement.speed.translation.y() = 0.f;
    else if (std::abs(newMovement.speed.translation.y()) < std::abs(currentMovement.speed.translation.y()) || !sameDirectionY)
    {
      if (sameDirectionY)
        currentMovement.speed.translation.y() = newMovement.speed.translation.y();
      else
      {
        // if movement is minimal, direction change is allowed!
        if (currentMovement.speed.translation.y() > 0.f)
          currentMovement.speed.translation.y() = std::min(0.f, 
            std::max(currentMovement.speed.translation.y() - maxStepAccY, 
              std::max(newMovement.speed.translation.y(), -maxSpeedY /1000.f)));
        else
          currentMovement.speed.translation.y() = std::max(0.f, 
            std::min(currentMovement.speed.translation.y() + maxStepAccY, 
              std::min(newMovement.speed.translation.y(), maxSpeedY /1000.f)));
      }
    }
    // acceleration y (here, the new and current speed direction is the same and new speed has higher absolute)
    else
    {
      if (newMovement.speed.translation.y() < 0) // right
        currentMovement.speed.translation.y() = std::max(std::max(newMovement.speed.translation.y(), currentMovement.speed.translation.y() - maxStepAccY),
          -maxSpeedY / 1000.f);
      else // left
        currentMovement.speed.translation.y() = std::min(std::min(newMovement.speed.translation.y(), currentMovement.speed.translation.y() + maxStepAccY),
          maxSpeedY / 1000.f);
    }
  }
  // set step duration based on new speed
  curStepDuration = calcDynamicStepDuration(currentMovement.speed.translation.x(), currentMovement.speed.translation.y(), currentMovement.speed.rotation);
  
}

void PatternGenerator2017::setCustomWalkingPhase()
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
  stateCounter = executedStep->duration;
}

void PatternGenerator2017::reset()
{
  robotPose2f = Pose2f();
  robotPose2fAfterCurrentStep = Pose2f();
  direction=0.f;
  for (int i = 0; i < 5; i++)
    deltaDirection[i] = 0.f;
  desiredPreviewLength=1;
  currentPreviewLength = 0; 
  running=false;
  currentMovement.speed = Pose2f();
  lastSpeed = Pose2f();
  newMovement.speed = Pose2f();
  curStepDuration = calcDynamicStepDuration(currentMovement.speed.translation.x(), currentMovement.speed.translation.y(), currentMovement.speed.rotation);
  PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams", curStepDuration);
  currentCustomStep = WalkRequest::none;
  customStepKickInPreview = false;
  currentWalkingPhase = unlimitedDoubleSupport;
  currentTimeStamp=0;
  currentStepFileIndex = -1;
  lastStep.onFloor[0] = lastStep.onFloor[1] = false;
  leftFootPose2f = lastLeftFootPose2f = Pose2f(0, 0, theWalkingEngineParams.footMovement.footYDistance);
  rightFootPose2f = lastRightFootPose2f = Pose2f(0, 0, -theWalkingEngineParams.footMovement.footYDistance);
  appliedReactiveStep[LEFT_FOOT] = 0;
  appliedReactiveStep[RIGHT_FOOT] = 0;
  localSteps.reset();
  stepsInPreview = 0;

  //ref zmp
  lastspdx = 0;
  lpxss = 0;
  plotZMP = 0;
  lastZMPRCS = ZMP(0, 0);
  localRefZMP2018.zmpWCS.clear();
  localRefZMP2018.zmpRCS.clear();
  plannedFootSteps.reset();
  currentFootStepTrajectory.clear();
}

StepData PatternGenerator2017::getNextStep()
{
  Footposition step;
  //default values
  step.footPos[LEFT_FOOT] = Point();
  step.footPos[RIGHT_FOOT] = Point();
  step.onFloor[LEFT_FOOT] = step.onFloor[RIGHT_FOOT] = true;
  step.direction = 0.f;
  step.footPos[RIGHT_FOOT].z = step.footPos[LEFT_FOOT].z = -walkHeight;
  step.footPos[RIGHT_FOOT].y = -theWalkingEngineParams.footMovement.footYDistance;
  step.footPos[LEFT_FOOT].y = theWalkingEngineParams.footMovement.footYDistance;
  step.footPos[LEFT_FOOT].r = baseRot;
  step.footPos[RIGHT_FOOT].r = -baseRot;

  // #ifndef WALKING_SIMULATOR
  float startZ = walkHeight;
  float factor = 0;
  int crouchingLength = theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
  
  switch(currentState)
  {
    case ready:
      //currentWalkingPhase=unlimitedDoubleSupport;
      addStep();
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
      break;
      
    case stopping:
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
        step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z =
          (1- factor) * (-startZ) + factor * (-theFLIPMParameter.paramsX.z_h + CoM.z);
        step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
        step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
        step.footPos[LEFT_FOOT].r=baseRot;
        step.footPos[RIGHT_FOOT].r=-baseRot;
      }
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
        step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z =
          (1 - factor) * (-startZ) + factor * (-theFLIPMParameter.paramsX.z_h + CoM.z);
        step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footMovement.footYDistance;
        step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footMovement.footYDistance;
        step.footPos[LEFT_FOOT].r=baseRot;
        step.footPos[RIGHT_FOOT].r=-baseRot;
      }
      break;
      
    case walking:
    case customSteps:
      break;
      
      
    default:
      break;
  }
  step.phase = unlimitedDoubleSupport;
  return step;
}

void PatternGenerator2017::updatePreview()
{
  if (running && desiredPreviewLength == currentPreviewLength)
    addStep();

  while (desiredPreviewLength > currentPreviewLength) // Add more steps to get a higher preview
  {
    addStep();
    currentPreviewLength++;
    if (desiredPreviewLength > currentPreviewLength)
      stateCounter--;
    if (running && desiredPreviewLength > currentPreviewLength)
      updateWalkPhase();
    if (desiredPreviewLength == currentPreviewLength)
      running = true;
  }
  if (stateCounter == 0)
    stateCounter = 1;
  if (currentPreviewLength < desiredPreviewLength) // Skip this frame, cause we need a lower preview
  {
    currentPreviewLength--;
  }
}

unsigned int PatternGenerator2017::singleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (!currentExecutedStep->onFloor[LEFT_FOOT] || !currentExecutedStep->onFloor[RIGHT_FOOT])
      return currentExecutedStep->duration;
    else if (std::next(currentExecutedStep) !=
      currentSteps.steps.end())
      return std::next(currentExecutedStep)->duration;
  }
  return getPhaseLength(1-theWalkingEngineParams.footMovement.doubleSupportRatio, curStepDuration);
}

unsigned int PatternGenerator2017::doubleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (currentExecutedStep->onFloor[LEFT_FOOT] && currentExecutedStep->onFloor[RIGHT_FOOT])
      return currentExecutedStep->duration;
    else if (std::next(currentExecutedStep) !=
      currentSteps.steps.end())
      return std::next(currentExecutedStep)->duration;
  }
  
  return getPhaseLength(theWalkingEngineParams.footMovement.doubleSupportRatio, curStepDuration);
}


void PatternGenerator2017::addStep()
{
  PLOT("module:PatternGenerator2017:curStepDurationAddStep", curStepDuration);
  Footposition newStep;
  newStep.stepsSinceCustomStep = stepsSinceLastCustomStep;

  newStep.footPos[LEFT_FOOT] = Point(leftFootPose2f);
  newStep.footPos[RIGHT_FOOT] = Point(rightFootPose2f);
  newStep.stepDurationInSec = curStepDuration;
  newStep.onFloor[LEFT_FOOT]=(currentWalkingPhase != secondSingleSupport);
  newStep.onFloor[RIGHT_FOOT]= (currentWalkingPhase != firstSingleSupport);
  
  newStep.footPos[LEFT_FOOT] += sideStepSum[LEFT_FOOT];
  newStep.footPos[RIGHT_FOOT] += sideStepSum[RIGHT_FOOT];
  
  /////////////////////////////////////////////////////////////////////////////
  /* special swing foot trajectory offset for custom steps */
  

  /////////////////////////////////////////////////////////////////////////////
  
  for (int i=0; i<2; i++)
  {
    ASSERT(newStep.footPos[i] == newStep.footPos[i]);
    
    // This makes boom if a foot on the floor is repositioned by more than 1 mm
    // Check transitions from phase to phase including rotation, also custom
    // steps. Same for rotation.
    {
      // TODO: fix this crash
      ASSERT(!(lastStep.onFloor[i] && newStep.onFloor[i]) || (lastStep.footPos[i].euklidDistance3D(newStep.footPos[i]-appliedReactiveStep[i]) < 0.001 && abs(lastStep.footPos[i].r - newStep.footPos[i].r) < 0.01));
    }
    
  }
  
  newStep.direction=direction;
  newStep.customStep = currentCustomStep;
  newStep.inKick = currentStepFileIndex != -1 && currentExecutedStep->kick;
  if (currentStepFileIndex != -1 && currentSteps.kickHackDuration > 0)
  {
    newStep.timeUntilKickHack = currentSteps.timeUntilKickHack;
    newStep.kickHackDuration = currentSteps.kickHackDuration;
    newStep.kickHackKneeAngle = currentSteps.kickHackKneeAngle;
  }
  newStep.phase=currentWalkingPhase;
  newStep.singleSupportDurationInFrames = singleSupportDuration();
  newStep.doubleSupportDurationInFrames = doubleSupportDuration();
  newStep.frameInPhase=0;
  if (currentWalkingPhase == firstSingleSupport || currentWalkingPhase == secondSingleSupport)
  {
    newStep.frameInPhase = newStep.singleSupportDurationInFrames - stateCounter;
  }
  if (currentWalkingPhase == firstDoubleSupport || currentWalkingPhase == secondDoubleSupport)
  {
    newStep.frameInPhase = newStep.doubleSupportDurationInFrames - stateCounter;
  }

  

  if (currentStepFileIndex != -1 && currentExecutedStep->kick && static_cast<int>(newStep.frameInPhase) > currentExecutedStep->duration / 2)
  {
    customStepKickInPreview = true;
  }

  newStep.timestamp=currentTimeStamp; // time is the frame number when it is executed after preview
  currentTimeStamp++;
  

  ASSERT(newStep.frameInPhase < newStep.singleSupportDurationInFrames + newStep.doubleSupportDurationInFrames);
  
  newStep.speed=currentMovement.speed;
  newStep.customStepRunning = (currentStepFileIndex != -1);
  // for ref zmp
  addRefZMP(newStep);
  newStep.lpxss = lpxss;
  newStep.zmp = zmp;
  newStep.lastZMPRCS = lastZMPRCS;
  // add to buffer w/o custom step trajectory offsets
  localSteps.addStep(newStep);

  // add swing foot trajectory from custom steps to foot position
  if (currentStepFileIndex != -1 && !newStep.onFloor[LEFT_FOOT] && currentExecutedStep->spline.size() > newStep.frameInPhase)
  {
    Point offset = currentExecutedStep->spline[newStep.frameInPhase];
    offset.rotate2D(leftFootPose2f.rotation);
    newStep.footPos[LEFT_FOOT] += offset;
  }
  if (currentStepFileIndex != -1 && !newStep.onFloor[RIGHT_FOOT] && currentExecutedStep->spline.size() > newStep.frameInPhase)
  {
    Point offset = currentExecutedStep->spline[newStep.frameInPhase];
    offset.rotate2D(rightFootPose2f.rotation);
    newStep.footPos[RIGHT_FOOT] += offset;
  }

  bool isDoubleSupport = 
    (newStep.phase == unlimitedDoubleSupport || newStep.phase == firstDoubleSupport || newStep.phase == secondDoubleSupport);
  if (isDoubleSupport && plannedFootSteps.getNumOfSteps() > 1 && 
     (plannedFootSteps.getStep(plannedFootSteps.getNumOfSteps()-1).phase == firstSingleSupport || plannedFootSteps.getStep(plannedFootSteps.getNumOfSteps()-1).phase == secondSingleSupport))
    createFootStepTrajectory();
  //newStep.lastSpeed = lastSpeed;
  plannedFootSteps.addStep(newStep);
  
  bool ok = true;
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
  
  LOG("PatternGenerator2017_addStep", "step.footPos[0].x", newStep.footPos[0].x);
  LOG("PatternGenerator2017_addStep", "step.footPos[0].y", newStep.footPos[0].y);
  LOG("PatternGenerator2017_addStep", "step.footPos[0].z", newStep.footPos[0].z);
  LOG("PatternGenerator2017_addStep", "step.footPos[0].r", newStep.footPos[0].r);
  LOG("PatternGenerator2017_addStep", "step.footPos[1].x", newStep.footPos[1].x);
  LOG("PatternGenerator2017_addStep", "step.footPos[1].y", newStep.footPos[1].y);
  LOG("PatternGenerator2017_addStep", "step.footPos[1].z", newStep.footPos[1].z);
  LOG("PatternGenerator2017_addStep", "step.footPos[1].r", newStep.footPos[1].r);
  LOG("PatternGenerator2017_addStep", "step.direction", newStep.direction);
  LOG("PatternGenerator2017_addStep", "timestamp", newStep.timestamp);

  for (int i = 0; i < 2; i++)
  {
    if (newStep.onFloor[i])
      lastStep.footPos[i] = newStep.footPos[i];
    lastStep.onFloor[i] = newStep.onFloor[i];
  }
}

void PatternGenerator2017::createFootStepTrajectory()
{
  int numOfSteps = plannedFootSteps.getNumOfSteps();
  if (numOfSteps == 0)
  {
    OUTPUT_ERROR("module:PatternGenerator2017:createFootStepTrajectory - No footsteps!" << currentState << "-" << currentWalkingPhase);
  }
  Footposition curStep = plannedFootSteps.getStep(numOfSteps-1);
  int footNum = (curStep.phase == firstSingleSupport);

  float yFac = std::abs(curStep.speed.y * 1000) / theWalkingEngineParams.speedLimits.y;
  float xStepHeight = curStep.speed.x > 0 ? theWalkingEngineParams.footMovement.stepHeight[0] : theWalkingEngineParams.footMovement.stepHeight[1];
  float walkStepHeight = (1 - yFac) * xStepHeight + yFac * theWalkingEngineParams.footMovement.stepHeight[2];

  float footPitch = theWalkingEngineParams.footMovement.footPitch * sgn(curStep.speed.x);
  float footRoll = theWalkingEngineParams.footMovement.footRoll * sgn(curStep.speed.y);

  // new stuff
  currentFootStepTrajectory.resize(curStep.singleSupportDurationInFrames);
  const int noControlPoints = 7;
  std::vector<Point> controlVector(noControlPoints);
  controlVector[0] = Point();
  for (int i = 1; i < noControlPoints-1; i++)
  {
    controlVector[i].x = theWalkingEngineParams.footMovement.forwardPolygon[i-1];
    controlVector[i].y = theWalkingEngineParams.footMovement.sideStepPolygon[i-1];
    controlVector[i].z = theWalkingEngineParams.footMovement.heightPolygon[i-1];
    controlVector[i].r = theWalkingEngineParams.footMovement.rotPolygon[i - 1];
  }
  controlVector[noControlPoints-1] = Point(1.f,1.f,0.f,1.f);
  std::vector<Point> offsetSpline(curStep.singleSupportDurationInFrames);
  BSpline<Point>::bspline(noControlPoints-1, 3, &(controlVector[0]), &(offsetSpline[0]), curStep.singleSupportDurationInFrames);
  Point offsetForThisFrame;

  if (footNum == LEFT_FOOT)
  {
    Pose2f footDiff = Pose2f(curStep.footPos[LEFT_FOOT].r, curStep.footPos[LEFT_FOOT].x,curStep.footPos[LEFT_FOOT].y) - lastLeftFootPose2f;
    Pose2f footDiffRelative = footDiff;
    footDiffRelative.rotate(-lastLeftFootPose2f.rotation);
    for (unsigned int i = 0; i < curStep.singleSupportDurationInFrames; i++)
    {
      offsetForThisFrame.x = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].x * footDiffRelative.translation.x();
      offsetForThisFrame.y = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].y * footDiffRelative.translation.y();
      offsetForThisFrame.z = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * walkStepHeight;
      offsetForThisFrame.r = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].r * footDiff.rotation;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].r = lastLeftFootPose2f.rotation + offsetForThisFrame.r;
      offsetForThisFrame.rotate2D(lastLeftFootPose2f.rotation);
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].x = lastLeftFootPose2f.translation.x() + offsetForThisFrame.x;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].y = lastLeftFootPose2f.translation.y() + offsetForThisFrame.y;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].z = offsetForThisFrame.z; 
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].rx = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * footRoll;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].ry = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * footPitch;
    }
  }
  if (footNum == RIGHT_FOOT)
  {
    Pose2f footDiff = Pose2f(curStep.footPos[RIGHT_FOOT].r, curStep.footPos[RIGHT_FOOT].x, curStep.footPos[RIGHT_FOOT].y) - lastRightFootPose2f;
    Pose2f footDiffRelative = footDiff;
    footDiffRelative.rotate(-lastRightFootPose2f.rotation);
    for (unsigned int i = 0; i < curStep.singleSupportDurationInFrames; i++)
    {
      offsetForThisFrame.x = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].x * footDiffRelative.translation.x();
      offsetForThisFrame.y = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].y * footDiffRelative.translation.y();
      offsetForThisFrame.z = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * walkStepHeight;
      offsetForThisFrame.r = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].r * footDiff.rotation;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].r = lastRightFootPose2f.rotation + offsetForThisFrame.r;
      offsetForThisFrame.rotate2D(lastRightFootPose2f.rotation);
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].x = lastRightFootPose2f.translation.x() + offsetForThisFrame.x;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].y = lastRightFootPose2f.translation.y() + offsetForThisFrame.y;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].z = offsetForThisFrame.z;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].rx = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * footRoll;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].ry = offsetSpline[curStep.singleSupportDurationInFrames - i - 1].z * footPitch;
    }
  }
  return;

  // old stuff
  int len = curStep.singleSupportDurationInFrames;
  currentFootStepTrajectory.resize(len);
  for (int i = 0; i < len; i++)
    currentFootStepTrajectory[i] = plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum];
  Point polygonEnd = curStep.footPos[footNum];
  Point* output = &(currentFootStepTrajectory[0]);

  // This makes boom if a single support phase cannot be planned.
  // Possible reason: step longer (duration) than preview
  //ASSERT(++++phases.begin() != phases.end());
  
  Point polygonStart = Point((footNum == LEFT_FOOT) ? lastLeftFootPose2f : lastRightFootPose2f);
  {
    float endingFactor = 1.f;
    float footPitch = theWalkingEngineParams.footMovement.footPitch * sgn(curStep.speed.x);
    float footRoll = theWalkingEngineParams.footMovement.footRoll * sgn(curStep.speed.y);
    
    START_POLYGON(5, polygonStart, polygonEnd);
    POINT(theWalkingEngineParams.footMovement.forwardPolygon[0], theWalkingEngineParams.footMovement.heightPolygon[0], endingFactor * walkStepHeight);
    POINT(theWalkingEngineParams.footMovement.forwardPolygon[1], theWalkingEngineParams.footMovement.heightPolygon[1], endingFactor * walkStepHeight);
    POINT(theWalkingEngineParams.footMovement.forwardPolygon[2], theWalkingEngineParams.footMovement.heightPolygon[2], endingFactor * walkStepHeight);
    POINT(theWalkingEngineParams.footMovement.forwardPolygon[3], theWalkingEngineParams.footMovement.heightPolygon[3], endingFactor * walkStepHeight);
    POINT(theWalkingEngineParams.footMovement.forwardPolygon[4], theWalkingEngineParams.footMovement.heightPolygon[4], endingFactor * walkStepHeight);
    END_POLYGON(output, len, POLYNOM_DEGREE);

    // Now add a rotation around x
    // TODO: find out, why this was in here. Was not used since parameter was always zero.
    /*for (int i = 1; i < len; i++)
      output[i].rx = (output[i] - output[i - 1]).rotate2D(-output[i].r).y * theWalkingEngineParams.footMovement.footRoll;*/
    // Add position offset from custom step trajectory
    for (int i = 0; i < len; i++)
    {
      output[len - i - 1].x += plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].x - polygonEnd.x;
      output[len - i - 1].y += plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].y - polygonEnd.y;
      output[len - i - 1].z += plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum].z - polygonEnd.z;
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum] = Point(currentFootStepTrajectory[len-i-1]);
    }
  }
}

void PatternGenerator2017::updateCoM(Point CoM)
{
  this->CoM=CoM;
}

void PatternGenerator2017::updateFootSteps()
{
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2017:saveCustomSteps")
    saveStepFiles();
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2017:loadCustomSteps")
    loadStepFiles();

  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:leftFoot", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:rightFoot", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:robotPoseAfterStep", "drawingOnField");

  DECLARE_PLOT("module:PatternGenerator2017:CurrentSpeedX");
  DECLARE_PLOT("module:PatternGenerator2017:CurrentSpeedY");
  DECLARE_PLOT("module:PatternGenerator2017:CurrentSpeedR");
  DECLARE_PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams");
  DECLARE_PLOT("module:PatternGenerator2017:fallDownAngleReduction.x");
  DECLARE_PLOT("module:PatternGenerator2017:fallDownAngleReduction.y");
  DECLARE_PLOT("module:PatternGenerator2017:curStepDurationAddStep");
  DECLARE_PLOT("module:PatternGenerator2017:stabilityErrorX");
  DECLARE_PLOT("module:PatternGenerator2017:stabilityErrorY");
  DECLARE_PLOT("module:PatternGenerator2017:stabilityError");
  DECLARE_PLOT("module:PatternGenerator2017:angleSumTiltFront");
  DECLARE_PLOT("module:PatternGenerator2017:angleSumTiltBack");

  DECLARE_DEBUG_DRAWING3D("module:PatternGenerator2017:customSteps", "field");

  COMPLEX_DRAWING3D("module:PatternGenerator2017:customSteps")
  {
    if (currentStepFileIndex != -1)
    {
      Pose2f pose;
      ColorRGBA stepColor = ColorRGBA(255, 0, 0, 255);

      
      unsigned char colorIncrease = static_cast<unsigned char>(255 / currentSteps.steps.size());

      for (const Step& step : currentSteps.steps)
      {
        if (!step.onFloor[LEFT_FOOT])
        {
          pose.translate(step.footPos[LEFT_FOOT].translation + Vector2f(0, theWalkingEngineParams.footMovement.footYDistance));
          pose.rotation += step.footPos[LEFT_FOOT].rotation;

          Pose3f pose3f(pose.translation.x() * 1000.f, pose.translation.y() * 1000.f, 0.f);
          pose3f.rotation = RotationMatrix(0.f, 0.f, pose.rotation);
          
          FOOT3D("module:PatternGenerator2017:customSteps", pose3f, true, stepColor);
          pose.translate(0,-theWalkingEngineParams.footMovement.footYDistance);
        }
        else if (!step.onFloor[RIGHT_FOOT])
        {
          pose.translate(step.footPos[RIGHT_FOOT].translation - Vector2f(0, theWalkingEngineParams.footMovement.footYDistance));
          pose.rotation += step.footPos[RIGHT_FOOT].rotation;

          Pose3f pose3f(pose.translation.x() * 1000.f, pose.translation.y() * 1000.f, 0.f);
          pose3f.rotation = RotationMatrix(0.f, 0.f, pose.rotation);

          FOOT3D("module:PatternGenerator2017:customSteps", pose3f, false, stepColor);
          pose.translate(0,theWalkingEngineParams.footMovement.footYDistance);
        }

        stepColor.r -= colorIncrease;
        stepColor.b += colorIncrease;
      }
    }

    Pose3f ball(theBallModelAfterPreview.estimate.position.x(), theBallModelAfterPreview.estimate.position.y(), 0.f);
    ball.rotateY(pi_2);
    CIRCLE3D("module:PatternGenerator2017:customSteps", ball, 50.f, 5.f, ColorRGBA::white);
  }

  localSteps.reset();

  if (theFallDownState.state != FallDownState::upright)
    reset();

  Point CoM(theRobotModel.centerOfMass.x() / 1000, theRobotModel.centerOfMass.y() / 1000, (theRobotModel.centerOfMass.z()) / 1000, 0);
  updateCoM(CoM);

  updateWalkState();
  
  if (useResetPreview 
      && running 
      && resetPreviewPossible 
      && !resetFootPosition.customStepRunning
      && currentState != customSteps)
  {
    stepsInPreview = 0;
    stepsSinceLastCustomStep = resetFootPosition.stepsSinceCustomStep;
    currentPreviewLength = 0;
    localRefZMP2018.zmpWCS.clear();
    localRefZMP2018.zmpRCS.clear();
    leftFootPose2f = lastLeftFootPose2f = resetFootPosition.footPos[LEFT_FOOT];
    rightFootPose2f = lastRightFootPose2f = resetFootPosition.footPos[RIGHT_FOOT];
    lastStep = resetFootPosition;
    lastStep.onFloor[LEFT_FOOT] = true;
    lastStep.onFloor[RIGHT_FOOT] = true;
    if (plannedFootSteps.steps[0].onFloor[RIGHT_FOOT])
    {
      robotPose2f = rightFootPose2f;
      robotPose2f.translate(0, theWalkingEngineParams.footMovement.footYDistance);
      //deltaDirection[firstSingleSupport] = (rightFootPose2f.rotation - lastRightFootPose2f.rotation) / (2 * singleSupportDuration());
      //deltaDirection[firstDoubleSupport] = 0.f;
    }
    else
    {
      robotPose2f = leftFootPose2f;
      robotPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
      //deltaDirection[secondSingleSupport] = (leftFootPose2f.rotation - lastLeftFootPose2f.rotation) / (2 * singleSupportDuration());
      //deltaDirection[secondDoubleSupport] = 0.f;
    }
    stateCounter = 0;
    currentWalkingPhase = skipStepInReset ? ((resetFootPosition.phase == firstSingleSupport) ? secondDoubleSupport : firstDoubleSupport) : resetFootPosition.phase;
    // speeds
    lastSpeed = speedBeforeStep;
    currentMovement.speed = resetFootPosition.speed;
    // ref zmp
    zmp.x() = resetFootPosition.zmp.x();
    zmp.y() = resetFootPosition.zmp.y();
    lastZMPRCS.x() = resetFootPosition.lastZMPRCS.x();
    lastZMPRCS.y() = resetFootPosition.lastZMPRCS.y();
    lpxss = resetFootPosition.lpxss;
    lastspdx = resetFootPosition.lastspdx;
    currentState = walking;
    
    currentTimeStamp = resetFootPosition.timestamp + 1;
    plannedFootSteps.steps.clear();
    direction = (leftFootPose2f.rotation + rightFootPose2f.rotation)/2.f;
    
  }
  
  updateWalkPhase();
  
  if (currentState == walking || currentState == stopping || currentState == customSteps)
    updatePreview();


  localSteps.walkState = currentState;

  static int timediff;
  timediff = theReferenceModificator.creationTime - currentTimeStamp;
  MODIFY("module:PatternGenerator2017:timediff", timediff);

  appliedReactiveStep[LEFT_FOOT] = 0;
  appliedReactiveStep[RIGHT_FOOT] = 0;
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
          appliedReactiveStep[footNum] += mod;
        }
      }
    }

  }

  localSteps.suggestedStep = getNextStep();
  localSteps.running = running;

  // not using currentMovement.speed directly,
  // because y speed there is not constant (switches between y and 0)
  speedBuffer.push_front(currentMovement.speed);

  localSteps.time = currentTimeStamp;

  if (useResetPreview && running)
  {
    bool isSingleSupport = (plannedFootSteps.steps[0].phase == firstSingleSupport ||
      plannedFootSteps.steps[0].phase == secondSingleSupport);
    bool stepStart = isSingleSupport &&
      plannedFootSteps.steps[0].frameInPhase == 0;

    bool nextDoubleSupport = isSingleSupport &&
      (plannedFootSteps.steps[1].phase == firstDoubleSupport || plannedFootSteps.steps[1].phase == secondDoubleSupport);
    
    if (stepStart)
    {
      int footNum = (plannedFootSteps.steps[0].phase == secondSingleSupport) ? LEFT_FOOT : RIGHT_FOOT;
      robotPose2fAfterCurrentStep = Pose2f(plannedFootSteps.steps[plannedFootSteps.steps[0].singleSupportDurationInFrames].footPos[footNum]);
      robotPose2fAfterCurrentStep.translate(0.f, (-1 + 2 * footNum)*theWalkingEngineParams.footMovement.footYDistance);
    }
    if (nextDoubleSupport)
    {
      //speedBeforeStep = Pose2f(plannedFootSteps.steps[0].lastSpeed);
    }
  }

  if (useResetPreview && !plannedFootSteps.steps.empty() && !plannedFootSteps.steps[0].customStepRunning) {
    localSteps.robotPoseAfterStep = Point(robotPose2fAfterCurrentStep);
  }
  else {
    // TODO: use position after all of the current custom step
    /*if (plannedFootSteps.steps[0].customStepRunning)
    {
      
    }
    else*/
      localSteps.robotPoseAfterStep = Point(robotPose2f);
  }
  
  
  if (running)
  {

    LOG("PatternGenerator2017_addStep", "currentWalkingPhase", currentWalkingPhase);
  }

  PLOT("module:PatternGenerator2017:PreviewLength", currentPreviewLength);
  PLOT("module:PatternGenerator2017:CurrentSpeedX", currentMovement.speed.translation.x());
  PLOT("module:PatternGenerator2017:CurrentSpeedY", currentMovement.speed.translation.y());
  PLOT("module:PatternGenerator2017:CurrentSpeedR", currentMovement.speed.rotation);

  DEBUG_RESPONSE("debug drawing:module:PatternGenerator2017:leftFoot")
  {
    leftFootPose2f.translation *= 1000.f;
    lastLeftFootPose2f.translation *= 1000.f;
    ARROW("module:PatternGenerator2017:leftFoot",
      leftFootPose2f.translation.x(), leftFootPose2f.translation.y(),
      leftFootPose2f.translation.x() + cos(leftFootPose2f.rotation)*30.f,
      leftFootPose2f.translation.y() + sin(leftFootPose2f.rotation)*30.f,
      3, Drawings::solidPen, ColorRGBA::blue);
    ARROW("module:PatternGenerator2017:leftFoot",
      lastLeftFootPose2f.translation.x(), lastLeftFootPose2f.translation.y(),
      lastLeftFootPose2f.translation.x() + cos(lastLeftFootPose2f.rotation)*30.f,
      lastLeftFootPose2f.translation.y() + sin(lastLeftFootPose2f.rotation)*30.f,
      3, Drawings::solidPen, ColorRGBA::gray);
    leftFootPose2f.translation /= 1000.f;
    lastLeftFootPose2f.translation /= 1000.f;
  }

  DEBUG_RESPONSE("debug drawing:module:PatternGenerator2017:rightFoot")
  {
    rightFootPose2f.translation *= 1000.f;
    lastRightFootPose2f.translation *= 1000.f;
    ARROW("module:PatternGenerator2017:rightFoot",
      rightFootPose2f.translation.x(), rightFootPose2f.translation.y(),
      rightFootPose2f.translation.x() + cos(rightFootPose2f.rotation)*30.f,
      rightFootPose2f.translation.y() + sin(rightFootPose2f.rotation)*30.f,
      3, Drawings::solidPen, ColorRGBA::yellow);
    ARROW("module:PatternGenerator2017:rightFoot",
      lastRightFootPose2f.translation.x(), lastRightFootPose2f.translation.y(),
      lastRightFootPose2f.translation.x() + cos(lastRightFootPose2f.rotation)*30.f,
      lastRightFootPose2f.translation.y() + sin(lastRightFootPose2f.rotation)*30.f,
      3, Drawings::solidPen, ColorRGBA::white);

    rightFootPose2f.translation /= 1000.f;
    lastRightFootPose2f.translation /= 1000.f;
  }
  POSE_2D_SAMPLE("module:PatternGenerator2017:robotPoseAfterStep", robotPose2f, ColorRGBA::blue);

  PLOT("module:PatternGenerator2017:fallDownAngleReduction.x", fallDownAngleReductionFactor.x());
  PLOT("module:PatternGenerator2017:fallDownAngleReduction.y", fallDownAngleReductionFactor.y());
}

void PatternGenerator2017::updateFootPositions()
{
  // interpolate foot steps into foot positions using speed and trajectory of steps
  // only one position for each foot is needed so we can use the last left and right pose
  // and interpolate from there
  localFootPositions.running = running;
  localFootPositions.currentState = currentState;

  if (!running || currentState == goingToReady || currentState == goingToStandby)
  {
    plannedFootSteps.reset();
    
    localFootPositions = localSteps.suggestedStep;
    angleSumBodyTiltFront = 0;
    angleSumBodyTiltBack = 0;
    return;
  }
  if (plannedFootSteps.steps.empty())
    localFootPositions = localSteps.suggestedStep;
  else
  {
    Vector2a orientation = useIMUModel ? theIMUModel.orientation : theInertialSensorData.angle;
    int angleErrorFront = static_cast<int>(toDegrees(orientation.y() - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1]));
    int angleErrorBack = static_cast<int>(toDegrees(orientation.y() - theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0]));
    if (currentState == customSteps)
      angleSumBodyTiltBack = 0; 
    else if (orientation.y() < theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[0])
      angleSumBodyTiltBack += angleErrorBack * angleErrorBack / 10;
    else
      angleSumBodyTiltBack = static_cast<int>(static_cast<float>(angleSumBodyTiltBack) / angleSumDecay);

    if (currentState == customSteps)
      angleSumBodyTiltFront = 0;
    else if (orientation.y() > theWalkingEngineParams.walkTransition.fallDownAngleMinMaxY[1])
      angleSumBodyTiltFront += angleErrorFront * angleErrorFront / 10;
    else
      angleSumBodyTiltFront = static_cast<int>(static_cast<float>(angleSumBodyTiltFront) / angleSumDecay);
    PLOT("module:PatternGenerator2017:angleSumTiltFront", angleSumBodyTiltFront);
    PLOT("module:PatternGenerator2017:angleSumTiltBack", angleSumBodyTiltBack);
    localFootPositions = plannedFootSteps.getStep(0);
    localFootPositions.inKick = plannedFootSteps.getStep(0).customStepRunning && plannedFootSteps.getStep(0).inKick;
    resetPreviewPossible = plannedFootSteps.steps.size() > 1 &&
      (plannedFootSteps.steps[0].phase == firstDoubleSupport || plannedFootSteps.steps[0].phase == secondDoubleSupport) &&
      (plannedFootSteps.steps[1].phase == firstSingleSupport || plannedFootSteps.steps[1].phase == secondSingleSupport);
    if (resetPreviewPossible)
    {
      nextPlannedFootPosition = plannedFootSteps.steps[1];
      if (skipStepInReset)
        resetFootPosition = plannedFootSteps.steps[1];
      else
        resetFootPosition = plannedFootSteps.steps[0];
    }
    plannedFootSteps.popFront();
  }
}

void PatternGenerator2017::updateRefZMP2018()
{
  localRefZMP2018.running = running;
  if (running)
  {
    while (localRefZMP2018.zmpWCS.size() >= (size_t)currentPreviewLength)
      localRefZMP2018.zmpWCS.erase(localRefZMP2018.zmpWCS.begin());
    while (localRefZMP2018.zmpRCS.size() >= (size_t)currentPreviewLength)
      localRefZMP2018.zmpRCS.erase(localRefZMP2018.zmpRCS.begin());
    // Generate ref zmp from foot steps. Usually just one new frame - except at start of controller.
    while (!localSteps.empty())
    {
      //addRefZMP(localSteps.getStep(0));
      localSteps.popFront(); // foot step for that frame has done its duty and is dismissed
    }
    ASSERT(localRefZMP2018.zmpWCS.size() >= PREVIEW_LENGTH);
  }
  else
  {
    localRefZMP2018.zmpWCS.clear();
    localRefZMP2018.zmpRCS.clear();
  }
}

void PatternGenerator2017::addRefZMP(const Footposition &footPosition)
{
  float vxss;
  float spdx = footPosition.speed.x;
  float tss = footPosition.singleSupportDurationInFrames*(float)theFrameInfo.cycleTime;
  float tds = footPosition.doubleSupportDurationInFrames*(float)theFrameInfo.cycleTime;
  unsigned int pc = footPosition.frameInPhase;
  float positionInCycle = static_cast<float>(footPosition.frameInPhase) + (useHalfFramesInRefZMP ? 0.5f : 0.f);
  float plgn[4];

  if (spdx<FOOT_LEN / tss)
    vxss = spdx;
  else
    vxss = FOOT_LEN / tss;
  float curStepDuration = footPosition.stepDurationInSec;
  float vxds = (curStepDuration*spdx - (2 * tss*vxss)) / (2 * tds);

  Point p, fp; // p: ZMP in foot coordinate system. fp: Position of foot (including
               // rotation. So, first create ZMP trajectory through foot and 
               // translate/rotate it into the woorld coordinate system by using fp

  fp = footPosition.footPos[ZMP::phaseToZMPFootMap[footPosition.phase]];

  Point rf = footPosition.footPos[RIGHT_FOOT];
  Point lf = footPosition.footPos[LEFT_FOOT];

  rf.rotate2D(-fp.r);
  lf.rotate2D(-fp.r);

  // Diese Art der Erzeugung ist bei Kurven nicht mehr korrekt. Um wirklich eine m�glichst Gleichf�rmige
  // Vorw�rtsbewegung zu erzeugen mu� der ZMP im Roboterkoordinatensystem in x und y Richtung erzeugt werden,
  // nicht wie derzeit im Fu�koordinatensystem. Allerdings erfordert das Bedingungen wie die Ausrichtung
  // des Standfu�es zu Begin und zum Ende der Phase f�r die x-Geschwindigkeit (da diese dann nicht mehr nur
  // von der Fu�l�nge abh�ngt, sondern auch von der Rotation), und die aktuelle Ausrichtung, um die aktuelle
  // Breite des Fu�es bestimmen zu k�nnen (wichtig f�r die y-Schwingung).

  float pxss = vxss*tss / 2;

  const float *polygonLeft = theWalkingEngineParams.footMovement.polygonLeft;
  const float *polygonRight = theWalkingEngineParams.footMovement.polygonRight;

  bool toConvert = false;
  switch (footPosition.phase)
  {

  case firstSingleSupport:
    p.y = FourPointBezier1D(polygonLeft, positionInCycle / footPosition.singleSupportDurationInFrames);
    p.x = -pxss + pc*(float)theFrameInfo.cycleTime*vxss;
    toConvert = true;
    break;

  case firstDoubleSupport:
  {
    plgn[0] = polygonLeft[3];
    plgn[1] = (float)(rf.y - lf.y) / 2;
    plgn[2] = plgn[1];
    plgn[3] = -(float)(lf.y - rf.y) + polygonRight[0];
    p.y = FourPointBezier1D(plgn, positionInCycle / footPosition.doubleSupportDurationInFrames);
    toConvert = true;
  }
  p.x = +pxss + pc*(float)theFrameInfo.cycleTime*vxds;
  break;

  case secondSingleSupport:
    p.y = FourPointBezier1D(polygonRight, positionInCycle / footPosition.singleSupportDurationInFrames);
    p.x = -pxss + pc*theFrameInfo.cycleTime*vxss;
    toConvert = true;
    break;

  case secondDoubleSupport:
  {
    plgn[0] = polygonRight[3];
    plgn[1] = -(float)(rf.y - lf.y) / 2;
    plgn[2] = plgn[1];
    plgn[3] = (float)(lf.y - rf.y) + polygonLeft[0];
    p.y = FourPointBezier1D(plgn, positionInCycle / footPosition.doubleSupportDurationInFrames);
    toConvert = true;
  }
  p.x = +pxss + pc*theFrameInfo.cycleTime*vxds;
  break;

  default:
    p.x = -pxss;
    p.y = -theWalkingEngineParams.footMovement.footYDistance;
    toConvert = true;
    break;
  }

  if (spdx>0 && -lpxss>p.x)
    // speed is >0, so the zmp should go forward, but wants to jump back. We won't allow it, and wait
    // until the zmp reaches the last zmp point
    p.x = -lpxss;
  else if (spdx<0 && -lpxss<p.x)
    p.x = -lpxss;
  else if (spdx == 0 && toConvert)
  {
    const float intfac = 0.99f;
    lastZMPRCS.x() = pxss * (1 - intfac) + intfac * lastZMPRCS.x();
    //p.x = lastZMPRCS.x;
    lpxss = (float)-p.x;
  }
  else
  {
    lpxss = pxss;
    lastZMPRCS.x() = -lpxss;
  }

  if (lastspdx * spdx < 0)
    lpxss = 0;
  lastspdx = spdx;

  Point pRCS;

  // Translate and rotate the ZMP to world coordinate system
  if (toConvert)
  {
    p.rotate2D(footPosition.direction);
    p += fp;
    pRCS = p;
    pRCS.rotate2D(-footPosition.direction);
  }

  // Arne 07.02.17 - ZMP in RCS //
  //Point pRCS = p;
  //pRCS.rotate2D(-(*_footList)->direction);
  ZMP zmpRCS = pRCS;
  zmpRCS.direction = footPosition.direction;


#ifdef CENTER_ZMP_X
  p.x = fp.x;
#endif

  // Collect data for plotting
  // Warning: At first start of controller the preview will be filled,
  // but the first plotted point is the end of the preview, so there will
  // be a jump from 0 to the end position
  static Point lastPlotZMP;
  static Point t_ZMP;
  t_ZMP = p - lastPlotZMP;
  t_ZMP.rotate2D(-footPosition.direction);
  plotZMP += t_ZMP;
  plotZMP.x = std::fmod(plotZMP.x, 1.f);
  PLOT("module:PatternGenerator2017:unrotatedZMP.x", plotZMP.x);
  lastPlotZMP = p;


  zmp = p;
  zmp.timestamp = footPosition.timestamp;
  // Arne 07.02.17 - ZMP in RCS //
  //refZMP.addZMP_RCS(zmpRCS);
  //refZMP.addZMP(zmp);
  localRefZMP2018.zmpRCS.push_back(zmpRCS);
  localRefZMP2018.zmpWCS.push_back(zmp);
  if ((footPosition.phase == firstDoubleSupport || footPosition.phase == secondDoubleSupport) && footPosition.frameInPhase == footPosition.doubleSupportDurationInFrames - 1)
  {
    RefZMPState refZMPState;
    refZMPState.lastZMPRCS = lastZMPRCS;
    refZMPState.zmp = zmp;
    refZMPState.lpxss = lpxss;
    refZMPStates.push_back(refZMPState);
    if (refZMPStates.size() > 3)
      refZMPStates.erase(refZMPStates.begin());
  }
}

void PatternGenerator2017::update(FootSteps & steps)
{
  DECLARE_PLOT("module:PatternGenerator2017:unrotatedZMP.x");
  if (lastTimeExecuted != theFrameInfo.time)
  {
    lastTimeExecuted = theFrameInfo.time;
    updateFootSteps();
    updateFootPositions();
    updateRefZMP2018();
  }
  steps = localSteps;
}

void PatternGenerator2017::update(Footpositions &footPositions)
{
  if (lastTimeExecuted != theFrameInfo.time || lastTimeExecuted == 0)
  {
    lastTimeExecuted = theFrameInfo.time;
    updateFootSteps();
    updateFootPositions();
    updateRefZMP2018();
  }
  footPositions = localFootPositions;
}

void PatternGenerator2017::update(RefZMP2018 &refZMP2018)
{
  if (lastTimeExecuted != theFrameInfo.time)
  {
    lastTimeExecuted = theFrameInfo.time;
    updateFootSteps();
    updateFootPositions();
    updateRefZMP2018();
  }
  refZMP2018 = localRefZMP2018;
}

void PatternGenerator2017::update(SpeedInfo & speedInfo)
{
  // TODO: what if length is not static!
  if (PREVIEW_LENGTH !=0 && speedBuffer.size()>= PREVIEW_LENGTH -1)
  {
    speedInfo.speed = speedBuffer[PREVIEW_LENGTH -1];
    speedInfo.speedBeforePreview = speedBuffer[0];
    speedInfo.timestamp = currentTimeStamp - PREVIEW_LENGTH;
  }
  else
    speedInfo.speed = Pose2f();
  speedInfo.deceleratedByAcc = decelByAcc;
  speedInfo.currentCustomStep = currentCustomStep;

  speedInfo.customStepKickInPreview = customStepKickInPreview;

  if (currentCustomStep != WalkRequest::none)
  {
    
  }

  PLOT("representation:SpeedInfo:speed.x", speedInfo.speed.translation.x() * 1000);
  PLOT("representation:SpeedInfo:speed.y", speedInfo.speed.translation.y() * 1000);
  PLOT("representation:SpeedInfo:speed.r", speedInfo.speed.rotation * 1000);
}

void PatternGenerator2017::update(FallDownAngleReduction& fallDownAngleReduction) {
  updateFallDownReduction();
  fallDownAngleReduction.reductionFactor = fallDownAngleReductionFactor;
}

void PatternGenerator2017::StepsFile::mirror()
{
  ballOffset.y() *= -1.f;
  kickAngle *= -1.f;
  for (Step& step : steps)
    step.mirror();
}

bool PatternGenerator2017::StepsFile::isApplicable(float distance)
{
  return kickDistance[0] <= distance && distance < kickDistance[1];
}

void PatternGenerator2017::Step::mirror()
{
  {
    Pose2f tmp = footPos[RIGHT_FOOT];
    footPos[RIGHT_FOOT] = footPos[LEFT_FOOT];
    footPos[LEFT_FOOT] = tmp;
    footPos[RIGHT_FOOT].rotation *= -1.f;
    footPos[RIGHT_FOOT].translation.y() *= -1.f;
    footPos[LEFT_FOOT].rotation *= -1.f;
    footPos[LEFT_FOOT].translation.y() *= -1.f;
  }
  {
    bool tmp = onFloor[RIGHT_FOOT];
    onFloor[RIGHT_FOOT] = onFloor[LEFT_FOOT];
    onFloor[LEFT_FOOT] = tmp;
  }
  for (Vector3f& vec : swingFootTraj)
    vec.y() *= -1.f;
  for (Point& point : spline)
    point.y *= -1.f;
  mirrored = !mirrored;
}

void PatternGenerator2017::Step::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(footPos);
  if (out)
  {
    if (Global::getSettings().naoVersion == RobotConfig::V6)
      duration *= 12;
    else
      duration *= 10;
  }
  STREAM(duration);
  STREAM(onFloor);
  STREAM(kick);
  STREAM(swingFootTraj);
  if (in)
  {
    if (Global::getSettings().naoVersion == RobotConfig::V6)
      duration = std::max(1, duration / 12);
    else
      duration = std::max(1, duration / 10);
    int vectorSize = static_cast<int>(swingFootTraj.size());
    std::vector<Point> control(vectorSize);
    spline.resize(duration);
    if (vectorSize > 0 && swingFootTraj[0].x() == swingFootTraj[0].x()
      && swingFootTraj[0].x() == 0 && swingFootTraj[0].y() == 0 && swingFootTraj[0].z() == 0
      && swingFootTraj[vectorSize - 1].x() == 0 && swingFootTraj[vectorSize - 1].y() == 0 && swingFootTraj[vectorSize - 1].z() == 0)
    {
      for (int i = 0; i < vectorSize; i++)
        control[i] = Point(swingFootTraj[i]);
      BSpline<Point>::bspline(vectorSize - 1, 3, &(control[0]), &(spline[0]), duration);
    }
    else
      for (int i = 0; i < duration; i++)
        spline[i] = Point();
  }
  STREAM_REGISTER_FINISH;
}

void PatternGenerator2017::StepsFile::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(ballOffset);
  STREAM(kickAngle);
  STREAM(kickDistance);
  STREAM(translationThresholdXFront);
  STREAM(translationThresholdXBack);
  STREAM(translationThresholdY);
  STREAM(rotationThreshold);
  STREAM(timeUntilKickHack);
  STREAM(kickHackDuration);
  STREAM(kickHackKneeAngle);
  STREAM(steps);
  STREAM_REGISTER_FINISH;
}

void PatternGenerator2017::loadStepFile(std::string file, int idx)
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
  
  // Must begin with single support
  ASSERT(stepFiles[idx].steps.size() == 0 || !(stepFiles[idx].steps.front().onFloor[0] &&
           stepFiles[idx].steps.front().onFloor[1]));
}

void PatternGenerator2017::saveStepFile(std::string file, int idx, bool robot)
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

void PatternGenerator2017::loadStepFiles()
{
  stepFiles.clear();
  stepFiles.resize(WalkRequest::numOfStepRequests);
  for (int i = 0; i < WalkRequest::numOfStepRequests - WalkRequest::StepRequest::beginScript - 1; i++)
  {
    loadStepFile(WalkRequest::getName((WalkRequest::StepRequest)(WalkRequest::StepRequest::beginScript + 1 + i)), i);
  }
  
}

void PatternGenerator2017::saveStepFiles()
{
  for (int i = 0; i < WalkRequest::numOfStepRequests - WalkRequest::StepRequest::beginScript - 1; i++)
  {
    saveStepFile(WalkRequest::getName((WalkRequest::StepRequest)(WalkRequest::StepRequest::beginScript + 1 + i)), i);
  }
}

MAKE_MODULE(PatternGenerator2017, dortmundWalkingEngine)
