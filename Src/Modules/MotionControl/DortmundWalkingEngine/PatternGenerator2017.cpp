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
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Bezier.h"
#include "Tools/RobotParts/FootShape.h"
#include "Tools/Debugging/Annotation.h"
#include <algorithm>
using namespace DWE;

#define FOOT_LEN 0.1f // half of footlength is the edge of the back of the foot

//#define LOGGING

#ifndef WALKING_SIMULATOR
#include "Tools/Math/BHMath.h"

#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

PatternGenerator2017::PatternGenerator2017() : speedBuffer(Pose2f())
{
  wasReady = false;
  walkHeight = initialStandbyHeight;
  reset();
  baseRot = 0;
  loadStepFiles();

  InMapFile stream("csConverter2019.cfg");
  if (stream.exists())
  {
    stream >> csConverterParams;
  }
  INIT_DEBUG_IMAGE_BLACK(FootStepsDrawing, 640, 480);
}

int PatternGenerator2017::getOptimalPreviewLength()
{
  return std::max((unsigned int)PREVIEW_LENGTH + 1, doubleSupportDuration() + singleSupportDuration() + 1);
  //return doubleSupportDuration() + singleSupportDuration() + 1;
}

void PatternGenerator2017::setStepLength()
{
  stepsInPreview++;
  // setting direction to current center of feet rotation
  // this is done to remove inaccuracies from adding floats via deltaDirection
  direction = (leftFootPose2f.rotation + rightFootPose2f.rotation) / 2;

  if (currentState == customSteps && executedStep != currentSteps.steps.end())
  {
    // in custom steps, translation is directly taken from step definition
    int frameForNextTwoPhases = currentExecutedStep->duration + executedStep->duration;
    float timeForStep = (frameForNextTwoPhases * theFrameInfo.cycleTime);
    if (currentWalkingPhase == secondDoubleSupport) // next up: right foot movement
    {
      lastRightFootPose2f = rightFootPose2f;
      robotPose2f = leftFootPose2f;
      robotPose2f.translate(executedStep->footPos[RIGHT_FOOT].translation.x(), -theWalkingEngineParams.footMovement.footYDistance + executedStep->footPos[RIGHT_FOOT].translation.y());
      robotPose2f.rotation += executedStep->footPos[RIGHT_FOOT].rotation;
      rightFootPose2f = robotPose2f;
      rightFootPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
      deltaDirection[firstSingleSupport] = (rightFootPose2f.rotation - lastRightFootPose2f.rotation) / (2 * executedStep->duration);
      deltaDirection[firstDoubleSupport] = 0.f;
      currentMovement.speed = rightFootPose2f - lastRightFootPose2f;
      currentMovement.speed.translation *= timeForStep;
      currentMovement.speed.rotation *= timeForStep * 0.5f;
      // bc of our speed y definition (we want continuous speed):
      currentMovement.speed.translation.y() = executedStep->footPos[RIGHT_FOOT].translation.y() / 2.f;
    }
    else // left foot movement
    {
      lastLeftFootPose2f = leftFootPose2f;
      robotPose2f = rightFootPose2f;
      robotPose2f.translate(executedStep->footPos[LEFT_FOOT].translation.x(), theWalkingEngineParams.footMovement.footYDistance + executedStep->footPos[LEFT_FOOT].translation.y());
      robotPose2f.rotation += executedStep->footPos[LEFT_FOOT].rotation;
      leftFootPose2f = robotPose2f;
      leftFootPose2f.translate(0, theWalkingEngineParams.footMovement.footYDistance);
      deltaDirection[secondSingleSupport] = (leftFootPose2f.rotation - lastLeftFootPose2f.rotation) / (2 * executedStep->duration);
      deltaDirection[secondDoubleSupport] = 0.f;
      currentMovement.speed = (leftFootPose2f - lastLeftFootPose2f);
      currentMovement.speed.translation *= timeForStep;
      currentMovement.speed.rotation *= timeForStep * 0.5f;
      // bc of our speed y definition (we want continuous speed):
      currentMovement.speed.translation.y() = executedStep->footPos[LEFT_FOOT].translation.y() / 2.f;
    }
    if (!useRealSpeedInCustomSteps)
      currentMovement.speed = Pose2f();

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
  if ((stepSpeed.rotation >= 0 && currentWalkingPhase == firstDoubleSupport) || (stepSpeed.rotation < 0 && currentWalkingPhase == secondDoubleSupport))
    stepSpeed.rotation *= 2.f;
  else
    stepSpeed.rotation = 0.f;
  // same for y translation
  if ((stepSpeed.translation.y() >= 0 && currentWalkingPhase == firstDoubleSupport) || (stepSpeed.translation.y() < 0 && currentWalkingPhase == secondDoubleSupport))
    stepSpeed.translation.y() *= 2.f;
  else
    stepSpeed.translation.y() = 0.f;

  // set new foot position and update robot pose (next to new foot position)
  if (currentWalkingPhase == secondDoubleSupport) // left foot stays
  {
    lastRightFootPose2f = rightFootPose2f;
    robotPose2f = leftFootPose2f;
    robotPose2f.translate(stepSpeed.translation.x(), -theWalkingEngineParams.footMovement.footYDistance + stepSpeed.translation.y());
    robotPose2f.rotation += stepSpeed.rotation;
    rightFootPose2f = robotPose2f;
    rightFootPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
    deltaDirection[firstSingleSupport] = (rightFootPose2f.rotation - lastRightFootPose2f.rotation) / (2 * singleSupportDuration());
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
    deltaDirection[secondSingleSupport] = (leftFootPose2f.rotation - lastLeftFootPose2f.rotation) / (2 * singleSupportDuration());
    deltaDirection[secondDoubleSupport] = 0.f;
  }
}

PatternGenerator2017::~PatternGenerator2017(void) {}

inline unsigned int PatternGenerator2017::getPhaseLength(float ratio, float stepDur)
{
  return std::max(static_cast<int>(std::roundf(ratio * stepDur / theFrameInfo.cycleTime / 2)), 1);
}

void PatternGenerator2017::transitionToWalk()
{
  currentState = newState = walking;
  currentWalkingPhase = unlimitedDoubleSupport;
  // shorten starting phase
  while (plannedFootSteps.steps.size() > (size_t)desiredPreviewLength / 2)
  {
    localRefZMP2018.zmpWCS.pop_back();
    localRefZMP2018.zmpRCS.pop_back();
    plannedFootSteps.steps.pop_back();
  }
  localSteps.reset();
}

bool PatternGenerator2017::updateWalkState()
{
  Point CoMspeed = CoM - lastCoM;
  lastCoM = CoM;
  int oldState = currentState;
  ASSERT(!running || stateCounter > 0);
  stateCounter--;
  const Rangef z_h_limits = Rangef(0.2f, 0.265f);
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
    newMovement.speed.translation = theSpeedRequest.translation / 1000.f;
    newMovement.speed.rotation = theSpeedRequest.rotation;
    currentWalkingPhase = unlimitedDoubleSupport;
    // if motion command is non-zero-walk, start walking
    if (!isSpeed0() && theFrameInfo.getTimeSince(timeStampLastZeroRequest) > 150)
    {
      transitionToWalk();
      stateCounter = 0;
    }
    // if motion command is not walk, go to standBy
    if (!(theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionRequest.motion == MotionRequest::walk)
        && std::abs(CoMspeed.x) < theWalkingEngineParams.walkTransition.stopSpeedThresholdX && std::abs(CoMspeed.y) < theWalkingEngineParams.walkTransition.stopSpeedThresholdY
        && (theJoinedIMUData.imuData[anglesource].gyro.cast<float>()).norm() < maxGyroSpeedForTransition && localFootPositions.phase == unlimitedDoubleSupport)
    {
      stateCounter = theWalkingEngineParams.walkTransition.crouchingDownPhaseLength;
      currentState = newState = State::goingToStandby;
    }
    break;
  case State::walking:
    newMovement.speed.translation = theSpeedRequest.translation / 1000.f;
    newMovement.speed.rotation = theSpeedRequest.rotation;
    // if motion command is zero-walk or not walk, get to ready again
    if (!(theMotionSelection.ratios[MotionRequest::walk] == 1.f && theMotionSelection.targetMotion == MotionRequest::walk) || isSpeed0())
    {
      timeStampLastZeroRequest = theFrameInfo.time;
      newState = State::stopping;
    }
    break;
  case State::stopping:
    newMovement.speed = Pose2f();
    if (currentMovement.speed.translation.y() == 0 && currentMovement.speed.translation.x() == 0 && currentMovement.speed.rotation == 0)
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
    stateCounter = 1;
    return;
  }

  if (stateCounter == 0)
  {
    if (currentStepFileIndex != -1 && executedStep == currentSteps.steps.end())
    {
      transitionFromCustomSteps();
      applyAcceleration();
    }
    switch (currentWalkingPhase)
    {
    case unlimitedDoubleSupport:
      stateCounter = 1;
      if (currentState == State::walking)
      {
        applyAcceleration();
        if (currentMovement.speed.translation.y() > 0 || currentMovement.speed.rotation > 0)
          currentWalkingPhase = firstDoubleSupport;
        else
          currentWalkingPhase = secondDoubleSupport;
        setStepLength();
      }
      break;
    case firstSingleSupport:
      if (currentState == customSteps)
      {
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
      }
      else
      {
        if (currentState != customSteps && newState != currentState) // transition from walk/customsteps
          currentState = newState;
        if (currentState == walking && currentState != customSteps && transitionToCustomSteps())
          currentState = customSteps;
        if (currentState == customSteps)
        {
          setCustomWalkingPhase();
          currentExecutedStep = executedStep;
          executedStep++;
          stateCounter = currentExecutedStep->duration;
        }
        else
        {
          currentCustomStep = WalkRequest::StepRequest::none;
          stepsSinceLastCustomStep++;
          currentWalkingPhase = firstDoubleSupport;
          stateCounter = doubleSupportDuration();
          applyAcceleration();
        }
      }
      setStepLength();
      break;
    case firstDoubleSupport:
      if (currentState == customSteps)
      {
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
        stateCounter = currentExecutedStep->duration;
      }
      else
      {
        currentWalkingPhase = secondSingleSupport;
        stateCounter = singleSupportDuration();
        currentCustomStep = WalkRequest::StepRequest::none;
      }
      sideStepSum[0] = Point();
      sideStepSum[1] = Point();
      break;
    case secondSingleSupport:
      if (currentState == customSteps)
      {
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
      }
      else
      {
        if (currentState != customSteps && newState != currentState) // transition from walk/customsteps
          currentState = newState;
        if (currentState == walking && currentState != customSteps && transitionToCustomSteps())
          currentState = customSteps;
        if (currentState == customSteps)
        {
          setCustomWalkingPhase();
          currentExecutedStep = executedStep;
          executedStep++;
          stateCounter = currentExecutedStep->duration;
        }
        else
        {
          currentWalkingPhase = secondDoubleSupport;
          stateCounter = doubleSupportDuration();
          applyAcceleration();
          currentCustomStep = WalkRequest::StepRequest::none;
        }
      }
      setStepLength();
      break;
    case secondDoubleSupport:
      if (currentState == customSteps)
      {
        setCustomWalkingPhase();
        currentExecutedStep = executedStep;
        executedStep++;
        stateCounter = currentExecutedStep->duration;
      }
      else
      {
        currentWalkingPhase = firstSingleSupport;
        stateCounter = singleSupportDuration();
        currentCustomStep = WalkRequest::StepRequest::none;
      }
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

Vector2f PatternGenerator2017::prependCustomStep(std::list<CustomStep>& steps, Vector2f distance, bool onFloorLeft)
{
  // step limits
  Vector2f min(-0.10f, -0.05f);
  Vector2f max(0.10f, 0.05f);
  float innerMin = -0.01f;
  float innerMax = 0.01f;

  if (onFloorLeft)
    max.y() = innerMax;
  if (!onFloorLeft)
    min.y() = innerMin;

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

  CustomStep step;
  step.duration = 1;
  step.onFloor[0] = true;
  step.onFloor[1] = true;
  step.footPos[0] = Vector2f(0.f, 0.f);
  step.footPos[1] = Vector2f(0.f, 0.f);
  step.swingFootTraj = {};

  steps.push_front(step);

  step.duration = getPhaseLength(1 - theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(distance.x(), distance.y(), 0.f));
  step.onFloor[0] = onFloorLeft;
  step.onFloor[1] = !onFloorLeft;
  step.footPos[0] = step.onFloor[0] ? Vector2f(0.f, 0.f) : distance;
  step.footPos[1] = step.onFloor[1] ? Vector2f(0.f, 0.f) : distance;
  step.swingFootTraj = {};

  steps.push_front(step);

  return distanceLeft;
}

bool PatternGenerator2017::prependCustomSteps(std::list<CustomStep>& steps, Pose2f pose, bool startFootLeft, bool endFootLeft, int maxSteps, const CustomStepsFile& stepsFile)
{
  if (std::abs(pose.rotation) >= prependStepsMaxDistance.rotation || std::abs(pose.translation.x()) > prependStepsMaxDistance.translation.x()
      || std::abs(pose.translation.y()) > prependStepsMaxDistance.translation.y())
    return false;

  bool footLeft = startFootLeft;

  const bool xDistWithinThresh = -stepsFile.translationThresholdXFront < pose.translation.x() && pose.translation.x() < stepsFile.translationThresholdXBack;
  const bool yDistWithinThresh = std::abs(pose.translation.y()) < stepsFile.translationThresholdY;
  const bool rotWithinThresh = std::abs(pose.rotation) < stepsFile.rotationThreshold;

  if (xDistWithinThresh && yDistWithinThresh && rotWithinThresh)
    pose = Pose2f();
  else
  {
    if (yDistWithinThresh && !startFootLeft == pose.translation.y() > 0)
      pose.translation.y() = 0.f;
    if (rotWithinThresh && !startFootLeft == pose.rotation > 0)
      pose.rotation = 0_deg;
  }

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
      numRightSteps += !endFootLeft;
    }
    else if (pose.rotation < 0)
    {
      numRightSteps = std::max(numRightSteps, static_cast<int>(std::ceil(-pose.rotation / prependStepsRotMax)));
      numLeftSteps = numRightSteps - 1;
      numLeftSteps += startFootLeft;
      numLeftSteps += endFootLeft;
    }
    else // if no rotation, you have at least the starting foot step
    {
      numRightSteps += !startFootLeft && !endFootLeft;
      numLeftSteps += startFootLeft && endFootLeft;
    }

    // if no steps for rotation are required, add at least two steps for translation
    if ((numLeftSteps == 0 && numRightSteps == 0) || (pose.translation.y() > 0.f && numLeftSteps == 0) || (pose.translation.y() < 0.f && numRightSteps == 0))
    {
      ++numLeftSteps;
      ++numRightSteps;
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
    float xPerStep = 0.f, yPerStep = 0.f, rotPerStep = 0.f;
    while (!solutionFound && numLeftSteps + numRightSteps <= maxSteps)
    {
      const int numOfRotations = pose.rotation > 0 ? numLeftSteps : numRightSteps;
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
      A << a, a_fromY, b_fromX, b;
      Vector2f perStep = A.colPivHouseholderQr().solve(pose.translation);
      xPerStep = perStep.x();
      yPerStep = perStep.y();
      solutionFound = !std::isnan(xPerStep) && !std::isnan(yPerStep) && std::abs(xPerStep) < prependStepsTransMax.x() && std::abs(yPerStep) < prependStepsTransMax.y();
      // edge case: one rotation only at the end -> can lead to wrong solutions since yPerStep can be 0.
      if (numOfRotations == 1 && yPerStep == 0.f && pose.translation.y() != 0.f)
        solutionFound = false;

      if (!solutionFound)
      {
        ++numLeftSteps;
        ++numRightSteps;
      }
    }
    if (!solutionFound)
    {
      // OUTPUT_WARNING("module:PatternGenerator2017:prependCustomSteps - Could not find find steps to prepend to custom step");
      return false;
    }

    for (int i = 0; i < numLeftSteps + numRightSteps; i++)
    {
      const bool last = i == numLeftSteps + numRightSteps - 1;
      Pose2f footPos;

      footPos.translation.x() = xPerStep;
      footPos.translation.y() = (footLeft == (yPerStep > 0)) ? yPerStep : 0.f;
      footPos.rotation = (footLeft == (rotPerStep > 0)) ? rotPerStep : 0.f;

      CustomStep step;
      step.duration = getPhaseLength(1 - theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(xPerStep, yPerStep, rotPerStep));
      step.onFloor[0] = !footLeft;
      step.onFloor[1] = footLeft;
      step.footPos[0] = step.onFloor[0] ? Pose2f() : footPos;
      step.footPos[1] = step.onFloor[1] ? Pose2f() : footPos;
      step.kick = false;
      step.swingFootTraj = {};
      step.isPrepend = !last; // do not reset preview in last step

      steps.push_back(step);

      step.duration = 1;
      step.onFloor[0] = true;
      step.onFloor[1] = true;
      step.footPos[0] = Pose2f();
      step.footPos[1] = Pose2f();
      step.kick = false;
      step.swingFootTraj = {};
      step.isPrepend = !last; // do not reset preview in last step

      steps.push_back(step);

      footLeft = !footLeft;
    }
  }

  // If no prepend steps are required, add step here if necessary.
  if (endFootLeft == footLeft)
  {
    CustomStep step;
    step.duration = getPhaseLength(1 - theWalkingEngineParams.footMovement.doubleSupportRatio, calcDynamicStepDuration(0.f, 0.f, 0.f));
    step.onFloor[0] = !footLeft;
    step.onFloor[1] = footLeft;
    step.footPos[0] = Pose2f();
    step.footPos[1] = Pose2f();
    step.kick = false;
    step.swingFootTraj = {};
    step.isPrepend = false; // do not reset preview at this point anymore

    steps.push_back(step);
    step.duration = 1;
    step.onFloor[0] = true;
    step.onFloor[1] = true;
    step.footPos[0] = Pose2f();
    step.footPos[1] = Pose2f();
    step.kick = false;
    step.swingFootTraj = {};
    step.isPrepend = false; // do not reset preview at this point anymore

    steps.push_back(step);
  }
  return true;
}

template <class it> void PatternGenerator2017::correctOdometry(it& steps)
{
  for (CustomStep& step : steps)
  {
    if (step.onFloor[LEFT_FOOT] == step.onFloor[RIGHT_FOOT])
      continue;

    const bool foot = step.onFloor[LEFT_FOOT];

    Pose2f footSpeed;
    footSpeed.translation = step.footPos[foot].translation * 1000.f / (step.duration * theFrameInfo.cycleTime);
    footSpeed.rotation = step.footPos[foot].rotation / (step.duration * theFrameInfo.cycleTime);

    step.footPos[foot] = OdometryCorrection::correct(footSpeed,
        step.footPos[foot],
        theOdometryCorrectionTables.backCorrectionTable,
        theOdometryCorrectionTables.forwardCorrectionTable,
        theOdometryCorrectionTables.sideCorrectionTable,

        theOdometryCorrectionTables.rotCorrectionTable,
        theOdometryCorrectionTables.rot2DCorrectionTable,
        true);
  }
}

Vector2f PatternGenerator2017::getBallModelWalk() const
{
  const Pose3f robotPose3fWalk(static_cast<Pose3f>((footPosBuf.back().footPos[LEFT_FOOT] + footPosBuf.back().footPos[RIGHT_FOOT]) / 2.f));
  const Pose2f robotPose2fWalk(static_cast<Pose2f>(robotPose3fWalk));
  return robotPose2fWalk * theBallModel.estimate.position;
}

Pose2f PatternGenerator2017::getRobotPoseAfterPreviewField() const
{
  Pose2f robotPose2fAfterCurrentStep = this->robotPose2fAfterCurrentStep;
  robotPose2fAfterCurrentStep.translation *= 1000.f;
  const Pose3f robotPose3fAfterCurrentStep(robotPose2fAfterCurrentStep);
  const Pose3f localRobotPose3fAfterPreview = transformWalkToField(robotPose3fAfterCurrentStep);
  return static_cast<Pose2f>(localRobotPose3fAfterPreview);
}

Pose2f PatternGenerator2017::calcKickPose(const CustomStepsFile& steps, const Vector2f& kickTarget)
{
  //Vector2f kickTargetRel = Transformation::fieldToRobot(theRobotPoseAfterPreview, kickTarget);
  const Vector2f kickTargetRel = Transformation::fieldToRobot(getRobotPoseAfterPreviewField(), kickTarget);

  const Vector2f ballWalk = getBallModelWalk();
  Vector2f localBallModelAfterPreview = ballWalk - this->robotPose2fAfterCurrentStep.translation * 1000.f;
  localBallModelAfterPreview.rotate(-this->robotPose2fAfterCurrentStep.rotation);

  Pose2f kickPose((kickTargetRel - localBallModelAfterPreview).angle(), localBallModelAfterPreview);

  kickPose.rotation -= steps.kickAngle;
  kickPose.translation /= 1000.f;
  kickPose.translate(-steps.ballOffset);

  return kickPose;
}

bool PatternGenerator2017::isStablePosition()
{
  Angle angleX = theJoinedIMUData.imuData[anglesource].angle.x();
  Angle angleY = theJoinedIMUData.imuData[anglesource].angle.y();

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

  if (currentState == walking && requestedStep >= WalkRequest::StepRequest::beginScript && stepsSinceLastCustomStep > stepsBetweenCustomSteps && !customStepKickInPreview && isStablePosition())
  {
    int idx = (int)(requestedStep - WalkRequest::StepRequest::beginScript - 1);

    // if type is ::any, currentSteps are already set
    if (theMotionSelection.walkRequest.stepRequest != WalkRequest::any)
    {
      // copy steps, may be modified according to ball position / mirror afterwards
      currentSteps = stepFiles[idx];
      if (theMotionRequest.kickRequest.mirror)
        currentSteps.mirror();
    }

    // initialize custom step
    int lastPhaseLength = doubleSupportDuration();

    if ((currentWalkingPhase == secondSingleSupport && currentSteps.steps.begin()->onFloor[0]) || (currentWalkingPhase == firstSingleSupport && currentSteps.steps.begin()->onFloor[1]))
    {
      std::list<CustomStep> preDoubleSupportStep;
      CustomStep dsStep;
      dsStep.isPrepend = true;
      preDoubleSupportStep.push_back(dsStep);
      currentSteps.steps.insert(currentSteps.steps.begin(), preDoubleSupportStep.begin(), preDoubleSupportStep.end());
      executedStep = currentSteps.steps.begin();
      //currentExecutedStep = executedStep;
      //executedStep++;

      //setCustomWalkingPhase();
      currentStepFileIndex = idx;
      currentCustomStep = requestedStep;
      ANNOTATION("CustomStep", "Executed " << WalkRequest::getName(currentCustomStep));
      // set preview to be long enough for custom steps (double supp + single supp)
      int maxPhaseLength = desiredPreviewLength;
      for (const auto& step : currentSteps.steps)
      {
        if (step.onFloor[LEFT_FOOT] == true && step.onFloor[RIGHT_FOOT] == true)
          lastPhaseLength = 0;
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

  if (requestedStep != WalkRequest::StepRequest::none && currentStepFileIndex != -1)
  {
    previousCustomStep = currentCustomStep;
    previousCustomStepTimeStamp = theFrameInfo.time;
    previousCustomStepMirrored = currentSteps.steps.back().mirrored;
    //OUTPUT_TEXT(currentTimeStamp << ":" << WalkRequest::getName(requestedStep));
    if (!customStepKickInPreview && currentStepFileIndex != -1)
    {
      for (const auto& step : currentSteps.steps)
        if (step.kick)
          customStepKickInPreview = true;
    }
  }

  return currentStepFileIndex != -1;
}

WalkRequest::StepRequest PatternGenerator2017::selectCustomStepForKick(const Vector2f& kickTarget)
{
  WalkRequest::StepRequest bestKick = WalkRequest::StepRequest::none;
  float bestScore = INFINITY;
  const Vector2f ballToKickTarget = kickTarget - Transformation::robotToField(theRobotPose, theBallModel.estimate.position);
  const std::vector<WalkRequest::StepRequest>& kickList =
      (theRoleSymbols.role == BehaviorData::ballchaserKeeper || theRoleSymbols.role == BehaviorData::replacementKeeper || theRoleSymbols.role == BehaviorData::keeper) ? activeGoalieKicks : activeKicks;

  localCustomStepSelection.ballModel = getBallModelWalk();
  localCustomStepSelection.robotPoseAfterPreviw = getRobotPoseAfterPreviewField();

  for (WalkRequest::StepRequest kick : kickList)
  {
    int idx = (int)(kick - WalkRequest::StepRequest::beginScript - 1);
    if (!stepFiles[idx].isApplicable(ballToKickTarget.norm() / 1000.f))
      continue;

    for (int i = 0; i < 2; i++)
    {
      CustomStepsFile steps = stepFiles[idx];
      if (i)
        steps.mirror();

      std::list<CustomStep> prependedSteps;
      const Pose2f kickPose(calcKickPose(steps, kickTarget));

      bool prependSuccessful = prependCustomSteps(prependedSteps, kickPose, currentWalkingPhase == firstSingleSupport, steps.steps.front().onFloor[LEFT_FOOT], maxPrependedSteps, steps);

      steps.steps.insert(steps.steps.begin(), prependedSteps.begin(), prependedSteps.end());

      if (customStepOdometryCorrection)
        correctOdometry(steps.steps);

      // TODO: better measurement or scrap it completely since maxPrependedSteps tells the story..
      // maybe use the distance and rotation of the prepended steps
      // TODO: distinguish between long and short kicks
      float score = getScoreForPrependSteps(prependedSteps);

      if (prependSuccessful)
      {
        auto& k = localCustomStepSelection.kicks.emplace_back();
        k.step = kick;
        k.mirror = i;
        k.pose = kickPose;
        k.score = score;
      }

      if (prependSuccessful && score < maxPrependStepsScore && score < bestScore)
      {
        bestKick = kick;
        bestScore = score;
        currentSteps = steps;

        const Vector2f ballWalk(getBallModelWalk());
        const Pose3f debugBallWalk(Vector3f(ballWalk.x(), ballWalk.y(), 50.f));
        debugKickBall = transformWalkToField(debugBallWalk).translation;
        debugKickCurrentPose = static_cast<Pose3f>(getRobotPoseAfterPreviewField());
        Pose3f kickPosemm(kickPose);
        kickPosemm.translation *= 1000.f;
        debugKickTargetPose = debugKickCurrentPose * kickPosemm;

        localCustomStepSelection.currentStep = kick;
        localCustomStepSelection.currentSteps = steps;
      }
    }
  }

  return bestKick;
}

float PatternGenerator2017::getScoreForPrependSteps(const std::list<CustomStep>& steps)
{
  float score = 0.f;
  for (const CustomStep& step : steps)
  {
    //score += 1.f;
    score += std::abs(step.footPos[LEFT_FOOT].translation.y()) * stepYTranslationWeightForScore;
    score += std::abs(step.footPos[RIGHT_FOOT].translation.y()) * stepYTranslationWeightForScore;
    score += (std::abs(step.footPos[LEFT_FOOT].rotation.toDegrees() / 90.f) * stepRotationWeightForScore);
    score += (std::abs(step.footPos[RIGHT_FOOT].rotation.toDegrees() / 90.f) * stepRotationWeightForScore);
  }
  return score;
}

void PatternGenerator2017::transitionFromCustomSteps()
{
  currentState = walking;
  currentStepFileIndex = -1;
  currentCustomStep = WalkRequest::StepRequest::none;

  Pose2f currentSpeed;
  if (currentWalkingPhase == firstSingleSupport)
  {
    currentSpeed = rightFootPose2f - Pose2f(leftFootPose2f).translate(0.f, -2.f * theWalkingEngineParams.footMovement.footYDistance);
    currentSpeed.translation /= 1000.f;
  }
  else
  {
    currentSpeed = leftFootPose2f - Pose2f(rightFootPose2f).translate(0.f, -2.f * theWalkingEngineParams.footMovement.footYDistance);
    currentSpeed.translation /= 1000.f;
  }
  calcDynamicStepDuration(currentSpeed.translation.x(), currentSpeed.translation.y(), currentSpeed.rotation);
}

float PatternGenerator2017::calcDynamicStepDuration(float speedX, float speedY, float speedR)
{
  // TODO: acceleration dependent on step length, step duration dependent on step length. What about dep on acc?

  if (theWalkingEngineParams.footMovement.minStepDuration != theWalkingEngineParams.footMovement.maxStepDuration)
  {
    if (running)
    {
      //float maxSpeedR = (speedX == 0.f && speedY == 0.f) ?
      //  (theWalkingEngineParams.speedLimits.rOnly * theWalkingEngineParams.speedLimits.speedFactor) : (theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedLimits.speedFactor);

      float xLimit = speedX > 0
          ? (theWalkingEngineParams.speedLimits.xForward * theWalkingEngineParams.speedLimits.speedFactor) / 1000.f
          : (theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedLimits.speedFactor) / 1000.f;
      float yLimit = xLimit; // 2.f * (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor) / 1000.f;
      float rLimit = xLimit; // 1.5f * maxSpeedR;

      float factorX = std::abs(xLimit - std::abs(speedX)) / xLimit;
      float factorY = std::abs(yLimit - std::abs(speedY)) / yLimit;
      float factorR = std::abs(rLimit - std::abs(speedR)) / rLimit;

      float distToMax = std::min(factorX, std::min(factorR, factorY));
      float result = distToMax * theWalkingEngineParams.footMovement.maxStepDuration + (1.f - distToMax) * theWalkingEngineParams.footMovement.minStepDuration;
      // duration of first step of side walk can be adjusted in parameters, the faster we walk sideways, the more the parameter weighs in
      if (theWalkingEngineParams.footMovement.leadingSideStepSpeedUp > 0.f && std::abs(currentMovement.speed.translation.y()) > 0.01f)
      {
        float ySpeedPercentage = std::abs(currentMovement.speed.translation.y()) / ((theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor) / 1000.f);
        // 1.f means no speed up, range is from 1.f to parameter
        float firstLegSpeedUp = 1.f * (1.f - ySpeedPercentage) + theWalkingEngineParams.footMovement.leadingSideStepSpeedUp * ySpeedPercentage;
        if (currentMovement.speed.translation.y() > 0 && currentWalkingPhase == WalkingPhase::secondSingleSupport)
          result /= firstLegSpeedUp;
        else if (currentMovement.speed.translation.y() > 0 && currentWalkingPhase == WalkingPhase::firstSingleSupport)
          result *= firstLegSpeedUp;

        if (currentMovement.speed.translation.y() < 0 && currentWalkingPhase == WalkingPhase::firstSingleSupport)
          result /= firstLegSpeedUp;
        else if (currentMovement.speed.translation.y() < 0 && currentWalkingPhase == WalkingPhase::secondSingleSupport)
          result *= firstLegSpeedUp;
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

bool PatternGenerator2017::isWalking()
{
  return !(currentState == ready || currentState == standby);
}

void PatternGenerator2017::applyAcceleration()
{
  PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams", curStepDuration);

  /************** rotation ****************/
  // apply rotation speed if feet were not rotated within boundaries
  float maxStepAccR = std::max<float>(theWalkingEngineParams.acceleration.maxAccR, theMotionRequest.walkRequest.accLimits.rotation) * curStepDuration;
  const bool sameDirection = !(newMovement.speed.rotation * currentMovement.speed.rotation < 0);
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
          currentMovement.speed.rotation = std::min(0.f, std::max(currentMovement.speed.rotation - maxStepAccR, static_cast<float>(newMovement.speed.rotation)));
        else
          currentMovement.speed.rotation = std::max(0.f, std::min(currentMovement.speed.rotation + maxStepAccR, static_cast<float>(newMovement.speed.rotation)));
      }
    }
    // acceleration r (here, the new and current speed direction is the same and new speed has higher absolute)
    else
    {
      float maxSpeedR = (newMovement.speed.translation.x() == 0.f && newMovement.speed.translation.y() == 0.f)
          ? (theWalkingEngineParams.speedLimits.rOnly * theWalkingEngineParams.speedLimits.speedFactor)
          : (theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedLimits.speedFactor);
      if (newMovement.speed.rotation < 0) // right
        currentMovement.speed.rotation = std::max<float>(std::max(static_cast<float>(newMovement.speed.rotation), currentMovement.speed.rotation - maxStepAccR), -maxSpeedR);
      else // left
        currentMovement.speed.rotation = std::min<float>(std::min(static_cast<float>(newMovement.speed.rotation), currentMovement.speed.rotation + maxStepAccR), maxSpeedR);
    }
  }

  /************** x ****************/
  float maxStepAccX = newMovement.speed.translation.x() > 0
      ? std::max<float>(theWalkingEngineParams.acceleration.maxAccXForward, theMotionRequest.walkRequest.accLimits.translation.x()) * curStepDuration / 2
      : std::max<float>(theWalkingEngineParams.acceleration.maxAccXBackward, theMotionRequest.walkRequest.accLimits.translation.x()) * curStepDuration / 2;
  const bool sameDirectionX = newMovement.speed.translation.x() * currentMovement.speed.translation.x() >= 0;
  // deceleration x -> always full up to 0
  if (std::abs(newMovement.speed.translation.x()) < std::abs(currentMovement.speed.translation.x()) || !sameDirectionX)
  {
    if (sameDirectionX)
      currentMovement.speed.translation.x() = newMovement.speed.translation.x();
    else
    {
      // if movement is minimal, direction change is allowed!
      if (currentMovement.speed.translation.x() > 0.f)
        currentMovement.speed.translation.x() = std::min(0.f, std::max(currentMovement.speed.translation.x() - maxStepAccX, newMovement.speed.translation.x()));
      else
        currentMovement.speed.translation.x() = std::max(0.f, std::min(currentMovement.speed.translation.x() + maxStepAccX, newMovement.speed.translation.x()));
    }
  }
  // acceleration x (here, the new and current speed direction is the same and new speed has higher absolute)
  else
  {
    float maxSpeedXForward = (std::abs(newMovement.speed.translation.y()) > 20.f)
        ? (theWalkingEngineParams.speedLimits.xForwardOmni * theWalkingEngineParams.speedLimits.speedFactor)
        : (theWalkingEngineParams.speedLimits.xForward * theWalkingEngineParams.speedLimits.speedFactor);
    // slow down in Penalty Shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    {
      maxSpeedXForward *= penaltyShootoutSlowDownFactor;
    }
    // slow down when hot
    if (theRobotHealth.maxJointTemperature > slowDownTemperature)
    {
      maxSpeedXForward /= theRobotHealth.maxJointTemperature / slowDownTemperature;
    }

    float maxSpeedXBackward = theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedLimits.speedFactor;
    // slow down in Penalty Shootout
    if (Global::getSettings().gameMode == Settings::penaltyShootout || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    {
      maxSpeedXBackward *= penaltyShootoutSlowDownFactor;
    }
    // slow down when hot
    if (theRobotHealth.maxJointTemperature > slowDownTemperature)
    {
      maxSpeedXBackward /= theRobotHealth.maxJointTemperature / slowDownTemperature;
    }

    //maxSpeedXForward /= std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());
    //maxSpeedXBackward /=  std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());

    if (newMovement.speed.translation.x() < 0) // backward
      currentMovement.speed.translation.x() = std::max(std::max(newMovement.speed.translation.x(), currentMovement.speed.translation.x() - maxStepAccX), -maxSpeedXBackward / 1000.f);
    else // forward
      currentMovement.speed.translation.x() = std::min(std::min(newMovement.speed.translation.x(), currentMovement.speed.translation.x() + maxStepAccX), maxSpeedXForward / 1000.f);

    currentMovement.speed.translation.x() /=
        std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());
  }

  /************** y ****************/
  // no acceleration in y at start if rotation is in different direction!
  if (currentWalkingPhase == unlimitedDoubleSupport && currentMovement.speed.rotation * newMovement.speed.translation.y() < 0)
    return;
  float maxStepAccY = std::max<float>(theWalkingEngineParams.acceleration.maxAccY, theMotionRequest.walkRequest.accLimits.translation.y()) * curStepDuration;
  float maxSpeedY = theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor;
  // slow down in Penalty Shootout
  if (Global::getSettings().gameMode == Settings::penaltyShootout || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    maxSpeedY *= penaltyShootoutSlowDownFactor;
  }
  // slow down when hot
  if (theRobotHealth.maxJointTemperature > slowDownTemperature)
  {
    maxSpeedY /= theRobotHealth.maxJointTemperature / slowDownTemperature;
  }

  //maxSpeedY /= std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());

  const bool sameDirectionY = newMovement.speed.translation.y() * currentMovement.speed.translation.y() >= 0;
  // if feet were not parallel, last support foot has to have a bigger than default y distance to the robotpose
  // (see setStepLength : new step is created like this:
  // support foot -> translate with speed + footYDistance -> rotate -> translate footYDistance
  bool lastBaseFootRight = (currentWalkingPhase == secondDoubleSupport);
  Pose2f lastBaseFoot = lastBaseFootRight ? rightFootPose2f : leftFootPose2f;
  Pose2f lastPose2f = robotPose2f;
  lastPose2f.rotation = lastBaseFoot.rotation;
  Pose2f poseDifference = lastPose2f - lastBaseFoot;
  const bool feetParallelY = std::abs(std::abs(poseDifference.translation.y()) - theWalkingEngineParams.footMovement.footYDistance) < 0.001f;
  const bool yStepPossible = (currentWalkingPhase == secondDoubleSupport && newMovement.speed.translation.y() <= 0)
      || (currentWalkingPhase == firstDoubleSupport && newMovement.speed.translation.y() >= 0);
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
          currentMovement.speed.translation.y() = std::min(0.f, std::max(currentMovement.speed.translation.y() - maxStepAccY, std::max(newMovement.speed.translation.y(), -maxSpeedY / 1000.f)));
        else
          currentMovement.speed.translation.y() = std::max(0.f, std::min(currentMovement.speed.translation.y() + maxStepAccY, std::min(newMovement.speed.translation.y(), maxSpeedY / 1000.f)));
      }
    }
    // acceleration y (here, the new and current speed direction is the same and new speed has higher absolute)
    else
    {
      if (newMovement.speed.translation.y() < 0) // right
        currentMovement.speed.translation.y() = std::max(std::max(newMovement.speed.translation.y(), currentMovement.speed.translation.y() - maxStepAccY), -maxSpeedY / 1000.f);
      else // left
        currentMovement.speed.translation.y() = std::min(std::min(newMovement.speed.translation.y(), currentMovement.speed.translation.y() + maxStepAccY), maxSpeedY / 1000.f);
    }

    currentMovement.speed.translation.y() /=
        std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());
  }
  // set step duration based on new speed
  curStepDuration = calcDynamicStepDuration(currentMovement.speed.translation.x(), currentMovement.speed.translation.y(), currentMovement.speed.rotation);
}

void PatternGenerator2017::setCustomWalkingPhase()
{
  if (executedStep->onFloor[LEFT_FOOT] && !executedStep->onFloor[RIGHT_FOOT])
    currentWalkingPhase = firstSingleSupport;
  else if (!executedStep->onFloor[LEFT_FOOT] && executedStep->onFloor[RIGHT_FOOT])
    currentWalkingPhase = secondSingleSupport;
  else if (executedStep->onFloor[LEFT_FOOT] && executedStep->onFloor[RIGHT_FOOT] && currentWalkingPhase == firstSingleSupport)
    currentWalkingPhase = firstDoubleSupport;
  else if (executedStep->onFloor[LEFT_FOOT] && executedStep->onFloor[RIGHT_FOOT] && currentWalkingPhase == secondSingleSupport)
    currentWalkingPhase = secondDoubleSupport;
  stateCounter = executedStep->duration;
}

void PatternGenerator2017::reset()
{
  robotPose2f = Pose2f();
  robotPose2fAfterCurrentStep = Pose2f();
  direction = 0.f;
  for (int i = 0; i < 5; i++)
    deltaDirection[i] = 0.f;
  desiredPreviewLength = 1;
  running = false;
  currentMovement.speed = Pose2f();
  newMovement.speed = Pose2f();
  curStepDuration = calcDynamicStepDuration(currentMovement.speed.translation.x(), currentMovement.speed.translation.y(), currentMovement.speed.rotation);
  PLOT("module:PatternGenerator2017:curStepDurationCalcWalkParams", curStepDuration);
  currentCustomStep = WalkRequest::none;
  customStepKickInPreview = false;
  currentWalkingPhase = unlimitedDoubleSupport;
  currentTimeStamp = 0;
  currentStepFileIndex = -1;
  lastStep.onFloor[0] = lastStep.onFloor[1] = false;
  lastStep.customStepRunning = false;
  leftFootPose2f = lastLeftFootPose2f = Pose2f(0, 0, theWalkingEngineParams.footMovement.footYDistance);
  rightFootPose2f = lastRightFootPose2f = Pose2f(0, 0, -theWalkingEngineParams.footMovement.footYDistance);
  appliedReactiveStep[LEFT_FOOT] = 0;
  appliedReactiveStep[RIGHT_FOOT] = 0;
  localSteps.reset();
  stepsInPreview = 0;

  //ref zmp
  localRefZMP2018.zmpWCS.clear();
  localRefZMP2018.zmpRCS.clear();
  lastRefZMPEndPositionFC = Vector2f::Zero();
  plannedFootSteps.reset();
  currentFootStepTrajectory.clear();
  walkingPhaseExtension = 0;
  comXBuffer.fill(0.f);
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

  switch (currentState)
  {
  case ready:
    //currentWalkingPhase=unlimitedDoubleSupport;
    updatePreview();
    wasReady = true;
    break;

  case standby:
    //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
    step.direction = 0;
    step.footPos[LEFT_FOOT] = 0;
    step.footPos[RIGHT_FOOT] = 0;
    step.footPos[RIGHT_FOOT].z = step.footPos[LEFT_FOOT].z = -startZ;
    step.footPos[RIGHT_FOOT].y = -theWalkingEngineParams.footMovement.footYDistance;
    step.footPos[LEFT_FOOT].y = theWalkingEngineParams.footMovement.footYDistance;
    step.footPos[LEFT_FOOT].r = baseRot;
    step.footPos[RIGHT_FOOT].r = -baseRot;
    break;

  case stopping:
    break;

  case goingToStandby:
    if (crouchingLength > 0)
    {
      factor = ((float)(stateCounter) / crouchingLength);
      //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
      step.direction = 0;
      step.footPos[LEFT_FOOT] = 0;
      step.footPos[RIGHT_FOOT] = 0;
      step.footPos[RIGHT_FOOT].x = CoM.x * factor;
      step.footPos[LEFT_FOOT].x = CoM.x * factor;
      step.footPos[RIGHT_FOOT].z = step.footPos[LEFT_FOOT].z = (1 - factor) * (-startZ) + factor * (-theFLIPMParameter.paramsX.z_h + CoM.z);
      step.footPos[RIGHT_FOOT].y = -theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].y = theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].r = baseRot;
      step.footPos[RIGHT_FOOT].r = -baseRot;
    }
    break;

  case goingToReady:
    if (crouchingLength > 0)
    {
      factor = 1 - ((float)(stateCounter) / crouchingLength);
      //startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
      step.direction = 0;
      step.footPos[LEFT_FOOT] = 0;
      step.footPos[RIGHT_FOOT] = 0;
      step.footPos[RIGHT_FOOT].x = CoM.x * factor;
      step.footPos[LEFT_FOOT].x = CoM.x * factor;
      step.footPos[RIGHT_FOOT].z = step.footPos[LEFT_FOOT].z = (1 - factor) * (-startZ) + factor * (-theFLIPMParameter.paramsX.z_h + CoM.z);
      step.footPos[RIGHT_FOOT].y = -theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].y = theWalkingEngineParams.footMovement.footYDistance;
      step.footPos[LEFT_FOOT].r = baseRot;
      step.footPos[RIGHT_FOOT].r = -baseRot;
    }
    break;

  case walking:
  case customSteps:
    break;


  default:
    break;
  }
  step.phase = unlimitedDoubleSupport;
  return std::move(step);
}

void PatternGenerator2017::updatePreview()
{
  //if (running && desiredPreviewLength == currentPreviewLength)
  //  addStep();

  while ((size_t)desiredPreviewLength > plannedFootSteps.steps.size()) // Add more steps to get a higher preview
  {
    updateWalkPhase();
    addStep();
    // if loop repeats, decrease stateCounter, usually done in updateWalkState()
    if ((size_t)desiredPreviewLength > plannedFootSteps.steps.size())
      stateCounter--;
  }
  //if (stateCounter == 0)
  //  stateCounter = 1;
  // in the other case, i.e more steps than preview length, no steps need to be added
}

unsigned int PatternGenerator2017::singleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (!currentExecutedStep->onFloor[LEFT_FOOT] || !currentExecutedStep->onFloor[RIGHT_FOOT])
      return currentExecutedStep->duration;
    else if (std::next(currentExecutedStep) != currentSteps.steps.end())
      return std::next(currentExecutedStep)->duration;
  }
  return getPhaseLength(1 - theWalkingEngineParams.footMovement.doubleSupportRatio, curStepDuration);
}

unsigned int PatternGenerator2017::doubleSupportDuration()
{
  if (currentStepFileIndex != -1)
  {
    if (currentExecutedStep->onFloor[LEFT_FOOT] && currentExecutedStep->onFloor[RIGHT_FOOT])
      return currentExecutedStep->duration;
    else if (std::next(currentExecutedStep) != currentSteps.steps.end())
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
  if (currentWalkingPhase == firstDoubleSupport) // leftFootPose2f already set to target pos!
    newStep.footPos[LEFT_FOOT] = Point(lastLeftFootPose2f);
  else if (currentWalkingPhase == secondDoubleSupport) // rightFootPose2f already set to target pos!
    newStep.footPos[RIGHT_FOOT] = Point(lastRightFootPose2f);
  newStep.stepDurationInSec = curStepDuration;
  newStep.onFloor[LEFT_FOOT] = (currentWalkingPhase != secondSingleSupport);
  newStep.onFloor[RIGHT_FOOT] = (currentWalkingPhase != firstSingleSupport);

  newStep.footPos[LEFT_FOOT] += sideStepSum[LEFT_FOOT];
  newStep.footPos[RIGHT_FOOT] += sideStepSum[RIGHT_FOOT];

  /////////////////////////////////////////////////////////////////////////////
  /* special swing foot trajectory offset for custom steps */


  /////////////////////////////////////////////////////////////////////////////

  for (int i = 0; i < 2; i++)
  {
    ASSERT(newStep.footPos[i] == newStep.footPos[i]);

    // This makes boom if a foot on the floor is repositioned by more than 1 mm
    // Check transitions from phase to phase including rotation, also custom
    // steps. Same for rotation.
    {
      // TODO: fix this crash
      //ASSERT(!(lastStep.onFloor[i] && newStep.onFloor[i]) || (lastStep.footPos[i].euklidDistance3D(newStep.footPos[i]-appliedReactiveStep[i]) < 0.001 && abs(lastStep.footPos[i].r - newStep.footPos[i].r) < 0.01));
    }
  }

  newStep.direction = direction;
  newStep.customStep = currentCustomStep;
  newStep.inKick = currentStepFileIndex != -1 && currentExecutedStep->kick;
  if (currentStepFileIndex != -1 && currentSteps.kickHackDurationKnee > 0)
  {
    newStep.timeUntilKickHackHip = currentSteps.timeUntilKickHackHip;
    newStep.kickHackDurationHip = currentSteps.kickHackDurationHip;
    newStep.kickHackHipAngle = currentSteps.kickHackHipAngle;
    newStep.kickHackKneeAngle = currentSteps.kickHackKneeAngle;
    newStep.timeUntilKickHackKnee = currentSteps.timeUntilKickHackKnee;
    newStep.kickHackDurationKnee = currentSteps.kickHackDurationKnee;
#ifdef TARGET_SIM
    newStep.timeUntilKickHackKnee *= 2;
    newStep.timeUntilKickHackHip *= 2;
#endif
  }
  newStep.phase = currentWalkingPhase;
  newStep.singleSupportDurationInFrames = singleSupportDuration();
  newStep.doubleSupportDurationInFrames = doubleSupportDuration();
  newStep.frameInPhase = 0;
  if (currentWalkingPhase == firstSingleSupport || currentWalkingPhase == secondSingleSupport)
  {
    newStep.frameInPhase = newStep.singleSupportDurationInFrames - stateCounter;
  }
  if (currentWalkingPhase == firstDoubleSupport || currentWalkingPhase == secondDoubleSupport)
  {
    newStep.frameInPhase = newStep.doubleSupportDurationInFrames - stateCounter;
  }

  newStep.timestamp = currentTimeStamp; // time is the frame number when it is executed after preview
  currentTimeStamp++;

  ASSERT(newStep.frameInPhase < newStep.singleSupportDurationInFrames + newStep.doubleSupportDurationInFrames);

  newStep.speed = currentMovement.speed;
  newStep.customStepRunning = (currentStepFileIndex != -1) && !currentExecutedStep->isPrepend;
  newStep.prependStepRunning = (currentStepFileIndex != -1) && currentExecutedStep->isPrepend;
  newStep.leftFootPose2f = leftFootPose2f;
  newStep.rightFootPose2f = rightFootPose2f;
  newStep.lastLeftFootPose2f = lastLeftFootPose2f;
  newStep.lastRightFootPose2f = lastRightFootPose2f;
  newStep.deltaDirection = deltaDirection[currentWalkingPhase]; // for ref zmp
  addRefZMP2020(newStep);
  newStep.lastRefZMPEndPositionFC = lastRefZMPEndPositionFC;
  // add to buffer w/o custom step trajectory offsets
  localSteps.addStep(newStep);

  // add swing foot trajectory from custom steps to foot position
  if (currentStepFileIndex != -1 && !newStep.onFloor[LEFT_FOOT] && currentExecutedStep->spline.size() > newStep.frameInPhase)
  {
    Point offset = currentExecutedStep->spline[newStep.frameInPhase];
    offset.rotate2D(leftFootPose2f.rotation);
    //newStep.footPos[LEFT_FOOT] += offset;
    lastCustomStepSpline.push_back(offset);
  }
  if (currentStepFileIndex != -1 && !newStep.onFloor[RIGHT_FOOT] && currentExecutedStep->spline.size() > newStep.frameInPhase)
  {
    Point offset = currentExecutedStep->spline[newStep.frameInPhase];
    offset.rotate2D(rightFootPose2f.rotation);
    //newStep.footPos[RIGHT_FOOT] += offset;
    lastCustomStepSpline.push_back(offset);
  }

  bool isDoubleSupport = (newStep.phase == unlimitedDoubleSupport || newStep.phase == firstDoubleSupport || newStep.phase == secondDoubleSupport);
  if (isDoubleSupport && plannedFootSteps.getNumOfSteps() > 1
      && (plannedFootSteps.getStep(plannedFootSteps.getNumOfSteps() - 1).phase == firstSingleSupport || plannedFootSteps.getStep(plannedFootSteps.getNumOfSteps() - 1).phase == secondSingleSupport))
    createFootStepTrajectory();
  if (isDoubleSupport)
  {
    lastCustomStepSpline.clear();
    lastFootSteps.push_back(newStep);
    lastFootStepsRefZMP.push_back(lastStepRefZMP);
    lastStepRefZMP.clear();
  }
  plannedFootSteps.addStep(newStep);

  bool ok = true;
  if (currentStepFileIndex == -1)
  {
    if (newStep.onFloor[LEFT_FOOT])
    {
      if (deltaDirection[currentWalkingPhase] < 0 || newStep.footPos[LEFT_FOOT].r >= direction)
        ok = true;
    }

    if (newStep.onFloor[RIGHT_FOOT])
    {
      if (deltaDirection[currentWalkingPhase] > 0 || newStep.footPos[RIGHT_FOOT].r <= direction)
        ok = true;
    }
  }
  else
    ok = true;

  if (ok)
    direction += deltaDirection[currentWalkingPhase];

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
  lastStep.customStepRunning = newStep.customStepRunning;
  lastStep.prependStepRunning = newStep.prependStepRunning;
}

void PatternGenerator2017::createFootStepTrajectory()
{
  unsigned int numOfSteps = plannedFootSteps.getNumOfSteps();
  if (numOfSteps == 0)
  {
    OUTPUT_ERROR("module:PatternGenerator2017:createFootStepTrajectory - No footsteps!" << currentState << "-" << currentWalkingPhase);
  }
  Footposition curStep = plannedFootSteps.getStep(numOfSteps - 1);
  int footNum = (curStep.phase == firstSingleSupport);

  float yFac = std::abs(curStep.speed.translation.y() * 1000) / (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor);
  float xStepHeight = curStep.speed.translation.x() > 0 ? theWalkingEngineParams.footMovement.stepHeight[0] : theWalkingEngineParams.footMovement.stepHeight[1];
  float walkStepHeight = (1 - yFac) * xStepHeight + yFac * theWalkingEngineParams.footMovement.stepHeight[2];

  float footPitch = theWalkingEngineParams.footMovement.footPitch * sgn(curStep.speed.translation.x());
  float footRoll = theWalkingEngineParams.footMovement.footRoll * sgn(curStep.speed.translation.y());

  // new stuff
  currentFootStepTrajectory.resize(curStep.singleSupportDurationInFrames);
  const int noControlPoints = 7;
  std::vector<Point> controlVector(noControlPoints);
  controlVector[0] = Point();
  for (int i = 1; i < noControlPoints - 1; i++)
  {
    controlVector[i].x = theWalkingEngineParams.footMovement.forwardPolygon[i - 1];
    controlVector[i].y = theWalkingEngineParams.footMovement.sideStepPolygon[i - 1];
    controlVector[i].z = theWalkingEngineParams.footMovement.heightPolygon[i - 1];
    controlVector[i].r = theWalkingEngineParams.footMovement.rotPolygon[i - 1];
  }
  controlVector[noControlPoints - 1] = Point(1.f, 1.f, 0.f, 1.f);
  std::vector<Point> offsetSpline(curStep.singleSupportDurationInFrames);
  BSpline<Point>::bspline(noControlPoints - 1, 3, &(controlVector[0]), &(offsetSpline[0]), curStep.singleSupportDurationInFrames);
  Point offsetForThisFrame;
  int numCustomStepOffsets = static_cast<int>(lastCustomStepSpline.size());
  unsigned int trajectorySteps = std::min(curStep.singleSupportDurationInFrames, numOfSteps);

  if (footNum == LEFT_FOOT)
  {
    Pose2f footDiff = Pose2f(curStep.footPos[LEFT_FOOT].r, curStep.footPos[LEFT_FOOT].x, curStep.footPos[LEFT_FOOT].y) - lastLeftFootPose2f;
    Pose2f footDiffRelative = footDiff;
    footDiffRelative.rotate(-lastLeftFootPose2f.rotation);
    for (unsigned int i = 0; i < trajectorySteps; i++)
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
    for (unsigned int i = 0; i < trajectorySteps; i++)
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
  if (numCustomStepOffsets > 0)
  {
    // we should not have an offset that is too long since we do not reset within custom steps atm
    int numOfOffsetsToAdd = std::min<int>(trajectorySteps, numCustomStepOffsets);
    for (int i = 0; i < numOfOffsetsToAdd; i++)
      plannedFootSteps.steps.at(numOfSteps - i - 1).footPos[footNum] += lastCustomStepSpline.at(numCustomStepOffsets - i - 1);
  }
}

void PatternGenerator2017::updateCoM(Point CoM)
{
  this->CoM = CoM;
}

void PatternGenerator2017::calcSupportFoot()
{
  supportFootBuffer.push_front(theFsrSensorData.calcSupportFoot());
  // get direction of support foot shift
  float direction = supportFootBuffer[0] - 0.5f * supportFootBuffer[1] - 0.5f * supportFootBuffer[2];
  supportFootDirectionBuffer.push_front(direction);
  float directionSum = 0.f;
  float alpha = 0.2f;
  for (int i = 0; i < 5; i++)
  {
    directionSum += alpha * (-1 + 2 * (supportFootDirectionBuffer[i] > 0.f));
    alpha += 0.2f;
  }
  if (supportFootState == bothFeetSupport)
  {
    if (directionSum > 1.f && supportFootBuffer[0] > 0.f)
      supportFootState = leftSupportOnly;
    else if (directionSum < -1.f && supportFootBuffer[0] < 0.f)
      supportFootState = rightSupportOnly;
  }
  else if (supportFootState == leftSupportOnly)
  {
    if (directionSum < -1.f && direction < 0.f)
      supportFootState = bothFeetSupport;
  }
  else if (supportFootState == rightSupportOnly)
  {
    if (directionSum > 1.f && direction > 0.f)
      supportFootState = bothFeetSupport;
  }
  PLOT("module:PatternGenerator2017:supportFoot", (supportFootState == bothFeetSupport) ? 0.f : ((supportFootState == rightSupportOnly) ? -0.1f : 0.1f));
}

void PatternGenerator2017::fillFootSteps()
{
  draw();
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2017:saveCustomSteps")
  saveStepFiles();
  DEBUG_RESPONSE_ONCE("module:PatternGenerator2017:loadCustomSteps")
  loadStepFiles();

  localSteps.reset();

  if (theFallDownState.state != FallDownState::upright)
    reset();
  if (currentState != walking && currentState != customSteps)
    walkingPhaseExtension = 0;

  Point CoM(theRobotModel.centerOfMass.x() / 1000, theRobotModel.centerOfMass.y() / 1000, (theRobotModel.centerOfMass.z()) / 1000, 0);
  updateCoM(CoM);
  calcSupportFoot();

  if (walkingPhaseExtension > 0)
    return;

  updateWalkState();

  localRefZMP2018.reset = false;

  if (useResetPreview && running && resetPreviewPossible)
  {
    previewReset();
  }

  if (currentState == walking || currentState == ready || currentState == stopping || currentState == customSteps)
    updatePreview();

  running = (size_t)desiredPreviewLength == plannedFootSteps.steps.size();

  localSteps.walkState = currentState;

  localSteps.suggestedStep = getNextStep();
  localSteps.running = running;

  speedBuffer.push_front(currentMovement.speed);

  localSteps.time = currentTimeStamp;

  if (useResetPreview && running)
  {
    bool leftStep = (plannedFootSteps.steps[0].phase == secondSingleSupport || plannedFootSteps.steps[0].phase == secondDoubleSupport);
    if (leftStep)
    {
      robotPose2fAfterCurrentStep = Pose2f(plannedFootSteps.steps[0].leftFootPose2f);
      robotPose2fAfterCurrentStep.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
    }
    else
    {
      robotPose2fAfterCurrentStep = Pose2f(plannedFootSteps.steps[0].rightFootPose2f);
      robotPose2fAfterCurrentStep.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
    }
  }

  if (useResetPreview && !plannedFootSteps.steps.empty() && !plannedFootSteps.steps[0].customStepRunning)
  {
    localSteps.robotPoseAfterStep = Point(robotPose2fAfterCurrentStep);
  }
  else
  {
    localSteps.robotPoseAfterStep = Point(robotPose2f);
  }


  if (running)
  {
    LOG("PatternGenerator2017_addStep", "currentWalkingPhase", currentWalkingPhase);
  }

  PLOT("module:PatternGenerator2017:PreviewLength", plannedFootSteps.steps.size());
  PLOT("module:PatternGenerator2017:CurrentSpeedX", currentMovement.speed.translation.x());
  PLOT("module:PatternGenerator2017:CurrentSpeedY", currentMovement.speed.translation.y());
  PLOT("module:PatternGenerator2017:CurrentSpeedR", currentMovement.speed.rotation);

  DEBUG_RESPONSE("debug drawing:module:PatternGenerator2017:leftFoot")
  {
    leftFootPose2f.translation *= 1000.f;
    lastLeftFootPose2f.translation *= 1000.f;
    ARROW("module:PatternGenerator2017:leftFoot",
        leftFootPose2f.translation.x(),
        leftFootPose2f.translation.y(),
        leftFootPose2f.translation.x() + cos(leftFootPose2f.rotation) * 30.f,
        leftFootPose2f.translation.y() + sin(leftFootPose2f.rotation) * 30.f,
        3,
        Drawings::solidPen,
        ColorRGBA::blue);
    ARROW("module:PatternGenerator2017:leftFoot",
        lastLeftFootPose2f.translation.x(),
        lastLeftFootPose2f.translation.y(),
        lastLeftFootPose2f.translation.x() + cos(lastLeftFootPose2f.rotation) * 30.f,
        lastLeftFootPose2f.translation.y() + sin(lastLeftFootPose2f.rotation) * 30.f,
        3,
        Drawings::solidPen,
        ColorRGBA::gray);
    leftFootPose2f.translation /= 1000.f;
    lastLeftFootPose2f.translation /= 1000.f;
  }

  DEBUG_RESPONSE("debug drawing:module:PatternGenerator2017:rightFoot")
  {
    rightFootPose2f.translation *= 1000.f;
    lastRightFootPose2f.translation *= 1000.f;
    ARROW("module:PatternGenerator2017:rightFoot",
        rightFootPose2f.translation.x(),
        rightFootPose2f.translation.y(),
        rightFootPose2f.translation.x() + cos(rightFootPose2f.rotation) * 30.f,
        rightFootPose2f.translation.y() + sin(rightFootPose2f.rotation) * 30.f,
        3,
        Drawings::solidPen,
        ColorRGBA::yellow);
    ARROW("module:PatternGenerator2017:rightFoot",
        lastRightFootPose2f.translation.x(),
        lastRightFootPose2f.translation.y(),
        lastRightFootPose2f.translation.x() + cos(lastRightFootPose2f.rotation) * 30.f,
        lastRightFootPose2f.translation.y() + sin(lastRightFootPose2f.rotation) * 30.f,
        3,
        Drawings::solidPen,
        ColorRGBA::white);

    rightFootPose2f.translation /= 1000.f;
    lastRightFootPose2f.translation /= 1000.f;
  }
  POSE_2D_SAMPLE("module:PatternGenerator2017:robotPoseAfterStep", robotPose2f, ColorRGBA::blue);

  PLOT("module:PatternGenerator2017:fallDownAngleReduction.x", theMotionState.walkingStatus.fallDownSpeedReductionFactor.x());
  PLOT("module:PatternGenerator2017:fallDownAngleReduction.y", theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());

  COMPLEX_DRAWING("module:PatternGenerator2017:drawSteps")
  {
    drawSteps();
  }
  SEND_DEBUG_IMAGE(FootStepsDrawing);

  COMPLEX_DRAWING("module:PatternGenerator2017:drawSteps") drawSteps();
  SEND_DEBUG_IMAGE(FootStepsDrawing);
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
    comXBuffer.push_front(theTorsoMatrix.translation.x());
    // check for DS phase extension
    if ((currentState == walking || currentState == customSteps) && (slowDSRefZMPUntilFSR || freezeDSRefZMPUntilFSR) && walkingPhaseExtension < maxFramesForDSExtension)
    {
      const Footposition& nextFootPosition = plannedFootSteps.steps[0];
      WalkingPhase nextPhase = nextFootPosition.phase;
      if ((nextPhase == firstDoubleSupport && supportFootState == rightSupportOnly) || (nextPhase == secondDoubleSupport && supportFootState == leftSupportOnly))
      {
        walkingPhaseExtension++;
      }
      else
        walkingPhaseExtension = 0;
    }
    else
      walkingPhaseExtension = 0;

    Vector2a orientation = theJoinedIMUData.imuData[anglesource].angle;
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

    if (customStepKickInPreview && plannedFootSteps.getStep(0).customStepRunning) // kick finally executed
      customStepKickInPreview = false;

    if (plannedFootSteps.getStep(0).customStepRunning)
    {
      stepsSinceLastCustomStep = 0;
    }

    resetPreviewPossible = plannedFootSteps.steps.size() > 1 && (!plannedFootSteps.steps[0].prependStepRunning || resetPreviewDuringPrependStep)
        && !plannedFootSteps.steps[0].customStepRunning && (plannedFootSteps.steps[0].phase == firstSingleSupport || plannedFootSteps.steps[0].phase == secondSingleSupport)
        && (plannedFootSteps.steps[0].singleSupportDurationInFrames > 10 && plannedFootSteps.steps[0].frameInPhase == plannedFootSteps.steps[0].singleSupportDurationInFrames - 10);
    if (resetPreviewPossible)
    {
      resetFootPosition = plannedFootSteps.steps[0];
    }
    if (walkingPhaseExtension == 0)
    {
      plannedFootSteps.popFront();
    }
  }
}

void PatternGenerator2017::updateRefZMP2018()
{
  localRefZMP2018.running = running;
  if (running)
  {
    if (walkingPhaseExtension > 0)
    {
      if (slowDSRefZMPUntilFSR && walkingPhaseExtension > 1)
        localRefZMP2018.zmpWCS[0] = localRefZMP2018.zmpWCS[0] * 0.5f + localRefZMP2018.zmpWCS[1] * 0.5f;
    }
    else
    {
      localRefZMP2018.zmpWCS.erase(localRefZMP2018.zmpWCS.begin());
      localRefZMP2018.zmpRCS.erase(localRefZMP2018.zmpRCS.begin());
    }
    // Generate ref zmp from foot steps. Usually just one new frame - except at start of controller.
    while (!localSteps.empty() && walkingPhaseExtension == 0)
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

void PatternGenerator2017::addRefZMP2020(const Footposition& footPosition)
{
  // Ref ZMP calculated in foot coordinate system - origin is below the joints
  // 5cm from back, 10cm to front (Nao V6)
  // takes foot positions before and after phase (only relevant for single support).
  // The foot position is then used to translate it into the WE coordinate system.
  // This means we only calculate the offsets relative to the foot coordinate system here.

  unsigned fullStepDurationInFrames = footPosition.singleSupportDurationInFrames + footPosition.doubleSupportDurationInFrames;
  float timeForFullStep = fullStepDurationInFrames * theFrameInfo.cycleTime;
  unsigned stepDurationInFrames =
      (footPosition.phase == firstSingleSupport || footPosition.phase == secondSingleSupport) ? footPosition.singleSupportDurationInFrames : footPosition.doubleSupportDurationInFrames;
  float timeForStep = stepDurationInFrames * theFrameInfo.cycleTime;

  // Since frameInPhase starts at 0 and we want to be between feet in a 1-frame double support,
  // we can add 0.5 to that to get a better double support transition.
  // positionInCycle is then transformed to be between 0 and 1.
  unsigned frameInPhase = footPosition.frameInPhase + ((footPosition.phase == firstSingleSupport || footPosition.phase == secondSingleSupport) ? footPosition.doubleSupportDurationInFrames : 0);
  float positionInCycle = ((static_cast<float>(footPosition.frameInPhase) + (useHalfFramesInRefZMP ? 0.5f : 0.f)) * theFrameInfo.cycleTime) / timeForStep;
  float positionInCycleXFullStep = ((static_cast<float>(frameInPhase)) * theFrameInfo.cycleTime) / timeForFullStep;
  float positionInCycleX = ((static_cast<float>(frameInPhase)) * theFrameInfo.cycleTime) / timeForStep;

  // We only want the zmp in the central area of the foot, so we limit it to there and
  // make sure, that in the middle of the cycle we cross the 0 position in foot coordinates.
  // For x, we want to go from -1.25cm to 4cm during the cycle.
  // This means a max movement of 5.25cm/step in x for a continious x zmp
  const float maxFootDifferenceXFront = useZeroRefZMP ? 0.f : 0.04f;
  const float maxFootDifferenceXBack = useZeroRefZMP ? 0.f : 0.0125f;

  const Point& footPositionInWEC = footPosition.footPos[ZMP::phaseToZMPFootMap[footPosition.phase]];
  Point footPositionInFC;

  switch (footPosition.phase)
  {
  case firstDoubleSupport:
  {
    // transition from left support to right (the new ref system).
    // Position of left foot should already be next step
    // so that we can infer where the single support refZMP should start
    // first reverse the setStepLength process to get the actual swing foot speed
    Pose2f parallelRightFoot = Pose2f(rightFootPose2f);
    parallelRightFoot.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
    parallelRightFoot.rotate(leftFootPose2f.rotation - rightFootPose2f.rotation);
    parallelRightFoot.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
    Pose2f footDifferenceToParallelFeet = leftFootPose2f - parallelRightFoot;
    float rotationXDifference = std::sin(leftFootPose2f.rotation - rightFootPose2f.rotation) * theWalkingEngineParams.footMovement.footYDistance;
    // first step: transform lastRefZMP from previous foot coordinates to new foot
    if (footPosition.frameInPhase == 0)
    {
      Pose2f lastRefZMPInWEC = lastLeftFootPose2f;
      lastRefZMPInWEC.translate(lastRefZMPEndPositionFC);
      lastRefZMPEndPositionFC.y() = Pose2f(lastRefZMPInWEC - rightFootPose2f).translation.y();
      Pose2f lastRefZMPTranslated = lastRefZMPInWEC;
      lastRefZMPTranslated.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
      lastRefZMPTranslated.rotate(rightFootPose2f.rotation - lastLeftFootPose2f.rotation);
      lastRefZMPTranslated.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
      lastRefZMPEndPositionFC.x() = Pose2f(lastRefZMPTranslated - rightFootPose2f).translation.x();
      if (useRotationForRefZMP)
        lastRefZMPEndPositionFC.x() += std::sin(rightFootPose2f.rotation - lastLeftFootPose2f.rotation) * theWalkingEngineParams.footMovement.footYDistance;
    }
    float endPointX = (footDifferenceToParallelFeet.translation.x() < 0)
        ? std::max(footDifferenceToParallelFeet.translation.x() / 2, -maxFootDifferenceXBack)
        : std::min(footDifferenceToParallelFeet.translation.x() / 2, maxFootDifferenceXFront);
    float startPointX = lastRefZMPEndPositionFC.x();
    footPositionInFC.x = useZeroRefZMP ? (startPointX + endPointX) / 2 : startPointX * (1.f - positionInCycleX) + (endPointX)*positionInCycleX;
    footPositionInFC.x += (useZeroRefZMP || !useRotationForRefZMP) ? 0.f : rotationXDifference * positionInCycleX;
    float endPointY = 0.f; //std::max(std::min(0.01f, footDifferenceToParallelFeet.translation.y() / 10.f), -0.01f);
    float startPointY = lastRefZMPEndPositionFC.y();
    footPositionInFC.y = startPointY * (1.f - positionInCycle) + (endPointY)*positionInCycle;
    if (footPosition.frameInPhase == stepDurationInFrames - 1)
      lastRefZMPEndPositionFC.y() = endPointY;
    break;
  }
  case secondDoubleSupport:
  {
    // transition from right support to left
    // Position of right foot should already be next step
    // so that we can infer where the single support refZMP should start
    Pose2f parallelLeftFoot = Pose2f(leftFootPose2f);
    parallelLeftFoot.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
    parallelLeftFoot.rotate(rightFootPose2f.rotation - leftFootPose2f.rotation);
    parallelLeftFoot.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
    Pose2f footDifferenceToParallelFeet = rightFootPose2f - parallelLeftFoot;
    float rotationXDifference = std::sin(rightFootPose2f.rotation - leftFootPose2f.rotation) * theWalkingEngineParams.footMovement.footYDistance;
    // first step: transform lastRefZMP from previous foot coordinates to new foot
    if (footPosition.frameInPhase == 0)
    {
      Pose2f lastRefZMPInWEC = lastRightFootPose2f;
      lastRefZMPInWEC.translate(lastRefZMPEndPositionFC);
      lastRefZMPEndPositionFC.y() = Pose2f(lastRefZMPInWEC - leftFootPose2f).translation.y();
      Pose2f lastRefZMPTranslated = lastRefZMPInWEC;
      lastRefZMPTranslated.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
      lastRefZMPTranslated.rotate(leftFootPose2f.rotation - lastRightFootPose2f.rotation);
      lastRefZMPTranslated.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
      lastRefZMPEndPositionFC.x() = Pose2f(lastRefZMPTranslated - leftFootPose2f).translation.x();
      if (useRotationForRefZMP)
        lastRefZMPEndPositionFC.x() -= std::sin(leftFootPose2f.rotation - lastRightFootPose2f.rotation) * theWalkingEngineParams.footMovement.footYDistance;
    }
    float endPointX = (footDifferenceToParallelFeet.translation.x() < 0)
        ? std::max(footDifferenceToParallelFeet.translation.x() / 2, -maxFootDifferenceXBack)
        : std::min(footDifferenceToParallelFeet.translation.x() / 2, maxFootDifferenceXFront);
    float startPointX = lastRefZMPEndPositionFC.x();
    footPositionInFC.x = useZeroRefZMP ? (startPointX + endPointX) / 2 : startPointX * (1.f - positionInCycleX) + (endPointX)*positionInCycleX;
    footPositionInFC.x += (useZeroRefZMP || !useRotationForRefZMP) ? 0.f : rotationXDifference * positionInCycleX;
    float endPointY = 0.f; // std::max(std::min(0.01f, footDifferenceToParallelFeet.translation.y() / 10.f), -0.01f);
    float startPointY = lastRefZMPEndPositionFC.y();
    footPositionInFC.y = startPointY * (1.f - positionInCycle) + (endPointY)*positionInCycle;
    if (footPosition.frameInPhase == stepDurationInFrames - 1)
      lastRefZMPEndPositionFC.y() = endPointY;
    break;
  }
  case firstSingleSupport: // left support
  {
    Pose2f parallelLeftFoot = Pose2f(leftFootPose2f);
    parallelLeftFoot.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
    parallelLeftFoot.rotate(rightFootPose2f.rotation - leftFootPose2f.rotation);
    parallelLeftFoot.translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);
    Pose2f footDifferenceToParallelFeet = rightFootPose2f - parallelLeftFoot;
    float rotationXDifference = Pose2f(rightFootPose2f - leftFootPose2f).translation.x();
    float endPointX = (footDifferenceToParallelFeet.translation.x() < 0)
        ? std::max(footDifferenceToParallelFeet.translation.x() / 2, -maxFootDifferenceXBack)
        : std::min(footDifferenceToParallelFeet.translation.x() / 2, maxFootDifferenceXFront);
    float startPointX = lastRefZMPEndPositionFC.x();
    float pc = startRefZMPCycleInDS ? positionInCycleXFullStep : positionInCycleX;
    footPositionInFC.x = useZeroRefZMP ? 0.f : startPointX * (1.f - pc) + (endPointX)*pc;
    footPositionInFC.x += (useZeroRefZMP || !useRotationForRefZMP) ? 0.f : rotationXDifference * positionInCycleX;
    float endPointY = 0.f; // std::max(std::min(0.01f, -footDifferenceToParallelFeet.translation.y() / 10.f), -0.01f);
    footPositionInFC.y = lastRefZMPEndPositionFC.y() + (endPointY - lastRefZMPEndPositionFC.y()) * positionInCycle;
    if (footPosition.frameInPhase == stepDurationInFrames - 1)
      lastRefZMPEndPositionFC = Vector2f(endPointX, endPointY);
    break;
  }
  case secondSingleSupport: // right support
  {
    Pose2f parallelRightFoot = Pose2f(rightFootPose2f);
    parallelRightFoot.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
    parallelRightFoot.rotate(leftFootPose2f.rotation - rightFootPose2f.rotation);
    parallelRightFoot.translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
    Pose2f footDifferenceToParallelFeet = leftFootPose2f - parallelRightFoot;
    float rotationXDifference = Pose2f(leftFootPose2f - rightFootPose2f).translation.x();
    float endPointX = (footDifferenceToParallelFeet.translation.x() < 0)
        ? std::max(footDifferenceToParallelFeet.translation.x() / 2, -maxFootDifferenceXBack)
        : std::min(footDifferenceToParallelFeet.translation.x() / 2, maxFootDifferenceXFront);
    float startPointX = lastRefZMPEndPositionFC.x();
    float pc = startRefZMPCycleInDS ? positionInCycleXFullStep : positionInCycleX;
    footPositionInFC.x = useZeroRefZMP ? 0.f : startPointX * (1.f - pc) + (endPointX)*pc;
    footPositionInFC.x += (useZeroRefZMP || !useRotationForRefZMP) ? 0.f : rotationXDifference * positionInCycleX;
    float endPointY = 0.f; // std::max(std::min(0.01f, -footDifferenceToParallelFeet.translation.y() / 10.f), -0.01f);
    footPositionInFC.y = lastRefZMPEndPositionFC.y() + (endPointY - lastRefZMPEndPositionFC.y()) * positionInCycle;
    if (footPosition.frameInPhase == stepDurationInFrames - 1)
      lastRefZMPEndPositionFC = Vector2f(endPointX, endPointY);
    break;
  }
  case unlimitedDoubleSupport:
  {
    footPositionInFC.y = -theWalkingEngineParams.footMovement.footYDistance;
    lastRefZMPEndPositionFC.y() = 0.f;
    break;
  }
  default:
  {
    footPositionInFC = Vector2f::Zero();
  }
  }

  lastStepRefZMP.push_back(footPositionInFC);

  Point pRCS = footPositionInFC;
  pRCS.rotate2D(-footPosition.direction);
  ZMP zmpRCS = pRCS;
  // TO WEC
  footPositionInFC.rotate2D(footPositionInWEC.r);
  footPositionInFC += footPositionInWEC;

  // save as zmp
  zmp = footPositionInFC;
  zmp.timestamp = footPosition.timestamp;
  localRefZMP2018.zmpWCS.push_back(zmp);
  localRefZMP2018.zmpRCS.push_back(zmpRCS);
}

void PatternGenerator2017::previewReset()
{
  stepsInPreview = 0;
  stepsSinceLastCustomStep = resetFootPosition.stepsSinceCustomStep;
  localRefZMP2018.zmpWCS.clear();
  localRefZMP2018.zmpRCS.clear();
  localRefZMP2018.reset = true;
  leftFootPose2f = resetFootPosition.leftFootPose2f;
  rightFootPose2f = resetFootPosition.rightFootPose2f;
  if (useSafetySteps) // if active, move foot position below upper body
  {
    // TODO: for now only x error relevant
    float comErrorX = theTorsoMatrix.translation.x() - comXBuffer.average();
    if (comErrorX > comErrorFrontForSafetySteps)
    {
      float xCorrection = std::min(maxSafetyStepCorrection.x(), comErrorX) / 1000.f;
      ANNOTATION("PatternGenerator2017", "safetyStep triggered with error " << xCorrection);
      if (plannedFootSteps.steps[0].onFloor[LEFT_FOOT])
        rightFootPose2f.translate(xCorrection, 0.f);
      else
        leftFootPose2f.translate(xCorrection, 0.f);
    }
    else if (comErrorX < comErrorBackForSafetySteps)
    {
      float xCorrection = std::max(-maxSafetyStepCorrection.x(), comErrorX) / 1000.f;
      ANNOTATION("PatternGenerator2017", "safetyStep triggered with error " << xCorrection);
      if (plannedFootSteps.steps[0].onFloor[LEFT_FOOT])
        rightFootPose2f.translate(xCorrection, 0.f);
      else
        leftFootPose2f.translate(xCorrection, 0.f);
    }
  }
  lastLeftFootPose2f = resetFootPosition.lastLeftFootPose2f;
  lastRightFootPose2f = resetFootPosition.lastRightFootPose2f;
  lastStep = resetFootPosition;
  lastStep.onFloor[LEFT_FOOT] = true;
  lastStep.onFloor[RIGHT_FOOT] = true;
  if (plannedFootSteps.steps[0].onFloor[LEFT_FOOT]) // check what the next planned step would have done
  {
    robotPose2f = rightFootPose2f;
    robotPose2f.translate(0, theWalkingEngineParams.footMovement.footYDistance);
    deltaDirection[firstSingleSupport] = resetFootPosition.deltaDirection;
    deltaDirection[firstDoubleSupport] = 0.f;
  }
  else
  {
    robotPose2f = leftFootPose2f;
    robotPose2f.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
    deltaDirection[secondSingleSupport] = resetFootPosition.deltaDirection;
    deltaDirection[secondDoubleSupport] = 0.f;
  }
  stateCounter = resetFootPosition.singleSupportDurationInFrames - resetFootPosition.frameInPhase - 1;
  currentWalkingPhase = resetFootPosition.phase;
  // speeds
  currentMovement.speed = resetFootPosition.speed;
  // ref zmp
  lastRefZMPEndPositionFC = resetFootPosition.lastRefZMPEndPositionFC;
  // reset custom step state - we do not reset within custom step
  currentState = DWE::State::walking;
  lastCustomStepSpline.clear(); // since we are always in walking state atm
  customStepKickInPreview = false;
  currentCustomStep = WalkRequest::StepRequest::none;
  currentStepFileIndex = -1;

  currentTimeStamp = resetFootPosition.timestamp + 1;
  plannedFootSteps.steps.clear();
  direction = resetFootPosition.direction;

  curStepDuration = resetFootPosition.stepDurationInSec;

  lastFootSteps.clear();
  lastFootStepsRefZMP.clear();
}

void PatternGenerator2017::updatePose()
{
  if (!plannedFootSteps.steps.empty())
    footPosBuf.push_front(plannedFootSteps.steps[0]);

  if (!footPosBuf.empty())
  {
    const auto& step = footPosBuf.back();
    // Seems to be necessary for fast scenes, because the robot's velocity
    // is not constant and changes during each step.
    // May not be required when using SelfLocator.
    if (step.phase == firstDoubleSupport || step.phase == secondDoubleSupport)
    {
      stepRobotPose3f = Pose3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 0.f);
      stepRobotPose3f.rotateZ(theRobotPose.rotation);

      poseBetweenLegs = static_cast<Pose3f>((step.footPos[LEFT_FOOT] + step.footPos[RIGHT_FOOT]) / 2.f);
    }
  }
}

void PatternGenerator2017::execute(tf::Subflow&)
{
  updatePose();
  fillFootSteps();
  updateFootPositions();
  updateRefZMP2018();
}

void PatternGenerator2017::update(FootSteps& steps)
{
  steps = localSteps;
}

void PatternGenerator2017::update(Footpositions& footPositions)
{
  footPositions = localFootPositions;
}

void PatternGenerator2017::update(RefZMP2018& refZMP2018)
{
  refZMP2018 = localRefZMP2018;
}

void PatternGenerator2017::update(SpeedInfo& speedInfo)
{
  // TODO: what if length is not static!
  if (PREVIEW_LENGTH != 0 && speedBuffer.size() >= PREVIEW_LENGTH - 1)
  {
    speedInfo.speed = speedBuffer[0];
    speedInfo.speedAfterPreview = speedBuffer[PREVIEW_LENGTH - 1];
    speedInfo.timestamp = currentTimeStamp; // -PREVIEW_LENGTH;
  }
  else
    speedInfo.speed = Pose2f();
  speedInfo.deceleratedByAcc = decelByAcc;
  speedInfo.currentCustomStep = currentCustomStep;
  if (!plannedFootSteps.empty())
    speedInfo.currentCustomStep = plannedFootSteps.steps[0].customStepRunning ? plannedFootSteps.steps[0].customStep : WalkRequest::StepRequest::none;

  speedInfo.lastCustomStep = previousCustomStep;
  speedInfo.lastCustomStepTimestamp = previousCustomStepTimeStamp;
  speedInfo.lastCustomStepMirrored = previousCustomStepMirrored;

  speedInfo.customStepKickInPreview = customStepKickInPreview;
  if (!plannedFootSteps.empty())
    speedInfo.stepsSinceLastCustomStep = plannedFootSteps.steps[0].stepsSinceCustomStep;
  else
    speedInfo.stepsSinceLastCustomStep = 0;

  if (currentCustomStep != WalkRequest::none)
  {
  }

  PLOT("representation:SpeedInfo:speed.x", speedInfo.speed.translation.x() * 1000);
  PLOT("representation:SpeedInfo:speed.y", speedInfo.speed.translation.y() * 1000);
  PLOT("representation:SpeedInfo:speed.r", speedInfo.speed.rotation * 1000);
}

void PatternGenerator2017::update(CustomStepSelection& customStepSelection)
{
  if (!localCustomStepSelection.kicks.empty())
  {
    const auto lowerScoreThan = [](const CustomStepSelection::Kick& a, const CustomStepSelection::Kick& b)
    {
      return a.score < b.score;
    };

    std::sort(localCustomStepSelection.kicks.begin(), localCustomStepSelection.kicks.end(), lowerScoreThan);

    customStepSelection = std::move(localCustomStepSelection);
  }
}

void PatternGenerator2017::loadStepFile(std::string file, int idx)
{
  stepFiles[idx].steps.clear();

  InMapFile s(file);
  if (s.exists())
    s >> stepFiles[idx];
  else
  {
    InMapFile s(file);
    if (s.exists())
      s >> stepFiles[idx];
    else
    {
      OUTPUT(idText, text, std::string("Unable to load ") << file << ". Creating new.");
      stepFiles[idx].steps.push_back(CustomStep());
      saveStepFile(file, idx);
    }
  }

  // Must begin with single support
  ASSERT(stepFiles[idx].steps.size() == 0 || !(stepFiles[idx].steps.front().onFloor[0] && stepFiles[idx].steps.front().onFloor[1]));
}

void PatternGenerator2017::saveStepFile(std::string file, int idx, bool robot)
{

  std::string fullPath;
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

  if (stream.exists())
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

void PatternGenerator2017::drawSteps()
{
  for (int x = 0; x < 640; x++)
    for (int y = 0; y < 480; y++)
      DEBUG_IMAGE_SET_PIXEL_WHITE(FootStepsDrawing, x, y);
  if (lastFootSteps.empty())
    return;

  // determine drawing base
  bool firstStepLeft = (lastFootSteps[0].phase == secondDoubleSupport);
  Pose2f footBase(0.f, 0.f, firstStepLeft ? 50.f : -50.f);
  Pose2f drawingBase(footBase.rotation, 320.f + footBase.translation.x(), 200.f - footBase.translation.y());
  drawFoot(firstStepLeft, drawingBase);

  //CROSS("module:PatternGenerator2017:drawSteps", 320, 240, 10, 2, Drawings::solidPen, ColorRGBA::black);
  COMPLEX_DRAWING("module:PatternGenerator2017:drawSteps")
  {
    for (unsigned int i = 1; i < lastFootSteps.size(); i++)
    {
      bool left = lastFootSteps[i].phase == secondDoubleSupport;
      Pose2f footDiff = left ? (lastFootSteps[i].leftFootPose2f - lastFootSteps[i].rightFootPose2f) : (lastFootSteps[i].rightFootPose2f - lastFootSteps[i].leftFootPose2f);
      footDiff.translation *= 1000.f;
      footBase += footDiff;
      drawingBase = Pose2f(Angle::normalize(footBase.rotation), 320.f + footBase.translation.x(), 200.f - footBase.translation.y());
      drawFoot(left, drawingBase);
      DRAWTEXT("module:PatternGenerator2017:drawSteps", drawingBase.translation.x(), drawingBase.translation.y(), 10, ColorRGBA(0, 0, 0, 150), i);

      for (unsigned int j = 0; j < lastFootStepsRefZMP[i].size(); j++)
      {
        Point zmpRotated(lastFootStepsRefZMP[i][j]);
        zmpRotated.rotate2D(drawingBase.rotation);
        DOT("module:PatternGenerator2017:drawSteps", drawingBase.translation.x() + zmpRotated.x * 1000.f, drawingBase.translation.y() - zmpRotated.y * 1000.f, ColorRGBA::red, ColorRGBA::red);
      }
    }
  }
}

Pose3f PatternGenerator2017::transformWalkToField(const Pose3f& pose) const
{
  const auto diff = [](const Pose3f& base, const Pose3f& ref)
  {
    const auto rot = ref.rotation * base.rotation.inverse();
    return Pose3f(rot, base.rotation.inverse() * (ref.translation - base.translation));
  };

  Pose3f foot3f = diff(poseBetweenLegs, pose);
  foot3f = stepRobotPose3f * foot3f;
  return foot3f;
}

Pose3f PatternGenerator2017::transformFieldToWalk(const Pose3f& pose) const
{
  const auto diff = [](const Pose3f& base, const Pose3f& ref)
  {
    const auto rot = ref.rotation * base.rotation.inverse();
    return Pose3f(rot, base.rotation.inverse() * (ref.translation - base.translation));
  };

  Pose3f foot3f = diff(stepRobotPose3f, pose);
  foot3f = poseBetweenLegs * foot3f;
  return foot3f;
}

void PatternGenerator2017::draw()
{
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:leftFoot", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:rightFoot", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:robotPoseAfterStep", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:PatternGenerator2017:drawSteps", "drawingOnImage");

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
  DECLARE_PLOT("module:PatternGenerator2017:refZMPFull:X");
  DECLARE_PLOT("module:PatternGenerator2017:refZMPFull:Y");
  DECLARE_PLOT("module:PatternGenerator2017:supportFoot");

  DECLARE_DEBUG_DRAWING3D("module:PatternGenerator2017:customSteps", "field");

  COMPLEX_DRAWING3D("module:PatternGenerator2017:customSteps")
  {
    //if (!plannedFootSteps.steps.empty() && (plannedFootSteps.steps[0].customStepRunning || plannedFootSteps.steps[0].prependStepRunning))
    if (!plannedFootSteps.steps.empty() && (footPosBuf.back().customStepRunning || footPosBuf.back().prependStepRunning))
    {
      if (debugRobotPose.translation.x() == 0.f && debugRobotPose.translation.y() == 0.f && debugRobotPose.rotation == 0_deg)
        debugRobotPose = theRobotPose;

      Pose2f pose = debugRobotPose;
      pose.translation = pose.translation / 1000.f;
      ColorRGBA stepColor = ColorRGBA(255, 0, 0, 255);


      unsigned char colorIncrease = static_cast<unsigned char>(255 / currentSteps.steps.size());

      for (const CustomStep& step : currentSteps.steps)
      {
        if (!step.onFloor[LEFT_FOOT])
        {
          pose.translate(step.footPos[LEFT_FOOT].translation + Vector2f(0, theWalkingEngineParams.footMovement.footYDistance));
          pose.rotation += step.footPos[LEFT_FOOT].rotation;

          Pose3f pose3f(pose.translation.x() * 1000.f, pose.translation.y() * 1000.f, 0.f);
          pose3f.rotation = RotationMatrix(0.f, 0.f, pose.rotation);

          FOOT3D("module:PatternGenerator2017:customSteps", pose3f, true, stepColor);
          pose.translate(0, -theWalkingEngineParams.footMovement.footYDistance);
        }
        else if (!step.onFloor[RIGHT_FOOT])
        {
          pose.translate(step.footPos[RIGHT_FOOT].translation - Vector2f(0, theWalkingEngineParams.footMovement.footYDistance));
          pose.rotation += step.footPos[RIGHT_FOOT].rotation;

          Pose3f pose3f(pose.translation.x() * 1000.f, pose.translation.y() * 1000.f, 0.f);
          pose3f.rotation = RotationMatrix(0.f, 0.f, pose.rotation);

          FOOT3D("module:PatternGenerator2017:customSteps", pose3f, false, stepColor);
          pose.translate(0, theWalkingEngineParams.footMovement.footYDistance);
        }

        stepColor.r -= colorIncrease;
        stepColor.b += colorIncrease;
      }
    }
    else
    {
      debugRobotPose = Pose2f();
    }
  }

  DECLARE_DEBUG_DRAWING3D("module:PatternGenerator2017:feet", "field");

  COMPLEX_DRAWING3D("module:PatternGenerator2017:feet")
  {
    const auto& steps = plannedFootSteps.steps;
    if (!steps.empty())
    {
      int counter = debugStepPos;
      for (const auto& step : steps)
      {
        const bool isSingleSupport = step.phase == firstSingleSupport || step.phase == secondSingleSupport;
        const bool isRight = step.phase == secondDoubleSupport || step.phase == firstSingleSupport;
        const Point& footPos = step.footPos[isRight];

        Pose3f foot3f = transformWalkToField(static_cast<Pose3f>(footPos));

        const unsigned char c = debugStepPos == counter ? 127 : 0;
        const unsigned char v = step.prependStepRunning || step.prependStepRunning ? 127 : 255;
        const unsigned char r = counter % 3 == 0 ? v : c;
        const unsigned char g = counter % 3 == 1 ? v : c;
        const unsigned char b = counter % 3 == 2 ? v : c;
        if (!isSingleSupport)
          ++counter;
        const ColorRGBA color = isSingleSupport ? ColorRGBA(r, g, b, 255) : ColorRGBA(0, 0, 0, 255);

        if (isRight)
          FOOT3D("module:PatternGenerator2017:feet", foot3f, false, color);
        else
          FOOT3D("module:PatternGenerator2017:feet", foot3f, true, color);
      }

      if (steps[0].phase == firstDoubleSupport || steps[0].phase == secondDoubleSupport)
        ++debugStepPos;
    }
  }

  DECLARE_DEBUG_DRAWING3D("module:PatternGenerator2017:kickpose", "field");

  COMPLEX_DRAWING3D("module:PatternGenerator2017:kickpose")
  {
    FOOT3D("module:PatternGenerator2017:kickpose", debugKickCurrentPose.translated(0.f, 50.f, 0.f), true, ColorRGBA::red);
    FOOT3D("module:PatternGenerator2017:kickpose", debugKickCurrentPose.translated(0.f, -50.f, 0.f), false, ColorRGBA::red);
    FOOT3D("module:PatternGenerator2017:kickpose", debugKickTargetPose.translated(0.f, 50.f, 0.f), true, ColorRGBA::green);
    FOOT3D("module:PatternGenerator2017:kickpose", debugKickTargetPose.translated(0.f, -50.f, 0.f), false, ColorRGBA::green);
    SPHERE3D("module:PatternGenerator2017:kickpose", debugKickBall.x(), debugKickBall.y(), debugKickBall.z(), 50.f, ColorRGBA::blue);
  }

  DEBUG_RESPONSE_ONCE("module:PatternGenerator2017:plotrefZMPFull")
  {
    for (size_t i = 0; i < localRefZMP2018.zmpWCS.size(); i++)
    {
      PLOT("module:PatternGenerator2017:refZMPFull:X", localRefZMP2018.zmpWCS[i].x());
      PLOT("module:PatternGenerator2017:refZMPFull:Y", localRefZMP2018.zmpWCS[i].y());
    }
  }
}

void PatternGenerator2017::drawFoot(bool left, const Pose2f& baseInImage)
{
  COMPLEX_DRAWING("module:PatternGenerator2017:drawSteps")
  {
    ColorRGBA drawColor = left ? ColorRGBA::gray : ColorRGBA::blue;
    for (unsigned int i = 0; i < FootShape::polygon.size(); i++)
    {
      Vector2f p1 = FootShape::polygon[i];
      Vector2f p2 = FootShape::polygon[(i + 1) % FootShape::polygon.size()];
      if (!left)
      {
        p1.y() = -p1.y();
        p2.y() = -p2.y();
      }
      p1.rotate(baseInImage.rotation);
      p2.rotate(baseInImage.rotation);
      LINE("module:PatternGenerator2017:drawSteps",
          p1.x() + baseInImage.translation.x(),
          -p1.y() + baseInImage.translation.y(),
          p2.x() + baseInImage.translation.x(),
          -p2.y() + baseInImage.translation.y(),
          2,
          Drawings::solidPen,
          drawColor);
    }
  }
}
MAKE_MODULE(PatternGenerator2017, dortmundWalkingEngine)
