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
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.h
* Generator for foot steps
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once
#include "StepData.h"

#include "Parameters.h"
#include "Point.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "WalkingInformations.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Modeling/BallModel.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Module/Module.h"
#else
#include "bhumanstub.h"
#endif

#define TRANSITION_NOT_POSSIBLE -1
#define OK 0
#define UNKNOWN_ERR	-2
#define WRONG_STATE	-3
#define PI 3.14159265358979

/**
* @class PatternGenerator
* This module generates the target foot positions on ground (not for the swing leg).
*/
class PatternGenerator
{
public:

  // Possible:
  // standby -> ready -> walking -> stopping

  /** 
  * Call this function to change the state. E.g. if we are in "ready" call this
  * function with valid MovementInformation to start walking
  * @param newState The desired new state
  * @param moveInf Movement information for walking
  * @return TRANSITION_NOT_POSSIBLE, if the new state is not possible
  */
  int changeState(State newState, MovementInformation &moveInf);

  /** 
  * Returns the current state.
  * @return The current state.
  */
  State& getCurrentState();

  /**
  * Returns the next feet positions. Must be called every frame if getCurrentState()!=standby!
  * @returns Position for both feet in double support phase, for support leg otherwise.
  */
  StepData getNextStep();

  /**
  * Updates the location of the CoM in robot coordinate system.
  * @param CoM The position of the center of mass in robot coordinate system.
  */
  void updateCoM(Point CoM);


  /** Constructor with all needed source data structures.
  * @param theWalkingEngineParams Walking Engine Parameters.
  * @param thePatternGenRequest The request to the pattern genertor, e.g. the next state.
  * @param theControllerParams Some controller parameters used here.
  * @param theRobotModel Model of the robot. Contains the actual CoM.
  * @param theFallDownState Information about the current state. If the robt has fallen down stop walking engine.
  */
  PatternGenerator(
    const WalkingEngineParams	  &theWalkingEngineParams,
    const PatternGenRequest     &thePatternGenRequest,
    const RobotModel            &theRobotModel,
    const RobotDimensions       &theRobotDimensions,
    const FallDownState         &theFallDownState,
    const ControllerParams      &theControllerParams,
    const MotionSelection       &theMotionSelection,
    const WalkingInfo           &theWalkingInfo,
    const ReferenceModificator  &theReferenceModificator,
    const FrameInfo             &theFrameInfo,
    const FreeLegPhaseParams    &theFreeLegPhaseParams,
    const BallModel             &theBallModel,
    const BallModelAfterPreview &theBallModelAfterPreview);

  /** Returns the length of a current single support phase */
  unsigned int singleSupportDuration();
  /** Returns the length of a current double support phase */
  unsigned int doubleSupportDuration();

  /** Sets the desired pitch */
  void setPitch(float pitch);
  /** Returns the current pitch */
  float getCurrentPitch();

  /**
  * Calculates the required step length for at a given speed.
  * @param speed The desired speed.
  * @return The required step length.
  */
  float getStepLength(float speed);
  /** Is the controller running? */
  bool isWalking();

  /**
  * Call this function to fill the structure with the required foot steps
  * @param steps The structure to fill.
  */
  void updateFootSteps(FootSteps & steps);

  /** Destructor */
  ~PatternGenerator(void);

  /** Resets the generator */
  void reset();

protected:
  const WalkingEngineParams     &theWalkingEngineParams; /**< Set by constructor */
  const PatternGenRequest       &thePatternGenRequest; /**< Set by constructor */
  const RobotModel              &theRobotModel; /**< Set by constructor */
  const RobotDimensions         &theRobotDimensions;
  const FallDownState           &theFallDownState; /**< Set by constructor */
  const ControllerParams        &theControllerParams; /**< Set by constructor */
  const MotionSelection         &theMotionSelection;
  const WalkingInfo             &theWalkingInfo;
  const ReferenceModificator    &theReferenceModificator;
  const FrameInfo               &theFrameInfo;
  const FreeLegPhaseParams      &theFreeLegPhaseParams;
  const BallModel               &theBallModel;
  const BallModelAfterPreview   &theBallModelAfterPreview;

  /** The current state of the pattern generator. */
  State currentState;

  /** The request state */
  State newState;

  /** The current executed speed */
  MovementInformation currentMovement;

  /** The requested speed */
  MovementInformation newMovement;

  /** Current support phase */
  WalkingPhase currentWalkingPhase;
  FootSteps *steps;
  int previewDelta;

  /** Default rotation of feet. Set to 0. */
  float baseRot;

  /** Current pitch of body. */
  float curPitch;

  /** Requested pitch of body. */
  float targetPitch;

  /** Current step duration */
  float curStepDuration;
  /** Is the dynamic step duration activated? */
  bool dynamicDuration;
  Point distanceLeft;
  bool running;

  /** Skip the current frame to reach the preview delta */
  bool skip;

  /** Delay counter to avoid to fast speed changes */
  int speedApplyDelay[3];

  /** Current time */
  int currentTimeStamp;

  /** Time counter in current state */
  unsigned int stateCounter; //FIXME: Value is used uninitialized

  /** Time counter in current support phase */
  unsigned int phaseCounter;

  /**
  * Calculates the length of a support phase.
  * @param ratio The current double support ratio. To calculate the
  *				legnth of a single support phase use 1-ds.
  * @param stepDur The current step duration
  * @return the duration of the phase in frames.
  */
  inline unsigned int getPhaseLength(float ratio, float stepDur);
  /** Updates the phase and state counter */
  void updateCounter();
  /** Handles the requested pitch */
  void handlePitch();
  /** Initialize a new walk phase */
  void initWalkingPhase();
  void applyStandType();
  /** The last added step. */
  Footposition lastStep;

  /** Current position of CoM. */
  Point CoM;

  /**
  * Current position of robot.
  * It is NOT the real position/direction. It is only used to place the feet, and only changed
  * during a phase change.
  */
  Point robotPose;

  Point robotPoseAfterStep;
  int cyclePosCounter;
  int curPreviewLength;
  FreeLegPhase kickOn;
  /** direction is the recent real direction. */
  float direction;

  /** deltaDirection is added to the direction in every frame. */
  float deltaDirection[4];

  /**
  * Modifier added to the robotPose to calculate the step.
  * One modifier for every phase and every foot.
  * First index: phase
  * Second index: foot number / robotPose (LEFT_FOOT, RIGHT_FOOT, ROBOT_POSE)
  */
  Point footModifier[numOfWalkingPhases][3];

  Point sideStepSum[2];

  /** If the ready state was reached at least once, to save the actual walkingheight. */
  bool wasReady;

  /** Contains the walkHeight of the robot in the ready state. */
  float walkHeight;

  /** Add a new step to buffer. */
  void addStep();

  /** Calculate some values for the next state. */
  void calcWalkParams();

  /** Calculate needed step length from given speed. */
  void setStepLength();

  /** Updates the current robot pose. */
  void updateRobotPose();
  /** Stops the robot to leave the walk. */
  void stopRobot(Point &pos);
  void calcFootRotation(int footNum);
  /** Calculates the needed rotational speed of body. */
  float getPitchSpeed();
  /** Applies the speed requested in newMovement. */
  void applySpeed(bool x, bool y, bool r_start, bool r_stop);
  /** Resets the speed apply delays. */
  void resetDelays();
  /** Decreases the speed apply delays. */
  void decreaseDelays();

  /** Calculate optimal preview depending on N and step phase length */
  int getOptimalPreviewLength();

  /** Decrease or increase preview on demand. */
  void handlePreviewDelta(int max = 1000);

  bool isSpeed0() { return 
    (thePatternGenRequest.speed.translation.x() == 0) &
    (thePatternGenRequest.speed.translation.y() == 0) &
    (thePatternGenRequest.speed.rotation == 0); };

  /* SpeedInfo related stuff */
  RingBufferWithSum<Pose2f, ControllerParams::N> speedBuffer;
  bool decelByAcc;
public:
  void updateSpeedInfo(SpeedInfo & speedInfo);
};
