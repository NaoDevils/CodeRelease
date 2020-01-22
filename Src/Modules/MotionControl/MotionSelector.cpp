/**
 * @file Modules/MotionControl/MotionSelector.cpp
 * This file implements a module that is responsible for controlling the motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author Jesse Richter-Klug
 */

#include <algorithm>
#include "MotionSelector.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(MotionSelector, motionControl)

PROCESS_LOCAL MotionSelector* MotionSelector::theInstance = 0;

void MotionSelector::stand()
{
  if(theInstance)
    theInstance->forceStand = true;
}

void MotionSelector::move()
{
  if(theInstance)
    theInstance->forceStand = false;
}


void MotionSelector::update(MotionSelection& motionSelection)
{
  static int interpolationTimes[MotionRequest::numOfMotions];
  interpolationTimes[MotionRequest::walk] = 300;
  interpolationTimes[MotionRequest::kick] = 100;
  //interpolationTimes[MotionRequest::dmpKick] = 200;
  interpolationTimes[MotionRequest::specialAction] = 200;
  interpolationTimes[MotionRequest::stand] = 1500;
  interpolationTimes[MotionRequest::keyFrame] = 200;
  //interpolationTimes[MotionRequest::stand] = 600;

  static const int playDeadDelay(2000);

  if(lastExecution)
  {
    MotionRequest::Motion requestedMotion = theMotionRequest.motion;
    if (theMotionRequest.motion == MotionRequest::walk && !theGroundContactState.contact)
    {
      requestedMotion = MotionRequest::walk;
      // TODO
    }

    if ((lastMotion == MotionRequest::walk) && (requestedMotion == MotionRequest::walk) && !forceStand)
      motionSelection.walkRequest = theMotionRequest.walkRequest;
    else
      motionSelection.walkRequest = WalkRequest();

    // check if the target motion can be the requested motion (mainly if leaving is possible)
    if((lastMotion == MotionRequest::walk && 
          (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact || 
            (theMotionSettings.leaveWalkForDive && requestedMotion == theMotionRequest.specialAction &&
              theMotionRequest.specialActionRequest.specialAction >= SpecialActionRequest::numOfBasicMotions &&
              theMotionRequest.specialActionRequest.specialAction < SpecialActionRequest::numOfSpecialActionIDs))) ||
       (lastMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) ||
       (lastMotion == MotionRequest::kick && theKickEngineOutput.isLeavingPossible) || //never immediatly leave kick or get up
      (lastMotion == MotionRequest::stand && theStandEngineOutput.isLeavingPossible))
    {
      motionSelection.targetMotion = requestedMotion;
    }

    if(requestedMotion == MotionRequest::specialAction)
    {
      motionSelection.specialActionRequest = theMotionRequest.specialActionRequest;
    }
    else
    {
      motionSelection.specialActionRequest = SpecialActionRequest();
      if(motionSelection.targetMotion == MotionRequest::specialAction)
        motionSelection.specialActionRequest.specialAction = SpecialActionRequest::numOfSpecialActionIDs;
    }

    const bool afterPlayDead(prevMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead);

    const int bodyInterpolationTime(afterPlayDead ? playDeadDelay : interpolationTimes[motionSelection.targetMotion]);
    interpolate(motionSelection.ratios, MotionRequest::numOfMotions, bodyInterpolationTime, motionSelection.targetMotion);

    if(motionSelection.ratios[MotionRequest::specialAction] < 1.f)
    {
      if(motionSelection.targetMotion == MotionRequest::specialAction)
        motionSelection.specialActionMode = MotionSelection::first;
      else
        motionSelection.specialActionMode = MotionSelection::deactive;
    }
    else
      motionSelection.specialActionMode = MotionSelection::active;

    if(motionSelection.specialActionMode == MotionSelection::active && motionSelection.specialActionRequest.specialAction != SpecialActionRequest::numOfSpecialActionIDs)
      lastActiveSpecialAction = motionSelection.specialActionRequest.specialAction;

  }

  if(lastMotion != motionSelection.targetMotion)
    prevMotion = lastMotion;

  lastMotion = motionSelection.targetMotion;

  PLOT("module:MotionSelector:ratios:walk", motionSelection.ratios[MotionRequest::walk]);
  PLOT("module:MotionSelector:ratios:specialAction", motionSelection.ratios[MotionRequest::specialAction]);
  PLOT("module:MotionSelector:ratios:stand", motionSelection.ratios[MotionRequest::stand]);
  PLOT("module:MotionSelector:ratios:kick", motionSelection.ratios[MotionRequest::kick]);
  PLOT("module:MotionSelector:ratios:keyFrame", motionSelection.ratios[MotionRequest::keyFrame]);
  PLOT("module:MotionSelector:lastMotion", lastMotion);
  PLOT("module:MotionSelector:prevMotion", prevMotion);
  PLOT("module:MotionSelector:targetMotion", motionSelection.targetMotion);

  lastExecution = theFrameInfo.time;

  for (unsigned int i = 0; i < MotionRequest::numOfMotions; i++)
    if (motionSelection.ratios[i] > 0.f)
      motionSelection.timeStampLastExecuted[i] = theFrameInfo.time;

#ifndef NDEBUG
  const Rangef& ratioLimits = Rangef::ZeroOneRange();
  for(int i = 0; i < MotionRequest::numOfMotions; ++i)
    ASSERT(ratioLimits.isInside(motionSelection.ratios[i]));
#endif
}

void MotionSelector::interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion)
{
  // increase / decrease all ratios according to target motion
  const unsigned deltaTime(theFrameInfo.getTimeSince(lastExecution));
  float delta(static_cast<float>(deltaTime) / interpolationTime);
  ASSERT(SystemCall::getMode() == SystemCall::logfileReplay || delta > 0.00001f);
  float sum(0);
  for(int i = 0; i < amount; i++)
  {
    if(i == targetMotion)
      ratios[i] += delta;
    else
      ratios[i] -= delta;
    ratios[i] = std::max(ratios[i], 0.0f); // clip ratios
    sum += ratios[i];
  }
  ASSERT(sum != 0);
  // normalize ratios
  for(int i = 0; i < amount; i++)
  {
    ratios[i] /= sum;
    if(std::abs(ratios[i] - 1.f) < 0.00001f)
      ratios[i] = 1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
  }
}
