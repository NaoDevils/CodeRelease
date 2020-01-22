/**
 * @file Modules/MotionControl/MotionSelector.h
 * This file declares a module that is responsible for controlling the motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author Jesse Richter-Klug (updated the arm things)
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(MotionSelector,
{,
  USES(SpecialActionsOutput),
  USES(WalkingEngineOutput),
  USES(KickEngineOutput),
  USES(StandEngineOutput),
  REQUIRES(FrameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSettings),
  REQUIRES(GroundContactState),
  PROVIDES(MotionSelection),
  REQUIRES(MotionSelection),
});

class MotionSelector : public MotionSelectorBase
{
private:
  static PROCESS_LOCAL MotionSelector* theInstance; /**< The only instance of this module. */

  bool forceStand = false;
  MotionRequest::Motion lastMotion;
  MotionRequest::Motion prevMotion;
  unsigned lastExecution; //FIXME: Value is used uninitialized
  SpecialActionRequest::SpecialActionID lastActiveSpecialAction;

  void update(MotionSelection& motionSelection);

  void interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion);

public:
  /**
  * Can be used to overwrite all other motion requests with a stand request.
  * Must be called again in every frame a stand is desired.
  */
  static void stand();
  static void move();
  
  MotionSelector() : 
    lastMotion(MotionRequest::specialAction), prevMotion(MotionRequest::specialAction),
    lastActiveSpecialAction(SpecialActionRequest::playDead)
  {
    theInstance = this;
  }
};
