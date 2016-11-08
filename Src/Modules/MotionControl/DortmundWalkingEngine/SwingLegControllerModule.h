/** 
* @file Modules/MotionControl/DortmundWalkingEngine/SwingLegControllerModule.h
* Module wrapper for the SwingLegController
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "SwingLegController.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/ReferenceModificator.h"

MODULE(SwingLegControllerModule,
{ ,
  REQUIRES(WalkingEngineParams),
  REQUIRES(ControllerParams),
  REQUIRES(FreeLegPhaseParams),
  REQUIRES(FootSteps),
  REQUIRES(MotionRequest),
  REQUIRES(ObservedError),
  REQUIRES(BallModel),
  REQUIRES(PatternGenRequest),
  REQUIRES(FallDownState),
  USES(WalkingInfo),
  PROVIDES_WITHOUT_MODIFY(Footpositions),
  PROVIDES_WITHOUT_MODIFY(ReferenceModificator),
});

class SwingLegControllerModule : public SwingLegControllerModuleBase
{
public:
  SwingLegControllerModule();

  SwingLegController controller;

  void update(Footpositions &footpositions);
  void update(ReferenceModificator &referenceModificator);
};


