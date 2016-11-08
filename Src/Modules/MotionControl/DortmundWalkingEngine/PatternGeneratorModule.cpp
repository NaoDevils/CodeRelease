/** 
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.cpp
* Module wrapper for PatternGenerator
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "PatternGeneratorModule.h"


PatternGeneratorModule::PatternGeneratorModule():
  patternGen(
             theWalkingEngineParams,
             thePatternGenRequest,
             theRobotModel,
             theRobotDimensions,
             theFallDownState,
             theControllerParams,
             theMotionSelection,
             theWalkingInfo,
             theReferenceModificator,
             theFrameInfo,
             theFreeLegPhaseParams,
             theBallModel,
             theBallModelAfterPreview)
{
}

void PatternGeneratorModule::update(FootSteps& steps)
{
  patternGen.updateFootSteps(steps);
}


void PatternGeneratorModule::update(SpeedInfo& speedInfo)
{
	patternGen.updateSpeedInfo(speedInfo);
}

MAKE_MODULE(PatternGeneratorModule, dortmundWalkingEngine) 
