#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/KickEngineKick.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/WalkingEngineKick.h>
#include <Modules/MotionControl/DortmundWalkingEngine/StepData.h>

STREAMABLE(Kicks,
  ,
  (std::vector<KickEngineParameters>) kickEngineParametersVector,
  (std::vector<CustomStepsFile>) customStepFileVector
);