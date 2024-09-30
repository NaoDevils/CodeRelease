#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickTypes/KickEngineKick.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickTypes/WalkingEngineKick.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"

STREAMABLE(Kicks,
  ,
  (std::vector<KickEngineParameters>) kickEngineParametersVector,
  (std::vector<CustomStepsFile>) customStepFileVector
);