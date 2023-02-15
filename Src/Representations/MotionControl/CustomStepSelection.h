#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

STREAMABLE(CustomStepSelection,
  STREAMABLE(Kick,,
    ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) step,
    (bool)(false) mirror,
    (Pose2f)(Pose2f(0.f, 0.f, 0.f)) pose,
    (float)(0.f) score
  );
  ,
  ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) currentStep,
  (CustomStepsFile) currentSteps,
  (Vector2f)(Vector2f::Zero()) ballModel,
  (Pose2f) robotPoseAfterPreviw,
  (std::vector<Kick>) kicks
);
