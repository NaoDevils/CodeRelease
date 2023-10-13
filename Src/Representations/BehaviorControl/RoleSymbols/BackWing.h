#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

STREAMABLE_WITH_BASE(BackWing, PositioningSymbols,
  ,
  (bool)(false) dummy
);
