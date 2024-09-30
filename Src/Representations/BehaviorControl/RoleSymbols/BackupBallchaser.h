/**
* \file BackupBallchaser.h
* The file declares a class that containts data about the desired position of the backupBallchaser on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

/**
* \class BackupBallchaser
* The file declares a class that containts data about the desired position of the backupBallchaser on the field.
*/
STREAMABLE_WITH_BASE(BackupBallchaser, PositioningSymbols,
  ,
  (bool)(false) dummy,
  (bool)(false) blockedByRobot,
  (float)(0.0f) framesSinceLastSideStep
    
  );
