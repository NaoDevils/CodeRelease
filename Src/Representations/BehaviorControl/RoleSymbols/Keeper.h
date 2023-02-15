/**
* \file Keeper.h
* The file declares a class that containts data about the desired position of the keeper on the field.
*/

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"


/**
* \class Keeper
* The file declares a class that containts data about the desired position of the keeper on the field.
*/
STREAMABLE_WITH_BASE(Keeper, PositioningSymbols,

  ENUM(KeeperBallSearchState,
    none,
    wait,
    supporter,
    movingBall,
    locate,
    penalty
  );
  /* Drawings, in world coordinate system */
  //void draw() const
  ,
  (bool)(false) prepared,
  (KeeperBallSearchState)(KeeperBallSearchState::none) ballSearchState,
  (bool)(false) catchBall, 
  (unsigned)(0) timeOfLastDive,
  (Vector2f)(Vector2f()) optKickTarget,
  (bool)(false) isBallchaser,
  (bool)(false) useLongKick,
  ((WalkRequest) StepRequest)(WalkRequest::StepRequest::any) walkKick,
  ((KickRequest) KickMotionID)(KickRequest::KickMotionID::kickMiddle) longKick
);
