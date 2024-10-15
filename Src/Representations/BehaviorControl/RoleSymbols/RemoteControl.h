/**
* \file RemoteControl.h
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

STREAMABLE_WITH_BASE(RemoteControl, PositioningAndKickSymbols,
  ///** debug drawings, in world coordinate system. */
  //void draw() const
  //{
  //  DEBUG_DRAWING("representation:RemoteControl", "drawingOnField")
  //  {
  //    POSE_2D_SAMPLE("representation:RemoteControl", optPosition, ColorRGBA::yellow);
  //  }

  //}
  ,
  //(Pose2f)(Pose2f(0.f,0.f)) optPosition, /**< The target position of the robot in field coordinates. */
  //(bool)(false) stopAtTarget,
  //(bool)(false) previewArrival,
  //(float)(50.f) thresholdXFront,
  //(float)(30.f) thresholdXBack,
  //(float)(50.f) thresholdY,
  //(Angle)(10_deg) thresholdRotation,
  (bool)(false) enabled,
  (bool)(false) enforceOffensiveRoles,
  (bool)(false) enforceDefensiveRoles,
  (bool)(false) kickPreference,
  (bool)(false) passPreference,
  (bool)(false) beQuiet,
  (bool)(false) roleDisabled,
  (bool)(false) ballInOwnHalf,
  (bool)(false) ballOnLeftSide,
  (bool)(false) ballSearch,
  (bool)(false) ballSearchLocal,
  (MotionRequest)(MotionRequest()) optMotionRequest,
  (HeadControlRequest)(HeadControlRequest()) optHeadControlRequest,
  (bool)(false) searchForTeammate,
  (bool)(false) passDecision,
  (Vector2f)(Vector2f(0.f,0.f)) remotePassTarget,
  (bool)(false) kickDecision,
  (Vector2f)(Vector2f(0.f,0.f)) remoteKickTarget,
  (bool)(false) blockedByRobot,
  (float)(0.0f) framesSinceLastSideStep
);

STREAMABLE(RemoteControlRequest,
  ENUM(Command,
    enable,
    disable,
    enforceOffensiveRoles,
    enforceDefensiveRoles,
    kickPreference,
    passPreference,
    stand,
    walk,
    kick,
    pass
  );
  ,
  (Command)(Command::stand) command,
  (Pose2f) target,
  (bool)(true) handled
);
