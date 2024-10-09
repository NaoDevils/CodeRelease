#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/MotionControl/MotionRequest.h"

STREAMABLE_WITH_BASE(FrontWing, PositioningSymbols,
  ///** debug drawings, in world coordinate system. */
  //void draw() const
  //{
  //  DEBUG_DRAWING("representation:FrontWing", "drawingOnField")
  //  {
  //    POSE_2D_SAMPLE("representation:FrontWing", optPosition, ColorRGBA::yellow);
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
  (Vector2f)(Vector2f(0.f,0.f))passTarget,
  (bool)(false) kickDecision,
  (Vector2f)(Vector2f(0.f,0.f))kickTarget,
  (bool)(false) blockedByRobot,
  (float)(0.0f) framesSinceLastSideStep
);
