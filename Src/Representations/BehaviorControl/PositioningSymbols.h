/**
* \file PositioningSymbols.h
* The file declares a class that containts data about the desired robot's position on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/BehaviorData.h"

/**
* \class PositioningSymbols
* A class that containts data about the desired robot's position on the field.
*/
STREAMABLE(PositioningSymbols,
  /* Drawings, in world coordinate system */
  void draw() const,
  (Pose2f)(Pose2f()) optPosition,
  (bool)(true) stopAtTarget,
  (bool)(true) previewArrival,
  (float)(30.f) thresholdXFront,
  (float)(30.f) thresholdXBack,
  (float)(30.f) thresholdY,
  (Angle)(10_deg) thresholdRotation,
  (float)(10000) distanceToOptPosition,
  (bool)(true) inIllegalPosition,
  (bool)(false) inBallSearch,

  (std::string)("None") log_currState,
  (std::string)("None") log_currObj,
  (std::string)("None") log_obj1,
  (std::string)("None") log_obj2,
  (std::string)("None") log_obj3,
  (std::string)("None") log_obj4,
  (std::string)("None") log_obj5
);
