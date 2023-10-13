#pragma once
#include "PositioningSymbols.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(PositioningAndKickSymbols, PositioningSymbols,
  /** debug drawings, in world coordinate system. */
  void draw() const
  {
    DEBUG_DRAWING("representation:Ballchaser", "drawingOnField")
    {
      POSE_2D_SAMPLE("representation:Ballchaser", optPosition, ColorRGBA::blue);
      ELLIPSE("representation:Ballchaser",
        optPosition.translation,
        (thresholdXBack + thresholdXFront) / 2, thresholdY,
        optPosition.rotation,
        5, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
      ARROW("representation:Ballchaser",
        optPosition.translation.x(), optPosition.translation.y(),
        kickTarget.x(), kickTarget.y(),
        5, Drawings::solidPen, ColorRGBA::red);
}
}
  ,
  (std::string)("None") log_danger,
  (std::string)("None") log_kickName,
  (std::string)("None") log_toBallDistance,

  ((MotionRequest) KickType)(longKick) kickType, /**< Selected kick type. */
  ((WalkRequest) StepRequest)(kickHack) walkKickType, /**< If walkKick (customSteps) is selected, use this id. */
  ((KickRequest) KickMotionID)(KickRequest::KickMotionID::kickInnerFast) longKickType,
  (bool)(false) mirrorKick, /** If true, kick gets mirrored. */
  (bool)(true) useFastKick, /**< If longKick is selected used to select long kick id in kick option. */
  (bool)(true) kickBlind,
  (Vector2f)(Vector2f(1000.f,0.f)) kickTarget, /**< The kick target in world coordinates. */

  (float)(200.f) dribbleSpeedX,
  (float)(0.f) dribbleSpeedY,
  (float)(60.f) kickPosYOffset,
  (bool)(false) blockWayToGoalOnApproach /**< If true, path planning should move between ball and own goal. */
);
