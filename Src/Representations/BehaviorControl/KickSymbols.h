/**
* \file KickSymbols.h
* The file declares a class that containts data about the best kick position, target and type.
* \author Ingmar Schwarz
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/Eigen.h"

/**
* \class KickSymbols
* A class that containts data about the best kick position, target and type.
*/
STREAMABLE(KickSymbols,
{
  void draw() const,
  ((MotionRequest) KickType)(longKick) kickType, /**< Selected kick type. */
  ((WalkRequest) StepRequest)(frontKickShort) walkKickType, /**< If walkKick (customSteps) is selected, use this id. */
  (bool)(false) mirrorKick, /** If true, kick gets mirrored. */
  (bool)(true) useFastKick, /**< If longKick is selected used to select long kick id in kick option. */
  (Vector2f)(Vector2f(1000.f,0.f)) kickTarget, /**< The kick target in world coordinates. */
  (Pose2f)(Pose2f(0.f,0.f)) kickPosition, /**< The target position of the robot in field coordinates. */
  (float)(200.f) dribbleSpeedX,
  (float)(0.f) dribbleSpeedY,
  (float)(60.f) kickPosYOffset,
  (float)(50.f) thresholdXFront,
  (float)(30.f) thresholdXBack,
  (float)(50.f) thresholdY,
  (Angle)(10_deg) thresholdRotation,
  (bool)(true) previewArrival,
  (bool)(false) isBallInOwnPenaltyArea,
  (bool)(false) blockWayToGoalOnApproach, /**< If true, path planning should move between ball and own goal. */
  (bool)(false) goalKickBlocked,
});
