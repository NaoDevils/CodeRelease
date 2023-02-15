/**
* \file BallchaserKeeper.h
* The file declares a class that containts data about the desired position of the ballchaserKeeper on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
* \class BallchaserKeeper
* The file declares a class that containts data about the desired position of the ballchaserKeeper on the field.
*/
STREAMABLE(BallchaserKeeper,
    BallchaserKeeper & operator=(const BallchaserKeeper & other)
    {
      if (this == &other)
        return *this;
      optPosition = other.optPosition;
      stopAtTarget = other.stopAtTarget;
      previewArrival = other.previewArrival;
      thresholdXFront = other.thresholdXFront;
      thresholdXBack = other.thresholdXBack;
      thresholdY = other.thresholdY;
      thresholdRotation = other.thresholdRotation;
      return *this;
    }
    /* Drawings, in world coordinate system */
    //void draw() const
    ,
    (Pose2f)(Pose2f()) optPosition,
    (bool)(false) stopAtTarget,
    (bool)(false) previewArrival,
    (float)(30.f) thresholdXFront,
    (float)(30.f) thresholdXBack,
    (float)(30.f) thresholdY,
    (Angle)(10_deg) thresholdRotation,
    (unsigned)(0) timeStampLastWalkKickExecution,
    (Vector2f)(Vector2f()) optKickTarget,
    (float)(60.f) yPosOffsetKick,
    (float)(700.f) kickoutAreaWidth,
    (float)(500.f) kickinAreaHeight,
    (bool)(false) ballInPenaltyArea,
    (bool)(false) ballCloseToLine,
    (bool)(false) ballKickable,
    (bool)(false) leftKick,
    (bool)(false) ballInDanger,
    (bool)(false) controlBall,
    (bool)(false) ballControllable,
    (bool)(false) rightSideOfField,
    (bool)(false) ballInKickoutArea,
    (bool)(false) ballInKickinArea,
    (bool)(false) ballInPenaltyKickinArea,
    (bool)(true) useDangerMap
  );
