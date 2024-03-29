/**
* @file WalkingInfo.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"
#include "Representations/Modeling/RobotPose.h"
#ifndef WALKING_SIMULATOR
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Debugging/Watch.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
* @class WalkingInfo
* Gives some information about the walk.
*/
STREAMABLE(WalkingInfo,
  StepData lastUsedFootPositions;
  bool isRunning = false;
  bool bodyTiltApplied = false;
  bool onFloor[2];

  Vector2f ballCSinWEWCS;

  void drawFoot(Point s, int t) const
  {
    Vector2f bodyPoints[4];
    bodyPoints[0] = Vector2f(45, 25);
    bodyPoints[1] = Vector2f(45, -25);
    bodyPoints[2] = Vector2f(-45, -25);
    bodyPoints[3] = Vector2f(-45, 25);

    //Point s = f.footPos[ZMP::phaseToZMPFootMap[f.phase]];
    Pose2f p(static_cast<Pose2f>(s));
    p = walkingCStoSelfLocRCS(p);
    for (int i = 0; i < 4; i++)
      bodyPoints[i] = p * bodyPoints[i];
    POLYGON("module:SwingLegController:steps", 4, bodyPoints, 4, Drawings::dashedPen,
      ColorRGBA::black, Drawings::solidBrush, ColorRGBA(static_cast<unsigned char>((t % 9) * 255/9), static_cast<unsigned char>(255 - (t % 9) * 255 / 9), static_cast<unsigned char>((t % 9) * 255 / 9)));
  }

  Pose2f walkingCStoSelfLocRCS(Pose2f p) const
  {
    float rotDiff = -robotPosition.rotation;
    p.translation -= ballCSinWEWCS * 1000;
    p.translation.rotate(rotDiff);
    p.rotate(rotDiff);
    return p;
  }

  Pose2f selfLocRCStoWalkingCS(Pose2f p) const // untested
  {
    float rotDiff = robotPosition.rotation;
    p.translation.rotate(-rotDiff);
    p.rotate(-rotDiff);
    p.translation += ballCSinWEWCS * 1000;
    return p;
  }

  Vector2f ballCStoWalkingCS(Vector2f p) const
  {
    return p.rotate((float)robotPosition.rotation) / 1000 + ballCSinWEWCS;
  }

  Vector2f toWorldCoords(Vector2f &rcs) const
  {
    return robotPosition * rcs;
  }

  Vector2f vecToWorldCoords(Vector2f &rcs) const
  {
    Vector2f null = Vector2f::Zero();
    return toWorldCoords(rcs) - toWorldCoords(null);
  }

  Point vecToWorldCoords(Point &rcs) const
  {
    Point null;
    return toWorldCoords(rcs) - toWorldCoords(null);
  }

  Pose2f toWorldCoords(Pose2f &rcs) const
  {
    Pose2f wcs(0, rcs.translation);
    wcs.rotate(robotPosition.rotation);
    wcs.translation = toWorldCoords(wcs.translation);
    return wcs;
  }

  Point toWorldCoords(const Point &rcs) const
  {
    Point wcs(rcs);
    wcs.rotate2D(robotPosition.rotation);
    return wcs + Point(robotPosition.translation.x(),
      robotPosition.translation.y(),
      0,
      robotPosition.rotation);
  }

  Vector2f toRobotCoords(const Vector2f &wcs) const
  {
    return robotPosition.inverse() * wcs;
  }

  Vector2f vecToRobotCoords(Vector2f &wcs) const
  {
    return toRobotCoords(wcs) - toRobotCoords(Vector2f::Zero());
  }

  Point vecToRobotCoords(const Point &wcs) const
  {
    Point null;
    return toRobotCoords(wcs) - toRobotCoords(null);
  }

  Pose2f toRobotCoords(Pose2f &wcs) const
  {
    Pose2f rcs = Pose2f(0, toRobotCoords(wcs.translation));
    rcs.rotate(-robotPosition.rotation);
    return rcs;
  }

  Point toRobotCoords(const Point &wcs) const
  {
    const Point rp(robotPosition.translation.x(),
      robotPosition.translation.y(),
      0,
      robotPosition.rotation);

    Point rcs = wcs - rp;
    rcs.rotate2D(-rp.r);
    return rcs;
  }
  ,
  (Pose2f) odometryOffset, /**< Distancte between last odometry position and current */
  (Pose2f) robotPosition, /**< Current position of body in world coordinate system of the walking engine */
  (Pose2f) offsetToRobotPoseAfterPreview, /**< Future position of robot after the preview phase */
  (Vector2f)(Vector2f::Zero()) expectedAcc, /**< Expected acceleration of the body */
  (bool)(true) isLeavingPossible, /**< Is is possible to leave the walking engine without falling? */
  (bool)(false) isCustomStepRunning,
  (Vector2f)(Vector2f::Zero()) desiredBodyRot
);
