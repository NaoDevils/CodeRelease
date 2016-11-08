/*
Copyright 2011, Oliver Urbann
All rights reserved.

This file is part of MoToFlex.

MoToFlex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MoToFlex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

Contact e-mail: oliver.urbann@tu-dortmund.de
*/

#include "SwingLegController.h"
#include "Tools/Math/Bspline.h"
#include "Tools/Math/interpolator.h"
#include "Point.h"
#include "Tools/Streams/RobotParameters.h"
#include <vector>

#ifndef WALKING_SIMULATOR
//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

#define PRINT_POINT  if (debug) OUTPUT(idText, text, "" << kickPol.back().x << ", " << kickPol.back().y << ", " << kickPol.back().z << ", " << kickPol.back().rx << ", " << kickPol.back().ry << ", " << kickPol.back().r );

// Starts a new foot reset polygon
// pointcount is the number of points between start and end
#define START_POLYGON(pointcount, start, end, fP) { \
  float footPitch = fP; \
  int counter = 1; \
  int pc = pointcount; \
  Point *polygon = new Point[pc + 2]; \
  polygon[0] = start; \
  polygon[pc + 1] = end;

// Defines the next point
// translation is the rate of the full translation from the start to the target
// height is the rate of the full step height
#define POINT(translation, height_factor, max_height) \
  polygon[counter] = polygon[0] + (polygon[pc + 1] - polygon[0])*translation; \
  polygon[counter].z = polygon[0].z + height_factor*max_height; \
  polygon[counter].ry = polygon[0].ry + height_factor*footPitch; \
  polygon[counter].rx = polygon[0].rx + height_factor*footPitch; \
  counter++;

// Or use this function to define the coordinates by hand.
// But do not mix this with POINT.
#define POINT_XY(_x, _y, height_factor, max_height) \
  polygon[counter].x = _x; \
  polygon[counter].y = _y; \
  polygon[counter].z =/*polygon[0].z+*/height_factor*max_height; \
  polygon[counter].ry = polygon[0].ry + height_factor*footPitch; \
  polygon[counter].r = (1 - counter / pc)*polygon[0].r + (1 - counter / pc)*polygon[pc + 1].r; \
  counter++;

// Writes the curve to output 
#define END_POLYGON(output, count, degree) \
  bspline(pc + 1, degree, polygon, output, count); } \
  // Check the curve in Matlab by using:
// >> plot(data(:,29), data(:,31))
// >> axis([0 0.1 -0.245 0.1-0.245])

void SwingLegController::reset()
{
  phases.clear();
  footToModify = -1;
  lastKick = 5;
  currentKickPhase = freeLegNA;
}

bool SwingLegController::initKick(int footNum, Point &endFootPos, bool longKick)
{
#ifndef WALKING_SIMULATOR

  if (lastKick < 6 && !longKick)
  {
    lastKick++;
    return false;
  }

  WalkingEngineParams curparams = (WalkingEngineParams &)theFreeLegPhaseParams;

  if (!longKick)
    curparams = theWalkingEngineParams;

  Vector2f endEffectorOffset(-0.01f - 0.06f, 0); // -0.06-0.03, -0.01-0.03
  endEffectorOffset.rotate(theWalkingInfo.robotPosition.rotation);
  Vector2f ballPos, ballPosWEWCS;

  float kickDirection = theMotionRequest.walkRequest.kickDirection;
  if (footNum == LEFT_FOOT) // Robot stands on right foot?
  {
    if (kickDirection > curparams.rotMax)
      kickDirection = curparams.rotMax;
    if (kickDirection < curparams.rotMin)
      kickDirection = curparams.rotMin;
  }
  else
  {
    if (kickDirection < -1 * curparams.rotMax)
      kickDirection = -1 * curparams.rotMax;
    if (kickDirection > -1 * curparams.rotMin)
      kickDirection = -1 * curparams.rotMin;
  }

  /* Fixme:
  (gdb) p kickStartLen
  $20 = 0
  (gdb) p normDistToRotMax
  $21 = 1.33333337
  (gdb) p distToRotMax
  $22 = <optimized out>
  (gdb) p footNum
  $23 = 1
  (gdb) p kickDirection
  $24 = 0.5
  (gdb) p theWalkingEngineParams.rotMax
  $25 = 1.5
  (gdb)
  */

  float distToRotMax = (float)(std::abs((1 - footNum * 2) * theWalkingEngineParams.rotMax - kickDirection));

  // Normalize the distance
  float normDistToRotMax = distToRotMax / theWalkingEngineParams.rotMax;

  // It is possible that it is larger than one, since it can be rotated into the other direction.
  // However, we want only to measure "how inner" the kick is, so everything outer is just 1.
  if (normDistToRotMax > 1)
    normDistToRotMax = 1.f;



  if (theMotionRequest.walkRequest.kickStrength != 0)
  {

    ballPos = theBallModel.estimate.position;
    if (normDistToRotMax < 0.5f)
      ballPos = Vector2f(theWalkingEngineParams.fixedBallPos[0], theWalkingEngineParams.fixedBallPos[1]);

    LOG("Kick", "BallPos.x", theBallModel.estimate.position.x);
    LOG("Kick", "BallPos.y", theBallModel.estimate.position.y);
    LOG("Kick", "clipped BallPos.x", ballPos.x);
    LOG("Kick", "clipped BallPos.y", ballPos.y);

    ballPosWEWCS = ballPos.rotate(theWalkingInfo.robotPosition.rotation) / 1000 + theWalkingInfo.ballCSinWEWCS + endEffectorOffset;

    LOG("Kick", "WCS BallPos.x", ballPos.x);
    LOG("Kick", "WCS BallPos.y", ballPos.y);
  }
  else
  {
    ballPosWEWCS.x() = phases.front().members.front().footPos[footNum].x;
    ballPosWEWCS.y() = phases.front().members.front().footPos[footNum].y;

    MARK("Kick", "BallPos.x");
    MARK("Kick", "BallPos.y");
    MARK("Kick", "clipped BallPos.x");
    MARK("Kick", "clipped BallPos.y");
    MARK("Kick", "WCS BallPos.x");
    MARK("Kick", "WCS BallPos.y");
  }



  float kickStartIn = normDistToRotMax * curparams.kickStart[0] + (1 - normDistToRotMax) * curparams.kickStart[1]; // Distance before the ball (in kick direction) [m]
  float kickStopIn = normDistToRotMax * curparams.kickStop[0] + (1 - normDistToRotMax) * curparams.kickStop[1]; // Distance after the ball [m]


  // Changed from using theWalkinEngineParams to curparams, seems to be a bug?!
  kickStartLen = (int)(normDistToRotMax * curparams.kickStartLen[0] + (1 - normDistToRotMax) * curparams.kickStartLen[1]);
  kickStopLen = (int)(normDistToRotMax * curparams.kickStopLen[0] + (1 - normDistToRotMax) * curparams.kickStopLen[1]);

  if (theMotionRequest.walkRequest.kickStrength == 0)
  {
    kickStartIn = kickStopIn = 0;
  }


  // Ball left = leg left, otherwise we have to use the right leg
  if ((ballPos.y() >= 0 && footNum == RIGHT_FOOT) ||
    (ballPos.y() < 0 && footNum == LEFT_FOOT))
  {
    // wrong foot
  }

  Vector2f kickStart2D(static_cast<float>(kickStartIn), 0);
  kickStart2D.rotate(kickDirection); // Kickstart in robot coordinate system
  kickStart2D.y() *= 1.7f; // Extend the y start since we must move the foot around the ball without touching it!
  kickStart2D = kickStart2D.rotate(theWalkingInfo.robotPosition.rotation) + ballPosWEWCS; // auf rotation!=0 achten bei ballPos
  kickStart.x() = kickStart2D.x();
  kickStart.y() = kickStart2D.y();
  kickStart.z() = curparams.stepHeight[0];

  LOG("Kick", "kickStart.x", kickStart.x);
  LOG("Kick", "kickStart.y", kickStart.y);
  LOG("Kick", "kickStart.z", kickStart.z);

  if (theMotionRequest.walkRequest.kickStrength != 0)
  {
    Vector2f kickVec2D(std::abs(kickStopIn - kickStartIn), 0);
    kickVec2D.rotate((kickDirection + theWalkingInfo.robotPosition.rotation));
    kickVec.x() = kickVec2D.x();
    kickVec.y() = kickVec2D.y();
    kickVec.z() = 0;
  }
  else
    kickVec = Vector3f::Zero();

  kickStop = kickVec + kickStart;

  LOG("Kick", "kickTime", theMotionRequest.kickTime);
  LOG("Kick", "kickDirection", kickDirection);

  LOG("Kick", "kickVec.x", kickVec.x);
  LOG("Kick", "kickVec.y", kickVec.y);

  ballPos = theBallModel.estimate.position;
  ballPosWEWCS = ballPos.rotate(theWalkingInfo.robotPosition.rotation) / 1000 + theWalkingInfo.ballCSinWEWCS + endEffectorOffset;

  Vector2f distToBall = ballPosWEWCS - Vector2f(endFootPos.x, endFootPos.y);
  distToBall.rotate(-theWalkingInfo.robotPosition.rotation);

  // Not so easy since the Max and Min must be mirrored when using the other foot. Settings are for
  // right foot. And remember, footNum is the standing foot, not the kicking foot.
  // footNum = 1, Min = -0.7, Ist = 0.6: 0.6 - -1 * -0.7 <=> 0.6 - 0.7
  // footNum = 0: -0.6 - 1 * -0.7 <=> -0.6 + 0.7
  // At the moment only max is used.
  // float distToRotMin =  std::abs(theMotionRequest.kickDirection - (1 - footNum * 2) * theWalkingEngineParams.rotMin);

  // footNum = 1, Max = 1.5, Ist = -1.4: -1 * 1.5 - -1.4 <=> -1.5 + 1.4
  // footNum = 0: 1 * 1.5 - 1.4 <=>  1.5 - 1.4

  float yMax = curparams.ballYMax[0];
  float yMin = curparams.ballYMin[0];
  float xMax = curparams.ballXMax[0];
  float xMin = curparams.ballXMin[0];
  // In this is true, then the foot is rotated for an "inner" kick. Then the ball cannot be at an "outer"
  // position. It should be more in the middle of the robot.
  // Set the maximum/minimum for y depending on the desired direction
  yMax = normDistToRotMax * curparams.ballYMax[0] + (1 - normDistToRotMax) * curparams.ballYMax[1];
  yMin = normDistToRotMax * curparams.ballYMin[0] + (1 - normDistToRotMax) * curparams.ballYMin[1];
  xMax = normDistToRotMax * curparams.ballXMax[0] + (1 - normDistToRotMax) * curparams.ballXMax[1];
  xMin = normDistToRotMax * curparams.ballXMin[0] + (1 - normDistToRotMax) * curparams.ballXMin[1];


  return
    distToBall.x() < xMax &&
    distToBall.x() > xMin &&
    (1 - footNum * 2) * distToBall.y() < yMax && // the smaller/larger comparison is exchanged when using other foot
    (1 - footNum * 2) * distToBall.y() > yMin;   // so both sides are multiplied with -1 in that case
#endif
}

void SwingLegController::PlanFootReset(int footNum)
{
  FootpositionListElement &curPos = phases.front().members.front();
  float yFac;
  yFac =  std::abs(curPos.speed.y * 1000) / theFreeLegPhaseParams.maxSpeedY;
  float freeLegStepHeight = (1 - yFac) * theFreeLegPhaseParams.stepHeight[0] + yFac * theFreeLegPhaseParams.stepHeight[1];
  yFac =  std::abs(curPos.speed.y * 1000) / theWalkingEngineParams.maxSpeedY;
  float walkStepHeight = (1 - yFac) * theWalkingEngineParams.stepHeight[0] + yFac * theWalkingEngineParams.stepHeight[1];

  if (curPos.phase == unlimitedDoubleSupport)
    lastKick = 5;

  if (footToModify == footNum &&
    !curPos.instantKickRunning)
  {
    // Beende Modifikation, wenn Fuss wieder in Luft
    if (!curPos.onFloor[footToModify])
      footToModify = -1;
    else
      // Sonst addiere Modifikator
      curPos.footPos[footToModify] += modifier;
  }
  
      
  // Full single support starting after current frame detected, 
  // so plan foot replacement
  if (phases.front().members.size() == 1 &&
      ++phases.begin() != phases.end() &&
      !(++phases.begin())->members.front().onFloor[footNum]
    )
  {
    int len = static_cast<int>((++phases.begin())->members.size());
    Point *output = new Point[len];
    FootList::iterator itMem = (++phases.begin())->members.begin();
    for (int i = 0; i < len; i++, itMem++)
      output[i] = itMem->footPos[footNum];
    Point polygonStart = curPos.footPos[footNum];
    
    // This makes boom if a single support phase cannot be planned.
    // Possible reason: step longer (duration) than preview
    ASSERT(++++phases.begin() != phases.end());
    
    Point polygonEnd = (++++phases.begin())->members.front().footPos[footNum];
    int idx = std::abs(theMotionRequest.walkRequest.kickDirection) > 0.7f;
    bool longKick = (++++phases.begin())->members.front().kickPhase == starting && 
                    ((
                    // Check for right foot
                    (theMotionRequest.walkRequest.kickDirection >= 0 &&
                    footNum == RIGHT_FOOT &&
                    theBallModel.estimate.position.y() / 1000 > -theFreeLegPhaseParams.ballYMax[idx] &&
                    theBallModel.estimate.position.y() / 1000 < -theFreeLegPhaseParams.ballYMin[idx]) ||

                    // Check for left foot
                    (theMotionRequest.walkRequest.kickDirection <= 0  && footNum == LEFT_FOOT &&
                    theBallModel.estimate.position.y() / 1000 < theFreeLegPhaseParams.ballYMax[idx] &&
                    theBallModel.estimate.position.y() / 1000 > theFreeLegPhaseParams.ballYMin[idx])) &&

                    // Check x
                    theBallModel.estimate.position.x() / 1000 < theFreeLegPhaseParams.ballXMax[idx] + 0.06 &&
                    std::abs(theBallModel.estimate.position.y() / 1000) < theFreeLegPhaseParams.ballYMax[idx] &&
                    theBallModel.estimate.position.x() / 1000 > theFreeLegPhaseParams.ballXMin[idx]);

    if (longKick)
    {
      Point p[6];
      if (std::abs(theMotionRequest.walkRequest.kickDirection) < 0.7f)
      {
        p[0] = polygonStart;
        p[1] = polygonStart;
        p[2] = Point(theFreeLegPhaseParams.kickStop[0], 0).rotate2D(theMotionRequest.walkRequest.kickDirection + theWalkingInfo.robotPosition.rotation) + polygonEnd;
        p[3] = Point(theFreeLegPhaseParams.kickStop[0], 0).rotate2D(theMotionRequest.walkRequest.kickDirection + theWalkingInfo.robotPosition.rotation) + polygonEnd;
        p[4] = polygonEnd;
        p[5] = polygonEnd;
      }
      else
      {
        Point ballDiffToCenter = Point(0, theBallModel.estimate.position.y() / 1000 - (1 - 2 * footNum) * theFreeLegPhaseParams.footYDistance).rotate2D(theWalkingInfo.robotPosition.rotation);
        Point yDist = Point(0, (1 - 2 * footNum) * 0.1f).rotate2D(theWalkingInfo.robotPosition.rotation);
        p[0] = polygonStart;
        p[1] = polygonStart + yDist + ballDiffToCenter;
        p[2] = ((polygonStart - polygonEnd) * 0.5f).rotate2D(theMotionRequest.walkRequest.kickDirection) + polygonEnd + ballDiffToCenter;
        p[3] = ((polygonStart - polygonEnd) * 0.33f).rotate2D(theMotionRequest.walkRequest.kickDirection) + polygonEnd + ballDiffToCenter;
        p[4] = Point(theFreeLegPhaseParams.kickStop[1], 0).rotate2D(theMotionRequest.walkRequest.kickDirection + theWalkingInfo.robotPosition.rotation) + polygonEnd + ballDiffToCenter;
        p[5] = Point(theFreeLegPhaseParams.kickStop[1], 0).rotate2D(theMotionRequest.walkRequest.kickDirection + theWalkingInfo.robotPosition.rotation) + polygonEnd + ballDiffToCenter;
      }

      START_POLYGON(5, polygonStart, polygonEnd, theFreeLegPhaseParams.footPitch);
      POINT_XY(p[0].x, p[0].y, theFreeLegPhaseParams.heightPolygon[0], freeLegStepHeight);
      POINT_XY(p[1].x, p[1].y, theFreeLegPhaseParams.heightPolygon[1], freeLegStepHeight);
      POINT_XY(p[2].x, p[2].y, theFreeLegPhaseParams.heightPolygon[2], freeLegStepHeight);
      POINT_XY(p[3].x, p[3].y, theFreeLegPhaseParams.heightPolygon[2], freeLegStepHeight);
      POINT_XY(p[4].x, p[4].y, theFreeLegPhaseParams.heightPolygon[3], freeLegStepHeight);
      POINT_XY(p[5].x, p[5].y, theFreeLegPhaseParams.heightPolygon[4], freeLegStepHeight);
      END_POLYGON(output, len, POLYNOM_DEGREE);
      currentKickPhase = curPos.kickPhase;
    }
    else if (initKick(!footNum, polygonEnd, longKick) && // must be the first so that it is surely called!
      (theMotionRequest.walkRequest.kickStrength < 0))
    {

      modifier.x = (kickStop.x() - polygonEnd.x) * theWalkingEngineParams.stepOffsetFactor;
      modifier.y = (kickStop.y() - polygonEnd.y) * theWalkingEngineParams.stepOffsetFactor;
      polygonEnd += modifier;
      footToModify = footNum;
      lastModified = -1;

      int kickTime = len - kickStartLen - kickStopLen - 1;

      if (kickTime < 1)
        return;
      Point _kickStart(kickStart.x(), kickStart.y(), kickStart.z(), polygonStart.r);
      Point _kickStop(kickStop.x(), kickStop.y(), kickStop.z(), polygonEnd.r);
      START_POLYGON(2, polygonStart, _kickStart, theFreeLegPhaseParams.footPitch);
      POINT_XY(polygonStart.x, polygonStart.y, theFreeLegPhaseParams.heightPolygon[0], kickStart.z());
      POINT_XY(kickStart.x(), kickStart.y(), theFreeLegPhaseParams.heightPolygon[1], kickStart.z());
      END_POLYGON(output, kickStartLen, 2);

      START_POLYGON(2, _kickStart, _kickStop, theFreeLegPhaseParams.footPitch);
      POINT_XY(kickStart.x(), kickStart.y(), theFreeLegPhaseParams.heightPolygon[2], kickStop.z());
      POINT_XY(kickStop.x(), kickStop.y(), theFreeLegPhaseParams.heightPolygon[2], kickStop.z());
      END_POLYGON(&output[kickStartLen], kickTime, 2);

      START_POLYGON(2, _kickStop, polygonEnd, theFreeLegPhaseParams.footPitch);
      POINT_XY(kickStop.x(), kickStop.y(), theFreeLegPhaseParams.heightPolygon[3], kickStop.z());
      POINT_XY(polygonEnd.x, polygonEnd.y, theFreeLegPhaseParams.heightPolygon[4], kickStop.z());
      END_POLYGON(&output[kickStartLen + kickTime], kickStopLen, 2);


      lastKick = 0;
      for (FootList::iterator fp = (++phases.begin())->members.begin();
        fp != (++phases.begin())->members.end(); fp++)
        fp->instantKickRunning = true;


    }
    else
    {
      float endingFactor = 1.;
      START_POLYGON(5, polygonStart, polygonEnd, theWalkingEngineParams.footPitch);
      POINT(theWalkingEngineParams.forwardPolygon[0], theWalkingEngineParams.heightPolygon[0], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.forwardPolygon[1], theWalkingEngineParams.heightPolygon[1], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.forwardPolygon[2], theWalkingEngineParams.heightPolygon[2], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.forwardPolygon[3], theWalkingEngineParams.heightPolygon[3], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.forwardPolygon[4], theWalkingEngineParams.heightPolygon[4], endingFactor * walkStepHeight);
      END_POLYGON(output, len, POLYNOM_DEGREE);

      // Now add a rotation around x
      for (int i = 1; i < len; i++)
        output[i].rx = (output[i] - output[i-1]).rotate2D(-output[i].r).y * theWalkingEngineParams.footRoll;
      currentKickPhase = freeLegNA;
    }

    (++phases.begin())->replace(output, footNum);

    delete[] output;
  }
}

void SwingLegController::addFootsteps(const Footposition &fp)
{
  if (phases.empty() || phases.back().members.back().phase != fp.phase)
    phases.push_back(Phase(fp));
  else
    phases.back().members.push_back(fp);
}


Footposition SwingLegController::Shrink()
{
  ASSERT(!phases.empty());

  Footposition footposition = phases.front().members.front();

  phases.front().members.pop_front();
  if (phases.front().members.empty())
    phases.pop_front();

  return footposition;
}

bool SwingLegController::modPossible(PhaseList::iterator ph,
  float modificatorRCS,
  int dim)
{
  /* Check every phase since it is possible that e.g. in the first phase
     -0.1 + 0.9 holds, but adding 0.9 to other phases can lead the very
     large sidesteps */
  for (PhaseList::iterator t_ph = ph; t_ph != phases.end(); t_ph++)
  {
    const Point modRCS = theWalkingInfo.vecToRobotCoords(t_ph->getSidestep());
    if (std::abs(modRCS.v[dim] + modificatorRCS) > theWalkingEngineParams.maxSidestep[dim] ||
      std::abs(modRCS.v[dim] + modificatorRCS) < theWalkingEngineParams.minSidestep[dim])// TODO: Fixme, min for the overall step!
      return false;
  }
  return true;
}

void SwingLegController::modifyFp(int startFoot,
  PhaseList::iterator ph,
  Vector2f posErrRCS,
  Vector2f velErrRCS,
  Vector2f zmpErrRCS,
  float modificatorRCS,
  int dim)
{
  Point modRCS;
  modRCS.v[dim] = modificatorRCS;
  modification[dim] = theWalkingInfo.vecToWorldCoords(modRCS);
  modification.creationTime = theFootSteps.time;
  modification.aTime[dim].startZMP = ph->members.front().timestamp;

  Vector2f posErrWCS = theWalkingInfo.vecToWorldCoords(posErrRCS);
  Vector2f velErrWCS = theWalkingInfo.vecToWorldCoords(velErrRCS);
  Vector2f zmpErrWCS = theWalkingInfo.vecToWorldCoords(zmpErrRCS);

  modification.handledErr.x()[0] += posErrWCS.x();
  modification.handledErr.x()[1] += velErrWCS.x();
  modification.handledErr.x()[2] += zmpErrWCS.x();

  modification.handledErr.y()[0] += posErrWCS.y();
  modification.handledErr.y()[1] += velErrWCS.y();
  modification.handledErr.y()[2] += zmpErrWCS.y();

  ph->modifySidestepSum(modification[dim]);
  // The next double support is modified. Since the foot replacement was
  // already done for the current phase, the interpolation to the upcomming
  // double support must be done now.
  if (ph == ++phases.begin())
  {
    Point interpol = modification[dim] / phases.front().members.size();
    int i = 0;
    for (FootList::iterator fp = phases.begin()->members.begin();
      fp != phases.begin()->members.end();
      fp++, i++)
      fp->modify(interpol * static_cast<float>(i), startFoot);
  }
  bool both = false;
  for (; ph != phases.end(); ph = skip(ph, 1))
  {
    ph->modify(modification[dim], startFoot);

    if (modification.aTime[dim].startFoot[startFoot] < 0)
    {
      modification.aTime[dim].startFoot[startFoot] = ph->members.front().timestamp;
    }
    if (both)
    {
      ph->modify(modification[dim], !startFoot);
      if (modification.aTime[dim].startFoot[!startFoot] < 0)
        modification.aTime[dim].startFoot[!startFoot] = ph->members.front().timestamp;
    }
    else
    {
      both = ((ph->members.front().phase == secondSingleSupport) ||
        (ph->members.front().phase == firstSingleSupport));
    }
  }
  for (int i = 0; i < 6; i++)
    ASSERT(modification[X].v[i] == modification[X].v[i]);
}

void SwingLegController::sidestep()
{
#if 1
  Vector2f posErrWCS(theObservedError.CoM_WCS.x()[0], theObservedError.CoM_WCS.y()[0]);
  Vector2f velErrWCS(theObservedError.CoM_WCS.x()[1], theObservedError.CoM_WCS.y()[1]);
  Vector2f zmpErrWCS(theObservedError.CoM_WCS.x()[2], theObservedError.CoM_WCS.y()[2]);

  Vector2f posErrRCS = theWalkingInfo.vecToRobotCoords(posErrWCS);
  Vector2f velErrRCS = theWalkingInfo.vecToRobotCoords(velErrWCS);
  Vector2f zmpErrRCS = theWalkingInfo.vecToRobotCoords(zmpErrWCS);

  for (int i = 0; i < 2; i++)
    modification.aTime[i].startZMP =
    modification.aTime[i].startFoot[LEFT_FOOT] =
    modification.aTime[i].startFoot[RIGHT_FOOT] = -1;
  modification.handledErr.x() = Vector3f::Zero();
  modification.handledErr.y() = Vector3f::Zero();

  for (PhaseList::iterator ph = phases.begin(); ph != phases.end(); ph++)
  {
    if (ph->members.front().phase == unlimitedDoubleSupport)
      return;
  }

  bool xDone = false, yDone = false;
  if (phases.empty()) // TODO: better solution ?
    return;
  for (PhaseList::iterator ph = ++phases.begin();
    ph != phases.end() && ph != skip(phases.begin(), 2); ph++)
  {
    FootpositionListElement fp = ph->members.front();
    int delta = fp.timestamp - phases.front().members.front().timestamp;
    if (delta > 2 && isDS(fp.phase) && delta < theControllerParams.N)
    {
      Vector2f offsetRCS(theControllerParams.Ge[delta - 1][0] * posErrRCS.x() +
        theControllerParams.Ge[delta - 1][1] * velErrRCS.x() +
        theControllerParams.Ge[delta - 1][2] * zmpErrRCS.x(),
        theControllerParams.Ge[delta - 1][0] * posErrRCS.y() +
        theControllerParams.Ge[delta - 1][1] * velErrRCS.y() +
        theControllerParams.Ge[delta - 1][2] * zmpErrRCS.y());

      int footNum = (fp.phase == firstDoubleSupport);

      if (!xDone && modPossible(ph, offsetRCS.x(), X)) // TODO: Make it possible to apply only a part of error
      {
        modifyFp(footNum,
          ph,
          Vector2f(posErrRCS.x(), 0),
          Vector2f(velErrRCS.x(), 0),
          Vector2f(zmpErrRCS.x(), 0),
          offsetRCS.x(),
          X);
        xDone = true;
      }

      if (!yDone &&
        ((fp.footPos[LEFT_FOOT].y -
        fp.footPos[RIGHT_FOOT].y -
        2 * theWalkingEngineParams.footYDistance -
        (1 - 2 * footNum) * offsetRCS.y() >= 0) ||
        ((1 - 2 * footNum) * offsetRCS.y() >= 0)) &&
        modPossible(ph, offsetRCS.y(), Y)) // Also possible to reduce the last side step
      {
        modifyFp(footNum,
          ph,
          Vector2f(0, posErrRCS.y()),
          Vector2f(0, velErrRCS.y()),
          Vector2f(0, zmpErrRCS.y()),
          offsetRCS.y(),
          Y);
        yDone = true;
      }
    }
  }
#endif
}

void SwingLegController::updateFootpositions(Footpositions &footpositions)
{
  DEBUG_RESPONSE("module:SwingLegController:debugOutput")
    debug = true;
  paramsKickMotion.handle();

  // if (paramsKickMotion.p.empty())
  // paramsKickMotion.p.push_back(Point());

  if (isRunning && !theFootSteps.running)
    reset();

  footpositions.kickPhase = currentKickPhase;
  isRunning = theFootSteps.running;

  if (theFallDownState.state != FallDownState::upright) reset();
  for (int i = 0; i < theFootSteps.getNumOfSteps(); i++)
    addFootsteps(theFootSteps.getStep(i));

  if (isRunning)
  {
    PlanFootReset(LEFT_FOOT);
    PlanFootReset(RIGHT_FOOT);
    footpositions = Shrink();
    footpositions.running = true;
    for (int i = 0; i < 2; i++)
    {
      ASSERT(footpositions.footPos[i].x == footpositions.footPos[i].x);
      ASSERT(footpositions.footPos[i].x == footpositions.footPos[i].x);
      ASSERT(footpositions.footPos[i].x == footpositions.footPos[i].x);
    }

  }
  else
  {
    footpositions.running = false;
    footpositions = theFootSteps.suggestedStep;
  }
  
  // Use this to have a view with all steps in preview
  // vd module:SwingLegController:phases
  MODIFY("module:SwingLegController:phases", infos);
  
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].x", footpositions.footPos[LEFT_FOOT].x);
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].y", footpositions.footPos[LEFT_FOOT].y);
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].z", footpositions.footPos[LEFT_FOOT].z);
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].r", footpositions.footPos[LEFT_FOOT].r);
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].rx", footpositions.footPos[LEFT_FOOT].rx);
  PLOT("module:SwingLegController:footpositions.footPos[LEFT_FOOT].ry", footpositions.footPos[LEFT_FOOT].ry);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].x", footpositions.footPos[RIGHT_FOOT].x);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].y", footpositions.footPos[RIGHT_FOOT].y);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].z", footpositions.footPos[RIGHT_FOOT].z);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].r", footpositions.footPos[RIGHT_FOOT].r);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].rx", footpositions.footPos[RIGHT_FOOT].rx);
  PLOT("module:SwingLegController:footpositions.footPos[RIGHT_FOOT].ry", footpositions.footPos[RIGHT_FOOT].ry);
  PLOT("module:SwingLegController:StepsCount", phases.size());
  PLOT("module:SwingLegController:footpositions.direction", footpositions.direction);
}

void SwingLegController::update(ReferenceModificator &referenceModificator)
{
  sidestep(); // This was in the other update, was there a reason for that?
  referenceModificator = modification;
}