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
  currentKickPhase = freeLegNA;
}

void SwingLegController::PlanFootReset(int footNum)
{
  FootpositionListElement &curPos = phases.front().members.front();
  float yFac;
  yFac =  std::abs(curPos.speed.y * 1000) / theWalkingEngineParams.speedLimits.y;
  float xStepHeight = curPos.speed.x > 0 ? theWalkingEngineParams.footMovement.stepHeight[0] : theWalkingEngineParams.footMovement.stepHeight[1];
  float walkStepHeight = (1 - yFac) * xStepHeight + yFac * theWalkingEngineParams.footMovement.stepHeight[1];

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
    {
      float endingFactor = 1.;
      START_POLYGON(5, polygonStart, polygonEnd, theWalkingEngineParams.footMovement.footPitch);
      POINT(theWalkingEngineParams.footMovement.forwardPolygon[0], theWalkingEngineParams.footMovement.heightPolygon[0], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.footMovement.forwardPolygon[1], theWalkingEngineParams.footMovement.heightPolygon[1], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.footMovement.forwardPolygon[2], theWalkingEngineParams.footMovement.heightPolygon[2], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.footMovement.forwardPolygon[3], theWalkingEngineParams.footMovement.heightPolygon[3], endingFactor * walkStepHeight);
      POINT(theWalkingEngineParams.footMovement.forwardPolygon[4], theWalkingEngineParams.footMovement.heightPolygon[4], endingFactor * walkStepHeight);
      END_POLYGON(output, len, POLYNOM_DEGREE);

      // Now add a rotation around x
      for (int i = 1; i < len; i++)
        output[i].rx = (output[i] - output[i-1]).rotate2D(-output[i].r).y * theWalkingEngineParams.footMovement.footRoll;
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
    if (std::abs(modRCS.v[dim] + modificatorRCS) > theWalkingEngineParams.maxSidestep[dim])
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
    Point interpol = modification[dim] / static_cast<float>(phases.front().members.size());
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
        2 * theWalkingEngineParams.footMovement.footYDistance -
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
  
  if (isRunning && !theFootSteps.running)
    reset();

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
