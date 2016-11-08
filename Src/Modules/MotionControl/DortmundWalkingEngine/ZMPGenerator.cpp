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

#include "ZMPGenerator.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/interpolator.h"
#include "Bezier.h"
#include <cmath>

#define FOOT_LEN	0.16f

//#define CENTER_ZMP_X

ZMPGenerator::ZMPGenerator(		
                           const FootSteps				&theFootSteps,
                           const WalkingEngineParams	&theWalkingEngineParams,
                           const ControllerParams		&theControllerParams,
                           const FreeLegPhaseParams	&theFreeLegPhaseParams,
                           const WalkingInfo			&theWalkingInfo,
                           const ReferenceModificator  &theReferenceModificator):
theFootSteps(theFootSteps),
theWalkingEngineParams(theWalkingEngineParams),
theControllerParams(theControllerParams),
theFreeLegPhaseParams(theFreeLegPhaseParams),
//theWalkingInfo(theWalkingInfo), unused
theReferenceModificator(theReferenceModificator)
{
  reset();
}


ZMPGenerator::~ZMPGenerator(void)
{
  freeMem();
}

void ZMPGenerator::reset()
{
  freeMem();
  lpxss=0;
  plotZMP = 0;
  lastZMPRCS = ZMP(0,0);
}

void ZMPGenerator::Shrink()
{
  while(lastPlannedZMP!=footPositions.begin())
  {
    Footposition *footPos=footPositions.front();
    footPositions.pop_front();
    delete footPos;
  }
}

void ZMPGenerator::freeMem()
{
  int size = static_cast<int>(footPositions.size());
  for (int i=0; i<size; i++)
  {
    Footposition *footPos=footPositions.front();
    footPositions.pop_front();
    delete footPos;
  }
}

void ZMPGenerator::addFootsteps(const Footposition &fp)
{
  Footposition *newpos=new Footposition;
  *newpos=fp;

  footPositions.push_back(newpos);

  if (footPositions.front()==footPositions.back())
    lastPlannedZMP=footPositions.begin();

}

DECLARE_INTERPOLATE_VAR(px, double, 0);
DECLARE_INTERPOLATE_VAR(stand_px, double, 0);

void ZMPGenerator::planZMP(RefZMP &refZMP)
{
  FootList::iterator _footList;
  double diff;

  for (_footList=lastPlannedZMP; _footList!=footPositions.end(); ++_footList)
  {
    float vxss;
    float spdx=(float)(*_footList)->speed.x;
    float tss=(*_footList)->singleSupportLen*(float)theControllerParams.dt;
    float tds=(*_footList)->doubleSupportLen*(float)theControllerParams.dt;
    unsigned int pc=(*_footList)->frameInPhase;
    float plgn[4];

    if (spdx<FOOT_LEN/tss)
      vxss=spdx;
    else
      vxss=FOOT_LEN/tss;
    float curStepDuration = (*_footList)->stepDuration;
    float vxds=(curStepDuration*spdx-(2*tss*vxss))/(2*tds);

    Point p, fp; // p: ZMP in foot coordinate system. fp: Position of foot (including
    // rotation. So, first create ZMP trajectory through foot and 
    // translate/rotate it into the woorld coordinate system by using fp

    fp = (*_footList)->footPos[ZMP::phaseToZMPFootMap[(*_footList)->phase]];

    Point rf=(*_footList)->footPos[RIGHT_FOOT];
    Point lf=(*_footList)->footPos[LEFT_FOOT];

    rf.rotate2D(-fp.r);
    lf.rotate2D(-fp.r);

    // Diese Art der Erzeugung ist bei Kurven nicht mehr korrekt. Um wirklich eine möglichst Gleichförmige
    // Vorwärtsbewegung zu erzeugen muß der ZMP im Roboterkoordinatensystem in x und y Richtung erzeugt werden,
    // nicht wie derzeit im Fußkoordinatensystem. Allerdings erfordert das Bedingungen wie die Ausrichtung
    // des Standfußes zu Begin und zum Ende der Phase für die x-Geschwindigkeit (da diese dann nicht mehr nur
    // von der Fußlänge abhängt, sondern auch von der Rotation), und die aktuelle Ausrichtung, um die aktuelle
    // Breite des Fußes bestimmen zu können (wichtig für die y-Schwingung).

    float pxss=vxss*tss/2;

    const float *polygonLeft = theWalkingEngineParams.polygonLeft;
    const float *polygonRight = theWalkingEngineParams.polygonRight;
    if ((*_footList)->kickPhase == ongoing)
    {
      polygonLeft = theFreeLegPhaseParams.polygonLeft;
      polygonRight = theFreeLegPhaseParams.polygonRight;
    }

    bool toConvert=false;
    switch ((*_footList)->phase)
    {

    case firstSingleSupport:
      p.y=FourPointBezier1D(polygonLeft, (float)(*_footList)->frameInPhase/(*_footList)->singleSupportLen);
      if ((*_footList)->kickPhase == freeLegNA)
        p.x=-pxss+pc*(float)theControllerParams.dt*vxss;
      else
        p.x = -0.02f;
      toConvert=true;    
      break;

    case firstDoubleSupport:
      if ((*_footList)->kickPhase == ending)
      {
        p.y = (rf.y - lf.y) / 2;
        toConvert = true;
      }
      else
      {
        plgn[0]=polygonLeft[3];
        plgn[1]=(float)(rf.y-lf.y)/2;
        plgn[2]=plgn[1];
        plgn[3]=-(float)(lf.y-rf.y)+polygonRight[0];
        p.y=FourPointBezier1D(plgn, (float)(*_footList)->frameInPhase/(*_footList)->doubleSupportLen);
        toConvert=true;
      }
      if ((*_footList)->kickPhase == freeLegNA)
        p.x=+pxss+pc*(float)theControllerParams.dt*vxds;
      else
        p.x = -0.02f;
      break;

    case secondSingleSupport:
      p.y=FourPointBezier1D(polygonRight, (float)(*_footList)->frameInPhase/(*_footList)->singleSupportLen);
      if ((*_footList)->kickPhase == freeLegNA)
        p.x=-pxss+pc*(float)theControllerParams.dt*vxss;
      else
        p.x = -0.02f;
      toConvert=true;
      break;

    case secondDoubleSupport:
      if ((*_footList)->kickPhase == ending)
      {
        p.y = (lf.y - rf.y) / 2;
        toConvert = true;
      }
      else
      {
        plgn[0]=polygonRight[3];
        plgn[1]=-(float)(rf.y-lf.y)/2;
        plgn[2]=plgn[1];
        plgn[3]=(float)(lf.y-rf.y)+polygonLeft[0];
        p.y=FourPointBezier1D(plgn, (float)(*_footList)->frameInPhase/(*_footList)->doubleSupportLen);
        toConvert=true;
      }
      if ((*_footList)->kickPhase == freeLegNA)
        p.x=+pxss+pc*(float)theControllerParams.dt*vxds;
      else
        p.x = -0.02f;
      break;

    case unlimitedSingleSupportLeft:
      p=(*_footList)->footPos[LEFT_FOOT]+Point(theFreeLegPhaseParams.zmpLeftX, theFreeLegPhaseParams.zmpLeftY);
      diff=p.y-zmp.y();
      if (std::abs(diff)>std::abs(theFreeLegPhaseParams.zmpMoveSpeedY))
        p.y=zmp.y()+sgn(p.y-zmp.y())*theFreeLegPhaseParams.zmpMoveSpeedY;
      break;

    case unlimitedSingleSupportRight:
      p=(*_footList)->footPos[RIGHT_FOOT]+Point(theFreeLegPhaseParams.zmpRightX, theFreeLegPhaseParams.zmpRightY);
      diff=p.y-zmp.y();
      if (std::abs(diff)>std::abs(theFreeLegPhaseParams.zmpMoveSpeedY))
        p.y=zmp.y()+sgn(p.y-zmp.y())*theFreeLegPhaseParams.zmpMoveSpeedY;
      break;

    default:
      p.x = -pxss;
      p.y = -theWalkingEngineParams.footYDistance;
      toConvert = true;
      break;
    }

    if (spdx>0 && -lpxss>p.x)
      // speed is >0, so the zmp should go forward, but wants to jump back. We won't allow it, and wait
      // until the zmp reaches the last zmp point
      p.x=-lpxss;
    else if (spdx<0 && -lpxss<p.x)
      p.x=-lpxss;
    else if (spdx == 0 && toConvert)
    {
      const float intfac = 0.99f;
      lastZMPRCS.x() = pxss * (1 - intfac) + intfac * lastZMPRCS.x();
      //p.x = lastZMPRCS.x;
      lpxss=(float)-p.x;
    }
    else
    {
      lpxss=pxss;
      lastZMPRCS.x() = -lpxss;
    }
    
    static float lastspdx = 0;
    
    if (lastspdx * spdx < 0)
      lpxss=0;
    lastspdx = spdx;

    // Translate and rotate the ZMP to world coordinate system
    if (toConvert)
    {
      p.rotate2D((*_footList)->direction);
      p+=fp;
    }

#ifdef CENTER_ZMP_X
    p.x=fp.x;
#endif

    // Collect data for plotting
    // Warning: At first start of controller the preview will be filled,
    // but the first plotted point is the end of the preview, so there will
    // be a jump from 0 to the end position
    static Point lastPlotZMP;
    static Point t_ZMP;
    t_ZMP = p - lastPlotZMP;
    t_ZMP.rotate2D(-(*_footList)->direction);
    plotZMP += t_ZMP;
    plotZMP.x = std::fmod(plotZMP.x, 1.f);
    lastPlotZMP = p;


    zmp=p;
    zmp.timestamp=(*_footList)->timestamp;
    refZMP.addZMP(zmp);
  }

  lastPlannedZMP=_footList;
}

void ZMPGenerator::updateRefZMP(RefZMP& refZMP)
{
	for (int i=0; i < theFootSteps.getNumOfSteps(); i++)
      addFootsteps(theFootSteps.getStep(i));

  refZMP.init();
  refZMP.running=theFootSteps.running;

  if (!isRunning && theFootSteps.running && !footPositions.empty())
  {

    isRunning=true;
  }

  if (isRunning && !theFootSteps.running)
  {
    reset();
    isRunning=false;
  }

  if (isRunning)
  {
    planZMP(refZMP);

    Shrink();
    for (FootList::iterator fp = footPositions.begin();
         fp != footPositions.end();
         fp++)
    {
      for (int dim = 0; dim < 2; dim++)
        for (int footNum = 0; footNum < 2; footNum++)
          if (theReferenceModificator.aTime[dim].startFoot[footNum] != -1 && 
              (int)(*fp)->timestamp >= theReferenceModificator.aTime[dim].startFoot[footNum])
            (*fp)->footPos[footNum] += theReferenceModificator[dim];
    }

    static ZMP lastZMP;
    ASSERT(std::abs(lastZMP.x() - refZMP.getZMP(0).x()) < 100);
    ASSERT(std::abs(lastZMP.y() - refZMP.getZMP(0).y()) < 100);
    lastZMP = refZMP.getZMP(0);
  }

  
}
