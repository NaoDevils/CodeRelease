#include "ZMPIPController2012.h"
//#include "LimbCombinator.h"
#define LOGGING
#ifndef WALKING_SIMULATOR
#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

//#include "CSTransform.h"

ZMPIPController2012::ZMPIPController2012(
  const RefZMP				&theRefZMP,
  const WalkingEngineParams	&theWalkingEngineParams,
  const ControllerParams		&theControllerParams,
  const ObservedError         &theObservedError,
  const ReferenceModificator  &theReferenceModificator):
theRefZMP(theRefZMP),
//theWalkingEngineParams(theWalkingEngineParams),
theControllerParams(theControllerParams),
theObservedError(theObservedError),
theReferenceModificator(theReferenceModificator),
positionDelayBuffer(Point())
{
  reset();																				    
}

void ZMPIPController2012::Shrink()
{
  while(pRef.begin()!=kElement)
    pRef.pop_front();
}

void ZMPIPController2012::reset()
{
  obs.x() = Eigen::Matrix<float, 3, 1>::Zero();
  obs.y() = Eigen::Matrix<float, 3, 1>::Zero();
  v = Vector2f::Zero();
  pRef.clear();

  kElementEmpty=true;

  isRunning=false;
  positionDelayBuffer.clear();
}

ZMP ZMPIPController2012::getReferenceZMP()
{
  if (isRunning)
  {
    return *kElement;
  }
  else return ZMP();
}

//Rensen: Removed due to unknown attribute warning
//#ifdef TARGET_ROBOT
//__attribute__((optimize("unroll-loops")))
//#endif
void ZMPIPController2012::executeController(Dimension d, const Eigen::Matrix<float, 1,ControllerParams::N> &refZMP)
{
  Vector3f err = theObservedError.ZMP_WCS(d) + theReferenceModificator.handledErr(d);
  
  float u = refZMP * theControllerParams.Gd;
  float cont = -theControllerParams.Gi * v(d) - theControllerParams.Gx * obs(d) - u;
  obs(d) = theControllerParams.A0 * obs(d) + err + theControllerParams.b0 * cont;
  v(d) += theControllerParams.c0 * obs(d) - (*kElement)(d);
}

//Rensen: Removed due to unknown attribute warning
//#ifdef TARGET_ROBOT
//__attribute__((optimize("unroll-loops")))
//#endif
Point ZMPIPController2012::controllerStep()
{
  ZMPList::iterator _pRef = kElement;
  Eigen::Matrix<Eigen::Matrix<float,1, ControllerParams::N>, 2, 1> refZMP;

  PLOT("module:ZMPIPController2012:refZMP.x", _pRef->x());
  PLOT("module:ZMPIPController2012:refZMP.y", _pRef->y());
  PLOT("module:ZMPIPController2012:calcZMP.x", obs(0)(2, 0));
  PLOT("module:ZMPIPController2012:calcZMP.y", obs(1)(2, 0));
  
  //Rensen: Added because of warning above
#ifdef TARGET_ROBOT
#pragma unroll
#endif
  for (int i = 0; i < ControllerParams::N; i++) {
    refZMP(0)(0,i) = _pRef->x();
    refZMP(1)(0,i) = _pRef->y();
    _pRef++;
  }
  
  executeController(X, refZMP.x());
  executeController(Y, refZMP.y());

  kElement++;

  return Point(obs(0)(0,0), obs(1)(0,0), theControllerParams.z_h);
}


void ZMPIPController2012::addRefZMP(ZMP zmp)
{
  pRef.push_back(zmp);

  if (kElementEmpty)
  {
    kElement=pRef.begin();
    kElementEmpty=false;
  }
}

void ZMPIPController2012::getObservations(Vector3f &x, Vector3f &y)
{
  x = obs(0);
  y = obs(1);
}


void ZMPIPController2012::updateKinematicRequest(TargetCoM & targetCoM)
{

  DECLARE_PLOT("module:ZMPIPController2012:refZMP.x");
  DECLARE_PLOT("module:ZMPIPController2012:refZMP.y");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.x");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.y");
  DECLARE_PLOT("module:ZMPIPController2012:targetCoM.x");
  DECLARE_PLOT("module:ZMPIPController2012:targetCoM.y");
  DECLARE_PLOT("module:ZMPIPController2012:calcZMP.y");
  
  for (int i=0; i<theRefZMP.numOfZMP; i++)
    addRefZMP(theRefZMP.getZMP(i));

  targetCoM.x=targetCoM.y=0;

  if (!isRunning && theRefZMP.running)
  {
    Start();
    isRunning=true;
  }

  if (isRunning && !theRefZMP.running)
  {
    End();
    isRunning=false;
  }
  if (isRunning)
  {
    ASSERT(pRef.size()>0);

    // Elemente, die vom ZMPGenerator modifiziert wurden und im selben Frame rausgeschickt wurden,
    // werden hier eingehangen und dann nochmal modififiert.

    (Point &)targetCoM=controllerStep();
    Shrink();
    /*LOG("WalkingEngine", "calc ZMP x", *obs.x[2]);
    LOG("WalkingEngine", "calc ZMP y", *obs.y[2]);
    LOG("WalkingEngine", "ref ZMP x", kElement->x);
    LOG("WalkingEngine", "ref ZMP y", kElement->y);
    LOG("WalkingEngine", "Target CoM x", targetCoM.x);
    LOG("WalkingEngine", "Target CoM y", targetCoM.y);
    LOG("WalkingEngine", "Target CoM z", targetCoM.z);*/


    for (ZMPList::iterator zmp = pRef.begin();
      zmp != pRef.end();
      zmp++)
    {
      for (int dim = 0; dim < 2; dim++)
        if (theReferenceModificator.aTime[dim].startZMP != -1 &&
          (int)(zmp->timestamp) >= theReferenceModificator.aTime[dim].startZMP)
            *zmp += ZMP(theReferenceModificator[dim].x, theReferenceModificator[dim].y);
    }
  }
  targetCoM.isRunning = isRunning;
  
  PLOT("module:ZMPIPController2012:targetCoM.x", targetCoM.x);
  PLOT("module:ZMPIPController2012:targetCoM.y", targetCoM.y);
  
  ASSERT(targetCoM.x == targetCoM.x && targetCoM.y == targetCoM.y);
}
