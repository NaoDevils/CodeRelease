/**
* @file ActualCoM 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Debugging/Watch.h"
#else
#include "bhumanstub.h"
#include "Watch.h"
#endif

/**
* @class ActualCoM 
* Representing the actual position of center of mass in the walking engines
* world coordinate system.
*/
struct ActualCoM : public TranslationPoint
{
  void watch()
  {
    WATCH(x);
    WATCH(y);
    WATCH(z);
  }
};

struct ActualCoMRCS : public ActualCoM
{
};