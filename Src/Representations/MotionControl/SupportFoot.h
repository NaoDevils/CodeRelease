/**
 * @file SupportFoot.h
 * This file declares a representation that stores the SupportFoot
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"
#include "Tools/Debugging/DebugDrawings.h"


STREAMABLE(SupportFoot,
  ENUM(SupportFootState,
    leftSupportOnly,
    rightSupportOnly,
    bothFeetSupport
  );

  void draw() const
  {
    PLOT("representation:SupportFoot:support", supportFoot);
    PLOT("representation:SupportFoot:supportThreshold:max", threshold);
    PLOT("representation:SupportFoot:supportThreshold:min", -threshold);
    PLOT("representation:SupportFoot:supportFoot",
        (supportFootState == SupportFoot::bothFeetSupport) ? 0.f : ((supportFootState == SupportFoot::rightSupportOnly) ? -threshold : threshold));
  }
  ,
  (float)(0.f) supportFoot, // -1 = left, 0 = both, +1 = right
  (float)(0.f) threshold,
  (SupportFootState)(bothFeetSupport) supportFootState
);
