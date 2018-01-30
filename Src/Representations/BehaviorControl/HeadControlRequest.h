/**
* @file HeadControlRequest.h
* 
* Declaration of class HeadControlRequest
* @author <A href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*/ 

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Angle.h"

/**
* The class calculates a head angle request dependent on given control type.
* For use in behavior control type soccer should be default.
*/
STREAMABLE(HeadControlRequest,
{
  ENUM(ControlType,
  {,
    ball, // fixiate on ball
    localize, // try to gather localization information
    localizeBall, // try to find ball (w/fieldCoverage)
    opponents, // fixiate on opponents
    soccer, // optimal mix for soccer
    direct, // direct control of angles
  }),

  (ControlType)(direct) controlType,
  (Angle)(0.f) pan,
  (Angle)(0.f) tilt,
});
