/**
  * @file PNGImage.h
  * Dummy representation to get the PNGLogger running.
  * @author Fabian Rensen
**/
#pragma once

#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(PNGImageDummy,
{
   PNGImageDummy() = default,
   (bool) (false) successful,
});
