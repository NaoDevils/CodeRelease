#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RefereeGesture,
  ENUM(Gesture,
    NONE,
    BOTH_HANDS_UP
  )
  ,
  (Gesture)(Gesture::NONE) gesture
);
