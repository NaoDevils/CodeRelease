/**
 * @file PSGoalieTrigger.h
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(PSGoalieTrigger,
  ENUM(KickDirection,
 // from goalie perspective
    LEFT,
    CENTER,
    RIGHT
  );
,
  (bool)(false) hasKicked,
  (KickDirection)(CENTER) kickDirection,

  //Debug info
  (int) numOfNonFieldColorPixels
);
