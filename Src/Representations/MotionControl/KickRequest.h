/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(KickRequest,
  ENUM(KickMotionID,
    kickInner,
    kickMiddle,
    kickOuter,
    kickInnerFast,
    kickMiddleFast,
    kickOuterFast,
    penaltyKickInner_left,
    penaltyKickInner_right,
    penaltyKickMiddle_left,
    penaltyKickMiddle_right,
    penaltyKickOuter_left,
    penaltyKickOuter_right,
    mostPowerfulKick, // this kick should contain the strongest kick so it can be triggered by need
    newKick,
    none
  );

  static KickMotionID getKickMotionFromName(const char* name),

  (KickMotionID)(none) kickMotionType,
  (bool)(false) mirror,
  (bool)(false) dynamical,
  (bool)(true) armsBackFix,
  (std::vector<DynPoint>) dynPoints,
  (Vector2f)(Vector2f(1000,0)) kickTarget
);
