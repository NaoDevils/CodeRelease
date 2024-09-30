/**
* \file Ballchaser.h
* The file declares a class that containts data about the best kick position, target and type for the ballchaser.
*/

#pragma once
#include "PositioningAndKickSymbols.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * \class Ballchaser
 * A class that containts data about the best kick position, target and type for
 * the ballchaser.
 */
STREAMABLE_WITH_BASE(Ballchaser, PositioningAndKickSymbols,
  ,
);
