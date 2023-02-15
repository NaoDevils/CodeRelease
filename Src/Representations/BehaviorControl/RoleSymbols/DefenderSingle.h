/**
* \file DefenderSingle.h
* The file declares a class that containts data about the desired position of the defenderSingle on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"

/**
* \class DefenderSingle
* The file declares a class that containts data about the desired position of the defenderSingle on the field.
*/
STREAMABLE_WITH_BASE(DefenderSingle, PositioningSymbols, );
