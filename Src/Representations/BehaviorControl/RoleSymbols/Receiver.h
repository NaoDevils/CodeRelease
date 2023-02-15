/**
* \file Receiver.h
* The file declares a class that containts data about the desired position of the receiver on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"

/**
* \class Receiver
* The file declares a class that containts data about the desired position of the receiver on the field.
*/
STREAMABLE_WITH_BASE(Receiver, PositioningSymbols, );
