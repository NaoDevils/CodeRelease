/**
 * @file RoboCupGameControlData.h
 * The file encapsulates definitions in the file RoboCupGameControlData.h
 * that is provided with the GameController in a namespace.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <cstdint>
#include <cstring>

namespace RoboCup
{
#include <RoboCupGameControlData.h>

// We don't want to set this to 20 as of now, because we will play with a
// maximum number of 7 robots anyway and the size of some arrays depend
// on this parameter.
#undef MAX_NUM_PLAYERS
#define MAX_NUM_PLAYERS 7

} // namespace RoboCup
