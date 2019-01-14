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
#include <SPLStandardMessage.h>

// This should be defined in one of the includes above
// but it isn't so i will just leave this here
#define MIXEDTEAM_INTENTION_DEFAULT 0
#define MIXEDTEAM_INTENTION_KEEPER 1
#define MIXEDTEAM_INTENTION_DEFENSIVE 2
#define MIXEDTEAM_INTENTION_KICK 3
#define MIXEDTEAM_INTENTION_LOST 4

#define MIXEDTEAM_SUGGESTION_DEFAULT 0
#define MIXEDTEAM_SUGGESTION_KEEPER 1
#define MIXEDTEAM_SUGGESTION_DEFENSIVE 2
#define MIXEDTEAM_SUGGESTION_OFFENSIVE 3
#define MIXEDTEAM_SUGGESTION_KICK 4
}
