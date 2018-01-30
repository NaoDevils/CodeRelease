/**
 * @file Representations/Infrastructure/MocapData.h
 *
 * This file contains the motion capture data
 *
 * @author Janine Hemmers
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/MocapRigidbody.h"
#include "Representations/Infrastructure/MocapMarkerSet.h"

STREAMABLE(MocapData,
{ ,
  (std::vector<MocapRigidbody>) rigidBodies,
  (MocapMarkerSet) markerSet,
  (MocapMarkerSet) markerSetBall,
  (unsigned int)(0) frameNumber,
  (double)(0) timestamp,
});