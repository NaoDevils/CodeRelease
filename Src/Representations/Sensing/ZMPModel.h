/**
* @file ZMPModel.h
*
* Declaration of class ZMPModel
*
* @author <A href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*/

#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Watch.h"
#endif
#include "Tools/Math/Eigen.h"

/**
 * @class ZMPModel
 *
 * Contains information about ZMP.
 */
STREAMABLE(ZMPModel,,
  (Vector2f)(Vector2f::Zero()) ZMP_FCS, // ZMP in Foot Coordinate System
  (Vector3f)(Vector3f::Zero()) ZMP_RCS, // ZMP in Robot Coordinate System
  (Vector3f)(Vector3f::Zero()) ZMP_WCS  // ZMP in Walking Engine World Coordinate System
);
