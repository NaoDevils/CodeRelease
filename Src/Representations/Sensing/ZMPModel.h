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
  (Vector3f)(Vector3f::Zero()) zmp_acc, //ZMP
  (Vector3f)(Vector3f::Zero()) ZMP_WCS // ZMP in Walking Engine World Coordinate System
);
