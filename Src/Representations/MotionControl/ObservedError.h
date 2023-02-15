/**
* @file ObservedError 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Tools/Math/Eigen.h"
/**
 * @class ObservedError
 *
 */
STREAMABLE(ObservedError,,
  (Eigen::Matrix<Vector3f, 2, 1>) CoM_WCS,
  (Eigen::Matrix<Vector3f, 2, 1>) ZMP_WCS
);
