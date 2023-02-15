/**
* @file ObservedFLIPMError 
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Tools/Math/Eigen.h"
/**
 * @class ObservedFLIPMError
 *
 */
STREAMABLE(ObservedFLIPMError,
	/** Constructor */
	ObservedFLIPMError(){
    ObservedError[0].setZero();
    ObservedError[1].setZero();
	};
  ,
  (Eigen::Matrix< Vector6d, 2, 1>) ObservedError
);
