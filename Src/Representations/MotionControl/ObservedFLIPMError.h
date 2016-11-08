/**
* @file ObservedFLIPMError 
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Tools/Math/Eigen.h"
/**
 * @class ObservedFLIPMError
 *
 */
class ObservedFLIPMError : public Streamable
{

public :

	/** Constructor */
	ObservedFLIPMError(){
	};

	/** Destructor */
	~ObservedFLIPMError()
	{};

  Eigen::Matrix< Vector6f, 2, 1> CoM_1_WCS, Acc_1_WCS, CoM_2_WCS;
	
	void serialize(In* in,Out* out)
    {
      STREAM_REGISTER_BEGIN;
	    STREAM(CoM_1_WCS);
	    STREAM(Acc_1_WCS);
	    STREAM(CoM_2_WCS);
      STREAM_REGISTER_FINISH;
    };
};
