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
    ObservedError[0].setZero();
    ObservedError[1].setZero();
	};

	/** Destructor */
	~ObservedFLIPMError()
	{};

  Eigen::Matrix< Vector6f, 2, 1> ObservedError;
	
	void serialize(In* in,Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(ObservedError);
      STREAM_REGISTER_FINISH;
    };
};
