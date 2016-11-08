/**
* @file ObservedError 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Tools/Math/Eigen.h"
/**
 * @class ObservedError
 *
 */
class ObservedError : public Streamable
{

public :

	/** Constructor */
	ObservedError(){};

	/** Destructor */
	~ObservedError()
	{};

  Eigen::Matrix< Vector3f, 2, 1> CoM_WCS, ZMP_WCS;

	void serialize(In* in,Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(CoM_WCS);
      STREAM(ZMP_WCS);
      STREAM_REGISTER_FINISH;
    };
};
