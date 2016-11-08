/**
* @file ControllerParams 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class ControllerParams
 *
 * Contains the parameters for the ZMP/IP-Controller calculated by the
 * matlab script writeParams.m
 */
class ControllerParams : public Streamable
{

public :
	float dt; /**< Duration of one frame */
  float z_h;	/**< desired CoM position (height) */
	static const int N = 50; /**< Length of the preview phase */
	Eigen::Matrix<float,N,1> Gd; /**< Data for preview controller */
	Matrix3f A0; /**< Data for preview controller */
	float Gi; /**< Data for preview controller */
	Eigen::Matrix<float, 1,3> Gx; /**< Data for preview controller */
	Vector3f b0; /**< Data for preview controller */
  Eigen::Matrix<float, 1, 3> c0; /**< Data for preview controller */
  Eigen::Matrix<float, 3, 2> L; /**< Data for preview controller */
  Vector3f Ge[N]; /**< Data for preview controller */

	/** Constructor */
	ControllerParams(){};

	/** Destructor */
	~ControllerParams()
	{};

	void serialize(In* in,Out* out) {};
};
