/**
* @file RotationPoint.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Math/graham.hpp"
#include "Tools/Math/Eigen.h"
#else
#include "graham.hpp"
#include "math/Vector2.h"
#include "math/Vector3.h"
#endif

#define LEFT_FOOT 0
#define RIGHT_FOOT 1


/**
 * @class RotationPoint
 * Calculates the point of rotation of the body
 * (a point of the convex hull of the support polygon).
*/
class RotationPoint
{
public:
	/** Constructor */
	RotationPoint(void);
	/** Destructor */
	~RotationPoint(void) {};


	Vector2f footHull[2][8]; /**< Given hull for left and right foot constisting of 8 points each */

	/** 
	* Calculates the current axis of rotation
	* @param direction The tipping direction. Use the down direction (x and y).
	* @param doubleSupport True, if current walking phase is a double support
	* @param footNum Number of the foot on floor (if doubleSupport==false), 0 is left foot, 1 is right foot
	* @return Plane vector of rotation axis
	*/
	Vector2f getCurrentRotationPoint(Vector2f direction, bool doubleSupport, int footNum);

private:

	GrahamScan dshull; /**< Convex hull for double support */
	GrahamScan ssHull[2]; /**<  Convex hull for left single support and right single support */

	bool lineIntersection(Vector2f p1, Vector2f p2, Vector2f p3, Vector2f p4);

};
				 
