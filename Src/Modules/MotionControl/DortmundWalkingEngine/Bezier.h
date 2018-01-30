/**
* @file Bezier.h
* 
* Simple four point bezier courve
*
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once

#include "Point.h"

 /** 
  * Calculates a bezier courve based on the four point polygon.
  * @param point The Point on the courve.
  * @param polygon The Polygon as array of points.
  * @param t The Parameter t, must be between 0 and 1.
  */
inline void FourPointBezier(Point *point, const Point *polygon, float t)
{
	point->x=(1-t)*(1-t)*(1-t)*polygon[0].x+3*t*(1-t)*(1-t)*polygon[1].x+3*t*t*(1-t)*polygon[2].x+t*t*t*polygon[3].x;
	point->y=(1-t)*(1-t)*(1-t)*polygon[0].y+3*t*(1-t)*(1-t)*polygon[1].y+3*t*t*(1-t)*polygon[2].y+t*t*t*polygon[3].y;
	point->z=(1-t)*(1-t)*(1-t)*polygon[0].z+3*t*(1-t)*(1-t)*polygon[1].z+3*t*t*(1-t)*polygon[2].z+t*t*t*polygon[3].z;
}

/** 
  * Calculates a one dimensional bezier courve based on the four point polygon.
  * @param polygon The Polygon as array of points.
  * @param t The Parameter t, must be between 0 and 1.
  * @return The coordinate of the curve point.
  */
inline float FourPointBezier1D(const float *polygon, float t)
{
	return (1-t)*(1-t)*(1-t)*polygon[0]+3*t*(1-t)*(1-t)*polygon[1]+3*t*t*(1-t)*polygon[2]+t*t*t*polygon[3];
}



