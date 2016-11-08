/**
* @file Bspline.h
* 
* Functions to calculate bspline curves.
*
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once

/**
 * Calculates a bspline curve.
 * @param n The number of control points minus 1.
 * @param t The degree of the polynomial plus 1
 * @param control Control point array made up of point stucture
 * @param output Array in which the calculate spline points are to be put
 * @param num_output How many points on the spline are to be calculated
 */
void bspline(int n, int t, Point *control, Point *output, int num_output);
float blend(int k, int t, int *u, float v);
void compute_point(int *u, int n, int t, float v, Point *control, Point *output);
void compute_intervals(int *u, int n, int t);
