/**
* @file Bspline.h
* 
* Functions to calculate bspline curves.
*
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

template <class P = Point> struct BSpline
{
  /**
    * Calculates a bspline curve.
    * @param n The number of control points minus 1.
    * @param t The degree of the polynomial plus 1
    * @param control Control point array made up of point stucture
    * @param output Array in which the calculate spline points are to be put
    * @param num_output How many points on the spline are to be calculated
  */
  static void bspline(int n, int t, P *control, P *output, int num_output)

    /*********************************************************************

    Parameters:
    n          - the number of control points minus 1
    t          - the degree of the polynomial plus 1
    control    - control point array made up of point stucture
    output     - array in which the calculate spline points are to be put
    num_output - how many points on the spline are to be calculated

    Pre-conditions:
    n+2>t  (no curve results if n+2<=t)
    control array contains the number of points specified by n
    output array is the proper size to hold num_output point structures


    **********************************************************************/

  {
    int *u;
    float increment, interval;
    P calcxyz;
    int output_index;

    u = new int[n + t + 1];
    compute_intervals(u, n, t);

    increment = (float)(n - t + 2) / (num_output - 1);  // how much parameter goes up each time
    interval = 0;

    for (output_index = 0; output_index<num_output - 1; output_index++)
    {
      compute_point(u, n, t, interval, control, &calcxyz);
      output[output_index] = calcxyz;
      interval = interval + increment;  // increment our parameter
    }
    output[num_output - 1] = control[n];   // put in the last point

    delete[] u;
  }
  static float blend(int k, int t, int *u, float v)  // calculate the blending value
  {
    float value;

    if (t == 1)			// base case for the recursion
    {
      if ((u[k] <= v) && (v<u[k + 1]))
        value = 1;
      else
        value = 0;
    }
    else
    {
      if ((u[k + t - 1] == u[k]) && (u[k + t] == u[k + 1]))  // check for divide by zero
        value = 0;
      else
        if (u[k + t - 1] == u[k]) // if a term's denominator is zero,use just the other
          value = (u[k + t] - v) / (u[k + t] - u[k + 1]) * blend(k + 1, t - 1, u, v);
        else
          if (u[k + t] == u[k + 1])
            value = (v - u[k]) / (u[k + t - 1] - u[k]) * blend(k, t - 1, u, v);
          else
            value = (v - u[k]) / (u[k + t - 1] - u[k]) * blend(k, t - 1, u, v) +
            (u[k + t] - v) / (u[k + t] - u[k + 1]) * blend(k + 1, t - 1, u, v);
    }
    return value;
  }

  static void compute_intervals(int *u, int n, int t)   // figure out the knots
  {
    int j;

    for (j = 0; j <= n + t; j++)
    {
      if (j<t)
        u[j] = 0;
      else
        if ((t <= j) && (j <= n))
          u[j] = j - t + 1;
        else
          if (j>n)
            u[j] = n - t + 2;  // if n-t=-2 then we're screwed, everything goes to 0
    }
  }

  static void compute_point(int *u, int n, int t, float v, P *control, P *output)
  {
    int k;
    float temp;

    // initialize the variables that will hold our outputted point
    *output = P();

    for (k = 0; k <= n; k++)
    {
      temp = blend(k, t, u, v);  // same blend is used for each dimension coordinate
      *output = *output + control[k] * temp;
      //output->y = output->y + (control[k]).y * temp;
      //output->z = output->z + (control[k]).z * temp;
    }
  }
};