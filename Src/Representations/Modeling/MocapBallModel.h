/**
* @file MocapBallModel.h
*
* Declaration of struct MocapBallModel
*
* @author Janine Hemmers
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
* @struct MocapBallModel
*
* Contains all mocap knowledge about the ball.
*/
STREAMABLE(MocapBallModel,
{
  /** Draws the mocap ball model on the field */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) translation, /**< The last seen position of the ball */
  (float)(0.f) validity, /**< Validity of the current ball in mocap system */
  (unsigned int)(0) mocapFrameNumber,
  (double)(0) timestamp,
});