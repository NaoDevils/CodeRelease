/**
 * @file TeamBallModel.h
 *
 * Declaration of a representation that represents a ball model based
 * on own observations as well as on teammate observations.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct TeamBallModel
 */
STREAMABLE(TeamBallModel,
{
  void draw() const,

  (Vector2f)(Vector2f::Zero()) position, /**< The position of the ball in global field coordinates (in mm) */
  (Vector2f)(Vector2f::Zero()) velocity, /**< The velocity of the ball in global field coordinates (in mm/s) */
  (bool)(false) isValid, /**< Position and velocity are valid (i.e. somebody has seen the ball), if true */
  (unsigned)(0) timeWhenLastValid, /**< The last timestamp, when \c isValid was true. */
  
  //BEGIN Added by Dortmund
  (float)(0.f) validity, /**< The validity of the ball hypothesis used as team ball model in range [0,1]. */
  (bool)(true) isLocalBallModel, /**< True if the local \c BallModel is used as team ball model. */
  //END
});
