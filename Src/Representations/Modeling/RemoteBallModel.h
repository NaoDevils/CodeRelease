/**
 * \file RemoteBallModel.h
 *
 * Declaration of a representation that represents a ball model based
 * only on teammate observations.
 *
 * \author Heiner Walter, heiner.walter@tu-dortmund.de
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * \struct RemoteBallModel
 * 
 * Remote ball model based only on ball observations by teammates. The local ball models 
 * (not using ball percepts of teammates directly) of all teammates are merged into a set 
 * of remote ball hypotheses. The own local ball model is NOT included. The remote 
 * hypothesis with the best validity is used to fill this \c RemoteBallModel.
 */
STREAMABLE(RemoteBallModel,
{
  void draw() const,

  (Vector2f)(Vector2f::Zero()) position, /**< The position of the ball in global field coordinates (in mm) */
  (Vector2f)(Vector2f::Zero()) velocity, /**< The velocity of the ball in global field coordinates (in mm/s) */
  (unsigned)(0) timeWhenLastSeen, /**< The last timestamp when a teammate has seen this ball hypothesis. */
  (unsigned)(0) timeWhenLastSeenByGoalie, /**< The last timestamp when the goalie (by playernumber) has seen this ball hypothesis. */
  (float)(0.f) validity, /**< The validity of the ball hypothesis used as remote ball model in range [0,1]. */
  (std::string)("") teammates, /**< Stores the player numbers of all teammates which updated this ball hypothesis (e.g. "1,2,5"). */ 
});

struct RemoteBallModelAfterPreview : public RemoteBallModel
{

};