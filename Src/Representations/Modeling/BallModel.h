/**
 * @file BallModel.h
 *
 * Declaration of struct BallModel
 *
 * @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "RobotPose.h"
#include "TeamBallModel.h"

/**
 * @struct BallState
 *
 * Base struct for ball position and velocity.
 */
STREAMABLE(BallState,
{
  /**
  * Converts the \c velocity of the ball which is given relative to the robot
  * pose \c rp into global field coordinates.
  * \param rp The ball position is assumed to be ralative to this robot pose.
  * \return The velocity of the ball relative to the field coordinate system (in mm/s).
  */
  Vector2f getVelocityInFieldCoordinates(const RobotPose& rp) const
  {
    float c(std::cos(rp.rotation));
    float s(std::sin(rp.rotation));
    return Vector2f(velocity.x()*c - velocity.y()*s, velocity.x()*s + velocity.y()*c);
  }
  /**
  * Updates the \c position and \c velocity of the ball by converting the
  * given values from global field coordinates to values relative to the robot
  * pose \c rp.
  * \param positionOnField The position of the ball relative to the field
  *                        coordinate system (in mm).
  * \param velocityOnField The velocity of the ball relative to the field
  *                        coordinate system (in mm/s).
  * \param rp The ball position is updated ralative to this robot pose.
  */
  void setPositionAndVelocityInFieldCoordinates(const Vector2f& positionOnField,
    const Vector2f& velocityOnField,
    const RobotPose& rp)
  {
    position = Transformation::fieldToRobot(rp, positionOnField);
    float c(std::cos(rp.rotation));
    float s(std::sin(rp.rotation));
    velocity = Vector2f(velocityOnField.x()*c + velocityOnField.y()*s,
      -velocityOnField.x()*s + velocityOnField.y()*c);
  },

  (Vector2f)(Vector2f::Zero()) position, /**< The position of the ball relative to the robot (in mm)*/
  (Vector2f)(Vector2f::Zero()) velocity, /**< The velocity of the ball relative to the robot (in mm/s)*/
  (float)(35) radius,                    /**< The assumed radius of the ball (in mm)*/
});

/**
 * @struct BallModel
 *
 * Contains all current knowledge about the ball.
 */
STREAMABLE(BallModel,
{
  /** Draws the estimate on the field */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) lastPerception, /**< The last seen position of the ball */
  (BallState) estimate, /**< The state of the ball estimated from own observations; it is propagated even if the ball is not seen */
  (unsigned)(0) timeWhenLastSeen, /**< Time stamp, indicating what its name says */
  (unsigned)(0) timeWhenLastSeenByTeamMate, /**< Time stamp, indicating what its name says */
  (unsigned)(0) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there */
  // Unused by Dortmund:
  (unsigned char)(0) seenPercentage, /**< How often was the ball seen in the recent past (0%...100%). */
  
  //BEGIN Added by Dortmund
  (float)(0.f) validity, /**< Validity of the current ball estimate in range [0,1]. */
  //END
});

/**
 * @struct GroundTruthBallModel
 * The same as the BallModel, but - in general - provided by an external
 * source that has ground truth quality
 */
struct GroundTruthBallModel : public BallModel
{
  /** Draws something*/
  void draw() const;
};

struct BallModelAfterPreview : public BallModel
{
};

/**
 * @struct BallModelCompressed
 * A compressed version of BallModel used in team communication
 */
STREAMABLE(BallModelCompressed,
{
  BallModelCompressed() = default;
  BallModelCompressed(const BallModel & ballModel);
  operator BallModel() const,

  (Vector2s) lastPerception, /**< The last seen position of the ball */
  (Vector2f) position,
  (Vector2f) velocity,
  (unsigned) timeWhenLastSeen, /**< Time stamp, indicating what its name says */
  (unsigned) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there */
  
  //BEGIN Added by Dortmund
  (float) validity, /**< Validity of the current ball estimate in range [0,1]. */
  //END
});
