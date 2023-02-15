/**
 * \file BallModel.h
 *
 * Declaration of representation BallModel
 *
 * \author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 * \author <A href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"

/**
 * \struct BallState
 *
 * Base struct for ball position and velocity.
 */
STREAMABLE(BallState,
  /**
  * Converts the \c velocity of the ball which is given relative to the robot
  * pose \c rp into global field coordinates.
  * \param rp The ball position is assumed to be relative to this robot pose.
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
  * \param rp The ball position is updated relative to this robot pose.
  */
  void setPositionAndVelocityFromFieldCoordinates(const Vector2f& positionOnField,
    const Vector2f& velocityOnField,
    const RobotPose& rp)
  {
    position = Transformation::fieldToRobot(rp, positionOnField);
    float c(std::cos(rp.rotation));
    float s(std::sin(rp.rotation));
    velocity = Vector2f(velocityOnField.x()*c + velocityOnField.y()*s,
      -velocityOnField.x()*s + velocityOnField.y()*c);
  },

  (Vector2f)(Vector2f(1000,0)) position, /**< The position of the ball relative to the robot (in mm)*/
  (Vector2f)(Vector2f::Zero()) velocity, /**< The velocity of the ball relative to the robot (in mm/s)*/
  (float)(50) radius                    /**< The assumed radius of the ball (in mm)*/
);

/**
 * \struct BallModel
 *
 * Contains all current knowledge about the ball collected from the robots own perception.
 * For ball information from team mates see representation \c TeamBallModel.
 *
 * The \c BallModel is constructed by a multi hypotheses filter. The best hypothesis is output
 * to this representation. Therefore it does not contain all ball percepts (\c theBallPercept)
 * seen by the robot but only those which fits to the best hypothesis.
 */
STREAMABLE(BallModel,
  /** Draws the estimate on the field */
  void draw() const,

  /** The unfiltered last ball percept which was used to update the \c estimate. */
  (Vector2f)(Vector2f(1000,0)) lastPerception,
  /** The filtered ball state (including position and velocity) estimated from percepts;
   *  it is propagated even if the ball is currently not seen. */
  (BallState) estimate,
  (unsigned)(0) timeWhenLastSeen, /**< The time of \c lastPerception. */
  (unsigned)(0) timeWhenLastSeenByTeamMate, /**< The time of the last ball percept of any team mate. */
  (unsigned)(0) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there. */
  // Unused by Dortmund:
  (unsigned char)(0) seenPercentage, /**< How often was the ball seen in the recent past (0%...100%). */
  
  //BEGIN Added by Dortmund
  (float)(0.f) validity, /**< Validity of the current ball estimate in range [0,1]. */
  (float)(-0.30f) friction, /**< Used ball friction in 1/s */
  (unsigned)(0) timeWhenBallInGoalBox
  //END
);

/**
 * \struct GroundTruthBallModel
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
 * \struct BallModelCompressed
 * A compressed version of BallModel used in team communication
 */
STREAMABLE(BallModelCompressed,
  BallModelCompressed() = default;
  BallModelCompressed(const BallModel & ballModel);
  operator BallModel() const,

  // last percept is equal to position for sending since 1 packet per second is not enough for difference
  //(Vector2s) lastPerception, /**< The last seen position of the ball */
  (Vector2s) position,
  (Vector2s) velocity,
  (unsigned)(0) timeWhenLastSeen, /**< Time stamp, indicating what its name says */
  //timeWhenDisappeared not sent, see above
  //(unsigned) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there */
  
  (std::uint8_t)(0) validity /**< Validity of the current ball estimate in range [0,255]. */
);

STREAMABLE(MultipleBallModel,
  /** Draws the estimate on the field */
  void draw() const,

  (std::vector<BallModel>) ballModels
);

struct MultipleBallModelAfterPreview : public MultipleBallModel
{
};

struct GroundTruthMultipleBallModel : public MultipleBallModel
{
  /** Draws something*/
  void draw() const;
};
