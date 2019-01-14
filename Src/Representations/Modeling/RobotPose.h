/**
 * @file RobotPose.h
 *
 * The file contains the definition of the struct RobotPose.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct RobotPose
 * The pose of the robot with additional information
 */
STREAMABLE_WITH_BASE(RobotPose, Pose2f,
{
  /**
   * Assignment operator for Pose2f objects
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
   */
  const RobotPose& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  }

  /** Draws the robot pose in the color of the team to the field view. */
  void draw() const,

  (float)(0) validity,                         /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
  (float)(0) symmetry,                         /**< The symmetry confidence of the robot pose. (0 = invalid, 1 = perfect) */
});

/**
 * @struct GroundTruthRobotPose
 * The same as the RobotPose, but - in general - provided by an external
 * source that has ground truth quality
 */
STREAMABLE_WITH_BASE(GroundTruthRobotPose, RobotPose,
{
  /** Draws the robot pose to the field view*/
  void draw() const,

  (unsigned int)(0) timestamp,
});

/**
* @struct MocapRobotPose
* The same as the RobotPose, but
* provided by the mocap system
*/
STREAMABLE_WITH_BASE(MocapRobotPose, RobotPose,
{ 
  /** Draws the robot pose to the field view*/
  void draw() const,

  (unsigned int)(0) mocapFrameNumber,
  (unsigned int)(0) timestamp,
});


/**
* @struct RobotPoseAfterPreview
* The same as the RobotPose, including 
* the preview phase of Dortmund Walking Engine.
*/
STREAMABLE_WITH_BASE(RobotPoseAfterPreview, RobotPose,
{
  void draw() const,
});

STREAMABLE_WITH_BASE(FixedOdometryRobotPose, RobotPose,
{ ,
});

/**
 * @struct RobotPoseCompressed
 * A compressed version of RobotPose used in team communication
 */
STREAMABLE(RobotPoseCompressed,
{
  RobotPoseCompressed() = default;
  RobotPoseCompressed(const RobotPose& robotPose);
  operator RobotPose() const,

  (Vector2s) translation,
  (short) rotation,
  (unsigned char) validity,
  (unsigned char) symmetry,
});
