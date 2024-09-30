/**
 * @file InverseKinematic.h
 * @author Alexander Härtl
 * @author jeff
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"

struct CameraCalibration;
struct JointAngles;
struct JointCalibration;
struct Pose3f;
struct RobotDimensions;

namespace InverseKinematic
{
  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg.
   * @param positionLeft The desired position (translation + rotation) of the left foots ankle point.
   * @param positionRight The desired position (translation + rotation) of the right foots ankle point.
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The Robot Dimensions needed for calculation.
   * @param ratio The ratio between the left and right yaw angle.
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target).
   */
  [[nodiscard]] bool calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio = 0.5f);

  /**
   * This method calculates the joint angles for the legs of the robot from a Pose3f for each leg and the body ptch and roll.
   * @param positionLeft The desired position (translation + rotation) of the left foots point
   * @param positionRight The desired position (translation + rotation) of the right foots point
   * @param bodyRotation The rotation of the body around the x-Axis and y-Axis
   * @param jointAngles The instance of JointAngles where the resulting joint angles are written into.
   * @param robotDimensions The RobotDimensions needed for calculation
   * @param ratio The ratio between the left and right yaw angle
   * @return Whether the target position was reachable or not (if the given target position is not reachable the computation proceeds using the closest reachable position near the target)
   */
  [[nodiscard]] bool calcLegJoints(
      const Pose3f& positionLeft, const Pose3f& positionRight, const Vector2f& bodyRotation, JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio = 0.5f);
  [[nodiscard]] bool calcLegJoints(
      const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation, JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio = 0.5f);

  /**
   * Solves the inverse kinematics for the head of the Nao such that the camera looks at a certain point.
   */

  /**
   * @brief Solves the inverse kinematics for the head of the Nao such that the camera looks at a certain point.
   * @param position Point the camera should look at in cartesian space relative to the robot origin.
   * @param robotDimensions The robot dimensions needed for the calculation.
   * @param lowerCamera true if the joint angles are to be determined for the lower camera, false for the upper camera.
   * @param cameraCalibration The camera calibration
   * @param imageTilt Tilt angle at which the point should appear in the image (0: center of image, less than 0 => closer to the top of the image.)
   * @param imagePan Pan angle at which the point should appear in the image (0: center of image, less than 0 => closer to the right of the image.)
   * @return Vector [pan, tilt] containing the resulting joint angles.
  */
  Vector2a calcHeadJoints(
      const Vector3f& position, const RobotDimensions& robotDimensions, const bool lowerCamera, const CameraCalibration& cameraCalibration, const Angle imageTilt = 0_deg, const Angle imagePan = 0_deg);

}; // namespace InverseKinematic
