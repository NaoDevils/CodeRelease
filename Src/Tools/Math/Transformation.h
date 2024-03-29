/**
 * @file Tools/Math/Transformation.h
 *
 * Declares a class that contains a set of static methods for
 * coordinate system transformations.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Eigen.h"

struct CameraMatrix;
struct CameraInfo;

/**
 * The class Transformation defines methods for
 * coordinate system transformations
 */
class Transformation
{
public:
  /**
   * Perform a transformation from 2D relative robot coordinates
   * to absolute field coordinates.
   * @param  rp Current robot pose
   * @param  relPos A position relative to the robot
   * @return Returns the position in absolute field coordinates
   */
  static Vector2f robotToField(const Pose2f& rp, const Vector2f& relPos);

  /**
   * Function does the transformation from absolute 2D field coordinates
   * to coordinates relative to the robot.
   * @param  rp Current robot pose
   * @param  fieldPos A position in absolute field coordinates
   * @return Returns the positon relative to the robot
   */
  static Vector2f fieldToRobot(const Pose2f& rp, const Vector2f& fieldPos);

  /**
   * Function does the transformation of velocity from 2D relative robot
   * coordinates to absolute field coordinates.
   * @param rp Current robot pose
   * @param relVel A velocity vector relative to robot
   * @return Returns the velocity in absolute field coordinates
   */
  static Vector2f robotToFieldVelocity(const Pose2f& rp, const Vector2f& relVel);

  /**
   * Function does the transformation of velocity from absolute 2D field
   * coordinates to coordinates relative to the robot.
   * @param rp Current robot pose
   * @param fieldVel A velocity vector in absolute field coordinates
   * @return Returns the velocity relative to the robot
   */
  static Vector2f fieldVelocityToRobot(const Pose2f& rp, const Vector2f& fieldVel);

  /**
   * Computes a position relative to the robot given a position of
   * a pixel in the image.
   * @param x Specifies the x-coordinate of the pixel.
   * @param y Specifies the y-coordinate of the pixel.
   * @param cameraMatrix The extrinsic camera parameters
   * @param cameraInfo The intrinsic camera parameters
   * @param relativePosition The resulting point
   */
  [[nodiscard]] static bool imageToRobot(const float x, const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& relativePosition);

  /**
   * Computes a position relative to the robot given a position of
   * a pixel in the image.
   * (integer version, calls overloaded float version, returns result converted to int)
   * @param x Specifies the x-coordinate of the pixel.
   * @param y Specifies the y-coordinate of the pixel.
   * @param cameraMatrix The extrinsic camera parameters
   * @param cameraInfo The intrinsic camera parameters
   * @param relativePosition The resulting point
   */
  [[nodiscard]] static bool imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2i& relativePosition);
  [[nodiscard]] static bool imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& relativePosition);

  /**
   * Computes a position relative to the robot given a position of
   * a pixel in the image.
   * @param pointInImage The point in the image
   * @param cameraMatrix The extrinsic camera parameters
   * @param cameraInfo The intrinsic camera parameters
   * @param relativePosition The resulting point
   */
  [[nodiscard]] static bool imageToRobot(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& relativePosition);
  [[nodiscard]] static bool imageToRobot(const Vector2f& pointInImage, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& relativePosition);

  /**
   * Computes a position relative to the robot on a horizontal plane
   * given a position of a pixel in the image as well as the distance of the plane from the ground.
   * @param pointInImage The point in the image
   * @param z The height of the horizontal plane (relative to the ground)
   * @param cameraMatrix The extrinsic camera parameters
   * @param cameraInfo The intrinsic camera parameters
   * @param pointOnPlane The resulting point
   */
  [[nodiscard]] static bool imageToRobotHorizontalPlane(const Vector2f& pointInImage, float z, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& pointOnPlane);

  /**
   * Calculates where a relative point in the world appears in an image.
   * @param point The coordinates of the point relative to the robot's origin.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointInImage The resulting point.
   * @return The result is valid, i.e. the point is in front of the camera. That
   *         still does not mean that the point is within the bounds of the image.
   */
  [[nodiscard]] static bool robotToImage(const Vector3f& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& pointInImage);
  [[nodiscard]] static bool robotToImage(const Vector2f& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& pointInImage);
  /**
   * Calculated where a point relative to the robot and rotated by the z-axis of
   * the camera appears in the image. The point of this method is to easily manipulate relative
   * point on the field from the cameras point of view.
   * @param point The coordinates of the point relative to the robot's origin and rotated by the camera z-axis.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointInImage The resulting point.
   * @return The result is valid, i.e. the point is in front of the camera. That
   *         still does not mean that the point is within the bounds of the image.
   */
  [[nodiscard]] static bool robotWithCameraRotationToImage(const Vector2f& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& pointInImage);

  /**
   * Computes a position relative to the robot and rotated by the z-axis of the camera
   * given a position of a pixel in the image.
   *
   * @param pointInImage The point in the image
   * @param cameraMatrix The extrinsic camera parameters
   * @param cameraInfo The intrinsic camera parameters
   * @param relativePosition The resulting point
   */
  [[nodiscard]] static bool imageToRobotWithCameraRotation(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& relativePosition);
};
