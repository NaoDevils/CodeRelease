/**
 * @file RobotDimensions.h
 * Description of the dimensions of the NAO robot.
 * @author Cord Niehaus
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"


/**
 * The class represents information about a single sonar sensor,
 * i.e. its 2f pose in the robot's torso and its opening angle.
 */
STREAMABLE_WITH_BASE(SonarSensorInfo, Pose2f,);

/**
 * This representation contains all necessary dimensions of the robot.
 * The torso coordinate frame is considert to be in the middle between the hip joints.
 */
STREAMABLE(RobotDimensions,
  /**
   * x-offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getXOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? xOffsetNeckToLowerCamera : xOffsetNeckToUpperCamera; }

  /**
   * Height offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  float getZOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? zOffsetNeckToLowerCamera : zOffsetNeckToUpperCamera; }

  /**
   * Tilt of current camera against neck.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
  Angle getTiltNeckToCamera(bool lowerCamera) const { return lowerCamera ? tiltNeckToLowerCamera : tiltNeckToUpperCamera; },

  (float) yHipOffset,               //!< The y offset of the left hip.
  (float) upperLegLength,           //!< Length between leg joints HipPitch and KneePitch in z-direction.
  (float) lowerLegLength,           //!< Length between leg joints KneePitch and AnklePitch in z-direction.
  (float) footHeight,               //!< Height between the sole of the foot and the foot joint AnkleRoll.
  (float) hipToNeckLength,          //!< Height offset between hip and joint headYaw.

  (float) xOffsetNeckToLowerCamera, //!< Forward offset between joint headPitch and lower camera.
  (float) zOffsetNeckToLowerCamera, //!< Height offset between joint headPitch and lower camera.
  (Angle) tiltNeckToLowerCamera,    //!< Tilt of lower camera against joint headPitch.

  (float) xOffsetNeckToUpperCamera, //!< Forward offset between joint headPitch and upper camera.
  (float) zOffsetNeckToUpperCamera, //!< Height offset between joint headPitch and upper camera.
  (Angle) tiltNeckToUpperCamera,    //!< Tilt of upper camera against joint headPitch.

  (Vector3f) armOffset,             //!< The offset of joint lShoulderPitch relative to the torso coordinate frame (y must be negated for right arm).
  (float) yOffsetElbowToShoulder,   //!< The offset between the joints lShoulderRoll and lElbowYaw in y (must be negated for right arm).
  (float) upperArmLength,           //!< The length between the joints ShoulderRoll and ElbowYaw in x-direction.
  (float) lowerArmLength,           //!< The length of the lower arm starting at ElbowRoll.
  (float) xOffsetElbowToWrist,      //!< The length from Elbow to WristJoint.
  (Vector3f) handOffset,            //!< The offset of a hand relative to his wrist coordinate frame.
  (float) handRadius,               //!< The radius of a virtuel sphere a hand can span.
  (float) armRadius,                //!< The radius of a arm.

  (Vector3f) imuOffset,             //!< The offset of the imu relative to the torso coordinate frame.

  (std::vector<Vector2f>) leftFsrPositions,   //!< The positions of the fsr on the left foot. --> fl, fr, bl, br
  (std::vector<Vector2f>) rightFsrPositions,  //!< The positions of the fsr on the right foot. --> fl, fr, bl, br

  // for NDD kinematic
  (float) footFront,               //!< The offset of the foot joints to the foot tip.
  (float) footBack,                //!< The offset of the foot joints to the back of the foot.
  (float) footOuter,               //!< The offset of the foot joints to the outer end of the foot.
  (float) footInner,               //!< The offset of the foot joints to the inner end of the foot.

  (SonarSensorInfo) leftSonarInfo,
  (SonarSensorInfo) rightSonarInfo,
  (Angle) sonarOpeningAngle
);
