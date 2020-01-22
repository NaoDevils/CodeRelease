/**
 * @file GroundContactDetector.h
 * Declaration of module GroundContactDetector.
 * @author Colin Graf
 * @author Sebastian Hoose
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(GroundContactDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  REQUIRES(FsrSensorData),
  USES(MotionInfo),
  USES(TorsoMatrix),
  PROVIDES(GroundContactState),
  DEFINES_PARAMETERS(
  {,
    (float)(0.08f) noContactMinAccNoise,
    (float)(0.04f) noContactMinGyroNoise,
    (float)(0.01f) contactMaxAngleNoise,
    (float)(0.007f) contactAngleActivationNoise,
    (float)(20.f) contactMaxAccZ, // deactivate
    (float)(0.5f) minFsrFootContactThreshold, //minimum edge hight for foot contact detection
    (bool)(true) filterFrequency, //use the medium of 10 frequency measurements
    (float)(-2.f) lowpass_a, //inverse of breaking frequency
  }),
});

/**
 * @class GroundContactDetector
 * A module for sensor data filtering.
 */
class GroundContactDetector : public GroundContactDetectorBase
{
private:
  bool contact = false; /**< Whether the robot has ground contact or not */
  unsigned int contactStartTime = 0; /**< Time when the robot started having ground contact */
  bool useAngle = false; /**< Whether the estimated angle will be used to detect ground contact loss */

  RotationMatrix expectedRotationInv;
  RingBufferWithSum<Vector2f, 60> angleNoises = {Vector2f::Zero()};
  RingBufferWithSum<Vector3f, 60> accNoises = {Vector3f::Zero()};
  RingBufferWithSum<Vector2f, 60> gyroNoises = {Vector2f::Zero()};
  RingBufferWithSum<Vector3f, 60> accValues = {Vector3f::Zero()};
  RingBufferWithSum<Vector2f, 60> gyroValues = {Vector2f::Zero()};
  RingBufferWithSum<float, 5> calibratedAccZValues;

  //foot contact and frequency members
  bool init = false; /** true if buffers are filled */
  bool leftHasGroundContact = true; /** true if left foot has contact to foot */
  bool rightHasGroundContact = true; /** true if right foot has contact to foot */
  unsigned int lastTimestampRight = 0; /** timestamp of last first contact of right foot on grount */
  unsigned int lastTimestampLeft = 0; /** timestamp of last first contact of left foot on grount */
  float frqLeft = 0.f; /** current frequency of left foot */
  float frqRight = 0.f; /** current frequency of right foot */
  RingBufferWithSum<float,7> fsrBufferLeft; /** buffer of last 10 fsr sensor datas (avg of all 4 sensors) of left foot */
  RingBufferWithSum<float,7> fsrBufferRight; /** buffer of last 10 fsr sensor datas (avg of all 4 sensors) of right foot */
  RingBufferWithSum<float,5> frequencyBufferLeft; /** buffer of last 5 frequencies of left foot */
  RingBufferWithSum<float,5> frequencyBufferRight; /** buffer of last 5 frequencies of right foot */
  RingBufferWithSum<float,10> convolutionBufferLeft; /** buffer of last 15 fsr sensor datas (avg of all 4 sensors) of left foot */
  RingBufferWithSum<float,10> convolutionBufferRight; /** buffer of last 15 fsr sensor datas (avg of all 4 sensors) of right foot */

  /**
   * Updates the GroundContactState representation .
   * @param groundContactState The ground contact representation which is updated by this module.
   */
  void update(GroundContactState& groundContactState);

  /**
   * Calculates if the left and right foot each having contact to ground and its frequencies based on FSR sensors
   * (in walk use only)
   */
  void footContact(float& frequencyLeft, float& frequencyRight);
};
