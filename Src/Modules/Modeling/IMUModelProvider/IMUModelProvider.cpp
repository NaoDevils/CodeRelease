/**
* @file IMUModel.cpp
*
* Definition of class IMUModel
*
* @author <a href="mailto:kaden@informatik.hu-berlin.de">Steffen Kaden</a>
* @url https://github.com/BerlinUnited/NaoTH/blob/develop/NaoTHSoccer/Source/Motion/SensorFilter/IMUModel.h
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/

#include "IMUModelProvider.h"

#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"


IMUModelProvider::IMUModelProvider() : integrated(1, 0, 0, 0)
{
  ukf_acc_global.P = Eigen::Matrix<double, 3, 3>::Identity(); // covariance matrix of current state
  ukf_rot.P = Eigen::Matrix<double, 6, 6>::Identity(); // covariance matrix of current state
  isWalking = false;
  isActive = true;
  lastActive = true;
  reloadParameters();
}

IMUModelProvider::~IMUModelProvider() {}

void IMUModelProvider::reset()
{
  ukf_rot.reset();
  ukf_acc_global.reset();
  integrated = Eigen::Quaterniond(1, 0, 0, 0);
  //reloadParameters();
}

void IMUModelProvider::update(IMUModel& imuModel)
{
  timeStampIsWalking = theMotionInfo.motion != MotionRequest::Motion::walk || theMotionInfo.walkRequest.isZeroSpeed() ? timeStampIsWalking : theFrameInfo.time;
  isWalking = theFrameInfo.getTimeSince(timeStampIsWalking) < 1000;
  isActive = theRobotInfo.transitionToFramework != 0.f && theRobotInfo.penalty == PENALTY_NONE;

  DECLARE_PLOT("module:IMUModelProvider:State:orientation:x");
  DECLARE_PLOT("module:IMUModelProvider:State:orientation:y");

  //TODO reset when necessary, e.g. after being lift or when coming from penalty
  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reset_filter")
  {
    reset();
  }

  if (isActive && !lastActive)
    reset();

  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reset_representation")
  imuModel = IMUModel();

  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reloadParameters")
  reloadParameters();

  /* handle ukf filter for rotation */
  if (isWalking)
  {
    ukf_rot.Q = Q_rotation_walk;
  }
  else
  {
    ukf_rot.Q = Q_rotation;
  }

  ukf_rot.generateSigmaPoints();

  // TODO: use generated angular acceleration as control vector?
  ukf_rot.predict(Vector3d::Zero(), theFrameInfo.cycleTime);

  // don't generate sigma points again because the process noise would be applied a second time
  // ukf.generateSigmaPoints();

  IMU_RotationMeasurement z;
  z << theInertialSensorData.acc.cast<double>().normalized(), theInertialSensorData.gyro.cast<double>();

  if (isWalking)
  {
    if (enableWhileWalking)
    {
      ukf_rot.update(z, R_rotation_walk);
    }
  }
  else
  {
    ukf_rot.update(z, R_rotation);
  }
  /* rotation ukf end */

  /* handle ukf filter for global acceleration */
  // transform acceleration measurement into global reference frame
  // TODO: Odometry as location measurement?
  // TODO: velocity of trunk in supfoot / local robot frame as velocity measurement
  // TODO: really needs bias removal or "calibration" of g
  const IMUAccMeasurementGlobal z_acc = ukf_rot.state.getRotationAsQuaternion()._transformVector(theInertialSensorData.acc.cast<double>());

  if (isWalking)
    ukf_acc_global.Q = Q_acc_walk;
  else
    ukf_acc_global.Q = Q_acc;

  ukf_acc_global.generateSigmaPoints();

  // TODO: use generated jerk as control vector?
  ukf_acc_global.predict(Vector3d::Zero(), theFrameInfo.cycleTime);

  if (isWalking)
  {
    if (enableWhileWalking)
      ukf_acc_global.update(z_acc, R_acc_walk);
  }
  else
    ukf_acc_global.update(z_acc, R_acc);

  /* acc ukf end */
  writeIMUData(imuModel);

  lastActive = isActive;
}

void IMUModelProvider::writeIMUData(IMUModel& imuModel)
{
  // raw sensor values
  imuModel.rotational_velocity_sensor = theInertialSensorData.gyro.cast<float>();
  imuModel.acceleration_sensor = theInertialSensorData.acc.cast<float>();

  // global position data
  // TODO: check for correct integration
  // TODO: prediction or state?
  imuModel.acceleration = ukf_acc_global.state.acceleration().col(0).cast<float>();
  imuModel.acceleration.z() -= theWalkCalibration.gravity;

  imuModel.location += imuModel.velocity * theFrameInfo.cycleTime + imuModel.acceleration * theFrameInfo.cycleTime * theFrameInfo.cycleTime * 0.5f;
  imuModel.velocity += imuModel.acceleration * theFrameInfo.cycleTime;

  // the state we are estimating in ukf_rot is (X * cycleTime) ms in the past. so predict (X * cycleTime) ms as estimate for the real current state
  UKF<RotationState<Measurement<6>, 6>> sensor_delay_corrected_rot = ukf_rot;
  sensor_delay_corrected_rot.generateSigmaPoints();
  sensor_delay_corrected_rot.predict(Vector3d::Zero(), theWalkingEngineParams.imuSensorDelayFrames * theFrameInfo.cycleTime);

  // store rotation in IMUData as a rotation vector
  imuModel.rotation = sensor_delay_corrected_rot.state.rotation().cast<float>();
  const RotationMatrix bodyIntoGlobalMapping(sensor_delay_corrected_rot.state.getRotationAsAngleAxisd().cast<float>());

  /*
    * Note: the following code lines use the inverse mapping, i.e. globalIntoBodyMapping, by using the third row of bodyIntoGlobalMapping's matrix representation
    * Note: The negative of the determined angles is the correct solution in this case because the projected "global" z axis is not pointing upwards in the torso's coordinate system.
    * The projected "global" z axis in the Body frame is NOT (0,0,1)!
    * So actually we want the inverse mapping.
    * Inverting the angle is sufficient because it's basically only a 2D rotation in the XZ or YZ plane
    * Note: using bodyIntoGlobalMapping would be wrong because the global frame can be rotated around z regarding the body
    * this would result in a redistribution of the inclination on the x,y axis.
    * (e.g. z rotation about 90° -> a rotation about the body's y axis becomes a rotation about the global x axis)
    * Note: don't use the y axis to determine the inclination in YZ plane because it might be mapped onto the x axis (a rotation about 90° around z)
    * this results in huge devation of the angles determined by atan2 because the projected y axis might end up in the second or third quadrant of the YZ plane
    */

  const Vector3f global_Z_in_body = bodyIntoGlobalMapping.row(2);
  const Quaternionf orientation_quaternion = Eigen::Quaternionf::FromTwoVectors(global_Z_in_body, Vector3f::UnitZ());
  imuModel.orientation_rotvec = quaternionToRotationVector(orientation_quaternion);
  const RotationMatrix bodyIntoGlobalMappingWithoutZ(orientation_quaternion);

  imuModel.orientation << static_cast<Angle>(-atan2f(bodyIntoGlobalMappingWithoutZ(1, 2), bodyIntoGlobalMappingWithoutZ(1, 1))),
      static_cast<Angle>(-atan2f(-bodyIntoGlobalMappingWithoutZ(0, 2), bodyIntoGlobalMappingWithoutZ(0, 0)));

  imuModel.rotational_velocity = ukf_rot.state.rotational_velocity().col(0).cast<float>();

  PLOT("module:IMUModelProvider:State:location:x", imuModel.location.x());
  PLOT("module:IMUModelProvider:State:location:y", imuModel.location.y());
  PLOT("module:IMUModelProvider:State:location:z", imuModel.location.z());
  PLOT("module:IMUModelProvider:State:velocity:x", imuModel.velocity.x());
  PLOT("module:IMUModelProvider:State:velocity:y", imuModel.velocity.y());
  PLOT("module:IMUModelProvider:State:velocity:z", imuModel.velocity.z());

  PLOT("module:IMUModelProvider:State:acceleration:x", imuModel.acceleration.x());
  PLOT("module:IMUModelProvider:State:acceleration:y", imuModel.acceleration.y());
  PLOT("module:IMUModelProvider:State:acceleration:z", imuModel.acceleration.z());
  PLOT("module:IMUModelProvider:State:rotation:x", toDegrees(imuModel.rotation.x()));
  PLOT("module:IMUModelProvider:State:rotation:y", toDegrees(imuModel.rotation.y()));
  PLOT("module:IMUModelProvider:State:rotation:z", toDegrees(imuModel.rotation.z()));
  PLOT("module:IMUModelProvider:State:rotational_velocity:x", toDegrees(imuModel.rotational_velocity.x()));
  PLOT("module:IMUModelProvider:State:rotational_velocity:y", toDegrees(imuModel.rotational_velocity.y()));
  PLOT("module:IMUModelProvider:State:rotational_velocity:z", toDegrees(imuModel.rotational_velocity.z()));
  PLOT("module:IMUModelProvider:State:orientation:x", toDegrees(imuModel.orientation.x()));
  PLOT("module:IMUModelProvider:State:orientation:y", toDegrees(imuModel.orientation.y()));
  PLOT("module:IMUModelProvider:State:orientation_rotvec:x", toDegrees(imuModel.orientation_rotvec.x()));
  PLOT("module:IMUModelProvider:State:orientation_rotvec:y", toDegrees(imuModel.orientation_rotvec.y()));
  PLOT("module:IMUModelProvider:State:orientation_rotvec:z", toDegrees(imuModel.orientation_rotvec.z()));

  PLOT("module:IMUModelProvider:Diff:angle:x", toDegrees(theInertialSensorData.angle.x() - imuModel.orientation.x()));
  PLOT("module:IMUModelProvider:Diff:angle:y", toDegrees(theInertialSensorData.angle.y() - imuModel.orientation.y()));
}

void IMUModelProvider::reloadParameters()
{
  /* parameters for the acceleration filter */
  // process noise
  Q_acc.setIdentity();
  Q_acc *= standAcceleration.processNoiseAcc;

  Q_acc_walk.setIdentity();
  Q_acc_walk *= walkAcceleration.processNoiseAcc;

  // measurement covariance matrix
  R_acc.setIdentity();
  R_acc *= standAcceleration.measurementNoiseAcc;

  R_acc_walk.setIdentity();
  R_acc_walk *= walkAcceleration.measurementNoiseAcc;

  /* parameters for the rotation filter */
  // process noise
  Q_rotation.setIdentity();
  Q_rotation.block<3, 3>(0, 0) *= standRotation.processNoiseRot;
  Q_rotation.block<3, 3>(3, 3) *= standRotation.processNoiseGyro;

  Q_rotation_walk.setIdentity();
  Q_rotation_walk.block<3, 3>(0, 0) *= walkRotation.processNoiseRot;
  Q_rotation_walk.block<3, 3>(3, 3) *= walkRotation.processNoiseGyro;

  // measurement covariance matrix
  R_rotation.setIdentity();
  R_rotation.block<3, 3>(0, 0) *= standRotation.measurementNoiseAcc;
  R_rotation.block<3, 3>(3, 3) *= standRotation.measurementNoiseGyro;

  R_rotation_walk.setIdentity();
  R_rotation_walk.block<3, 3>(0, 0) *= walkRotation.measurementNoiseAcc;
  R_rotation_walk.block<3, 3>(3, 3) *= walkRotation.measurementNoiseGyro;
}

MAKE_MODULE(IMUModelProvider, sensing)
