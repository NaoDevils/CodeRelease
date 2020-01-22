/**
* @file IMUModel.cpp
* 
* Definition of class IMUModel
*
* @author <a href="mailto:kaden@informatik.hu-berlin.de">Steffen Kaden</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/ 

#include "IMUModelProvider.h"

#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"


IMUModelProvider::IMUModelProvider() :
    integrated(1,0,0,0)
{
  ukf_acc_global.P = Eigen::Matrix<double,3,3>::Identity(); // covariance matrix of current state
  ukf_rot.P        = Eigen::Matrix<double,6,6>::Identity(); // covariance matrix of current state

  reloadParameters();
}

IMUModelProvider::~IMUModelProvider()
{
}

void IMUModelProvider::update(IMUModel &imuModel)
{

  DECLARE_PLOT("module:IMUModelProvider:Measurement:gyro:x");
  DECLARE_PLOT("module:IMUModelProvider:Measurement:gyro:y");
  DECLARE_PLOT("module:IMUModelProvider:Measurement:gyro:z");

  DECLARE_PLOT("module:IMUModelProvider:Measurement:acc:x");
  DECLARE_PLOT("module:IMUModelProvider:Measurement:acc:y");
  DECLARE_PLOT("module:IMUModelProvider:Measurement:acc:z");

  DECLARE_PLOT("module:IMUModelProvider:State:orientation:x");
  DECLARE_PLOT("module:IMUModelProvider:State:orientation:y");

  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reset_filter")
  {
    ukf_rot.reset();
    ukf_acc_global.reset();
    integrated = Eigen::Quaterniond(1, 0, 0, 0);
  }

  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reset_representation")
    imuModel = IMUModel();

  DEBUG_RESPONSE_ONCE("module:IMUModelProvider:reloadParameters")
    reloadParameters();

  /* handle ukf filter for rotation */
  if(theMotionInfo.motion == MotionRequest::Motion::walk)
  {
    ukf_rot.Q = Q_rotation_walk;
  } else {
    ukf_rot.Q = Q_rotation;
  }

  ukf_rot.generateSigmaPoints();

  Eigen::Vector3d u_rot(0,0,0); // TODO: use generated angular acceleration as control vector?
  ukf_rot.predict(u_rot, theFrameInfo.cycleTime);

  // don't generate sigma points again because the process noise would be applied a second time
  // ukf.generateSigmaPoints();

  Eigen::Vector3d gyro;
  // gyro z axis seems to measure in opposite direction (turning left measures negative angular velocity, should be positive)
  gyro << theInertialSensorData.gyro.x(), theInertialSensorData.gyro.y(), -theInertialSensorData.gyro.z();
  Eigen::Vector3d acceleration = Eigen::Vector3d(theInertialSensorData.acc.x(), theInertialSensorData.acc.y(), theInertialSensorData.acc.z());

  IMU_RotationMeasurement z;
  z << acceleration.normalized(), gyro;
  
  if (theMotionInfo.motion == MotionRequest::Motion::walk)
  {
    if(enableWhileWalking)
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
  IMU_AccMeasurementGlobal z_acc = ukf_rot.state.getRotationAsQuaternion()._transformVector(acceleration);

  if (theMotionInfo.motion == MotionRequest::Motion::walk)
    ukf_acc_global.Q = Q_acc_walk;
  else
    ukf_acc_global.Q = Q_acc;

  ukf_acc_global.generateSigmaPoints();

  Eigen::Vector3d u_acc(0,0,0); // TODO: use generated jerk as control vector?
  ukf_acc_global.predict(u_acc, theFrameInfo.cycleTime);

  if (theMotionInfo.motion == MotionRequest::Motion::walk)
  {
    if (enableWhileWalking)
      ukf_acc_global.update(z_acc, R_acc_walk);
  }
  else
    ukf_acc_global.update(z_acc, R_acc);

  /* acc ukf end */

  writeIMUData(imuModel);
}

void IMUModelProvider::writeIMUData(IMUModel &imuModel)
{
  // raw sensor values
  imuModel.rotational_velocity_sensor = theInertialSensorData.gyro.cast<float>();
  imuModel.acceleration_sensor        = theInertialSensorData.acc.cast<float>();

  // global position data
  // TODO: check for correct integration
  // TODO: prediction or state?
  imuModel.acceleration.x() = static_cast<float>(ukf_acc_global.state.acceleration()(0,0));
  imuModel.acceleration.y() = static_cast<float>(ukf_acc_global.state.acceleration()(1,0));
  imuModel.acceleration.z() = static_cast<float>(ukf_acc_global.state.acceleration()(2,0)) - gravity;

  imuModel.location += imuModel.velocity * theFrameInfo.cycleTime + imuModel.acceleration * theFrameInfo.cycleTime * theFrameInfo.cycleTime * 0.5f;
  imuModel.velocity += imuModel.acceleration * theFrameInfo.cycleTime;

  // the state we are estimating in ukf_rot is 20 ms in the past. so predict 20 ms as estimate for the real current state
  UKF<RotationState<Measurement<6>,6> > sensor_delay_corrected_rot = ukf_rot;
  sensor_delay_corrected_rot.generateSigmaPoints();
  Eigen::Vector3d u_rot(0,0,0);
  sensor_delay_corrected_rot.predict(u_rot, 2 * theFrameInfo.cycleTime);

  // store rotation in IMUData as a rotation vector
  imuModel.rotation = eigenVectorToVector3D(sensor_delay_corrected_rot.state.rotation()).cast<float>();
  RotationMatrix bodyIntoGlobalMapping(sensor_delay_corrected_rot.state.getRotationAsAngleAxisd().cast<float>());
  
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

  imuModel.orientation = Vector2a(static_cast<Angle>(-atan2f(-bodyIntoGlobalMapping(2, 1), bodyIntoGlobalMapping(2, 2))),
    static_cast<Angle>(-atan2f(bodyIntoGlobalMapping(2, 0), bodyIntoGlobalMapping(2, 2))));

  Eigen::Vector3d global_Z_in_body(bodyIntoGlobalMapping(2,0), bodyIntoGlobalMapping(2,1), bodyIntoGlobalMapping(2,2));
  imuModel.orientation_rotvec = quaternionToVector3D(Eigen::Quaterniond::FromTwoVectors(global_Z_in_body, Eigen::Vector3d(0,0,1))).cast<float>();

  imuModel.rotational_velocity.x() = static_cast<float>(ukf_rot.state.rotational_velocity()(0, 0));
  imuModel.rotational_velocity.y() = static_cast<float>(ukf_rot.state.rotational_velocity()(1, 0));
  imuModel.rotational_velocity.z() = static_cast<float>(ukf_rot.state.rotational_velocity()(2,0));

  PLOT("module:IMUModelProvider:Measurement:rotational_velocity:x", theInertialSensorData.gyro.x());
  PLOT("module:IMUModelProvider:Measurement:rotational_velocity:y", theInertialSensorData.gyro.y());
  PLOT("module:IMUModelProvider:Measurement:rotational_velocity:z", theInertialSensorData.gyro.z());
  PLOT("module:IMUModelProvider:Measurement:acc:x", theInertialSensorData.acc.x());
  PLOT("module:IMUModelProvider:Measurement:acc:y", theInertialSensorData.acc.y());
  PLOT("module:IMUModelProvider:Measurement:acc:z", theInertialSensorData.acc.z());
  PLOT("module:IMUModelProvider:Measurement:angle:x", toDegrees(theInertialSensorData.angle.x()));
  PLOT("module:IMUModelProvider:Measurement:angle:y", toDegrees(theInertialSensorData.angle.y()));

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
  Q_acc *= walkAcceleration.processNoiseAcc;
    
  // measurement covariance matrix
  R_acc.setIdentity();
  R_acc *= standAcceleration.measurementNoiseAcc;

  R_acc_walk.setIdentity();
  R_acc_walk *= walkAcceleration.measurementNoiseAcc;

  /* parameters for the rotation filter */
  // process noise
  Q_rotation.setIdentity();
  Q_rotation.block<3,3>(0,0) *= standRotation.processNoiseRot;
  Q_rotation.block<3,3>(3,3) *= standRotation.processNoiseGyro;

  Q_rotation_walk.setIdentity();
  Q_rotation.block<3,3>(0,0) *= walkRotation.processNoiseRot;
  Q_rotation.block<3,3>(3,3) *= walkRotation.processNoiseGyro;

  // measurement covariance matrix
  R_rotation.setIdentity();
  R_rotation.block<3,3>(0,0) *= standRotation.measurementNoiseAcc;
  R_rotation.block<3,3>(3,3) *= standRotation.measurementNoiseGyro;

  R_rotation_walk.setIdentity();
  R_rotation_walk.block<3,3>(0,0) *= walkRotation.measurementNoiseAcc;
  R_rotation_walk.block<3,3>(3,3) *= walkRotation.measurementNoiseGyro;
}

MAKE_MODULE(IMUModelProvider, sensing)
