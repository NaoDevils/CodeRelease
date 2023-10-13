/**
* @file IMUModelProvider.h
* 
* Declaration of module IMUModelProvider
*
* @author <a href="mailto:kaden@informatik.hu-berlin.de">Steffen Kaden</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/


#pragma once


#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Math/RotationMatrix.h"

#include "UnscentedKalmanFilter.h"
#include "IMURotationMeasurement.h"
#include "IMURotationState.h"

STREAMABLE(IMUModelAccelerationFilterParameters,,
  (float)(0.01f) processNoiseAcc,   // [m^2/s^4]
  (float)(1.f) measurementNoiseAcc // [m^2/s^4]
);

STREAMABLE(IMUModelRotationFilterParameters,,
  (float)(0.01f) processNoiseRot,   // [rad^2]
  (float)(0.01f) processNoiseGyro,  // [rad^2/s^2]
  (float)(10.f) measurementNoiseAcc,// [m^2/s^4]
  (float)(1.f) measurementNoiseGyro// [rad^2/s^2]
);

MODULE(IMUModelProvider,
  REQUIRES(InertialSensorData),
  REQUIRES(FrameInfo),
  REQUIRES(WalkingEngineParams),
  REQUIRES(RobotInfo),
  USES(WalkCalibration),
  USES(MotionInfo),
  PROVIDES(IMUModel),
  LOADS_PARAMETERS(,
    (bool)(true) enableWhileWalking,
    (IMUModelAccelerationFilterParameters) standAcceleration,
    (IMUModelAccelerationFilterParameters) walkAcceleration,
    (IMUModelRotationFilterParameters) standRotation,
    (IMUModelRotationFilterParameters) walkRotation
  )
);


class IMUModelProvider : public IMUModelProviderBase
{
public:
  IMUModelProvider();
  virtual ~IMUModelProvider();

  void update(IMUModel& imuModel);
  void writeIMUData(IMUModel& imuModel);
  void reset();

private:
  void initRotationMatrix(Matrix3f& rm, const Vector3f& vec);

  /* filter for rotation */
  UKF<RotationState<Measurement<6>, 6>> ukf_rot;

  typedef Measurement<6> IMU_RotationMeasurement;

  Eigen::Matrix<double, 6, 6> Q_rotation;
  Eigen::Matrix<double, 6, 6> Q_rotation_walk;
  Eigen::Matrix<double, 6, 6> R_rotation;
  Eigen::Matrix<double, 6, 6> R_rotation_walk;

  /* filter for global acceleration */
  UKF<State<Measurement<3>, 3>> ukf_acc_global;

  typedef Measurement<3> IMUAccMeasurementGlobal;

  Eigen::Matrix<double, 3, 3> Q_acc;
  Eigen::Matrix<double, 3, 3> Q_acc_walk;
  Eigen::Matrix<double, 3, 3> R_acc;
  Eigen::Matrix<double, 3, 3> R_acc_walk;

private: /* small helper */
  unsigned timeStampIsWalking = 0;
  bool isWalking = false;
  bool isActive = true, lastActive = true;

  template <typename T> static Eigen::Vector<T, 3> quaternionToRotationVector(const Eigen::Quaternion<T>& q)
  {
    const Eigen::AngleAxis<T> temp(q);
    return temp.angle() * temp.axis();
  }

  Eigen::Quaterniond integrated;

  void reloadParameters();
};
