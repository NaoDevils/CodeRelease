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
#include "Representations/Sensing/InertialData.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Math/RotationMatrix.h"

#include "UnscentedKalmanFilter.h"
#include "IMURotationMeasurement.h"
#include "IMURotationState.h"

STREAMABLE(IMUModelAccelerationFilterParameters,
{ ,
  (float)(0.01f) processNoiseAcc,
  (float)(1.f) measurementNoiseAcc,
});

STREAMABLE(IMUModelRotationFilterParameters,
{ ,
  (float)(0.01f) processNoiseRot,
  (float)(0.01f) processNoiseGyro,
  (float)(10.f) measurementNoiseAcc,
  (float)(1.f) measurementNoiseGyro,
});

MODULE(IMUModelProvider,
{ ,
  REQUIRES(InertialSensorData),
  REQUIRES(FrameInfo),
  USES(MotionInfo),
  PROVIDES(IMUModel),
  LOADS_PARAMETERS(
  {,
    (bool)(true) enableWhileWalking,
    (IMUModelAccelerationFilterParameters) standAcceleration,
    (IMUModelAccelerationFilterParameters) walkAcceleration,
    (IMUModelRotationFilterParameters) standRotation,
    (IMUModelRotationFilterParameters) walkRotation,
    (float)(9.81f) gravity,
  }),
});


class IMUModelProvider : public IMUModelProviderBase
{
public:
  IMUModelProvider();
  virtual ~IMUModelProvider();

  void update(IMUModel &imuModel);
  void writeIMUData(IMUModel &imuModel);

private:
  void initRotationMatrix(Matrix3f& rm, const Vector3f& vec);

  /* filter for rotation */
  UKF<RotationState<Measurement<6>,6> > ukf_rot;

  typedef Measurement<6> IMU_RotationMeasurement;

  Eigen::Matrix<double,6,6> Q_rotation;
  Eigen::Matrix<double,6,6> Q_rotation_walk;
  Eigen::Matrix<double,6,6> R_rotation;
  Eigen::Matrix<double,6,6> R_rotation_walk;

  /* filter for global acceleration */
  UKF<State<Measurement<3>, 3> > ukf_acc_global;

  typedef Measurement<3> IMU_AccMeasurementGlobal;

  Eigen::Matrix<double,3,3> Q_acc;
  Eigen::Matrix<double,3,3> Q_acc_walk;
  Eigen::Matrix<double,3,3> R_acc;
  Eigen::Matrix<double,3,3> R_acc_walk;

private: /* small helper */
  Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond& q) const {
    Eigen::AngleAxisd temp(q);
    return temp.angle() * temp.axis();
  }

  Vector3d quaternionToVector3D(const Eigen::Quaterniond& q) const {
    Eigen::AngleAxisd temp(q);
    Eigen::Vector3d temp2(temp.angle() * temp.axis());
    return Vector3d(temp2(0),temp2(1),temp2(2));
  }

  Vector3d eigenVectorToVector3D(const Eigen::Vector3d& v) const {
    return Vector3d(v(0),v(1),v(2));
  }
      
  Eigen::Quaterniond integrated;

  void reloadParameters();
};
