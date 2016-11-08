/**
 * \file KalmanMultiDimensional.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 * Template class for an arbitrary kalman filter.
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"

/**
 * \class KalmanMultiDimensional
 * 
 * Implements an arbitrary kalman filter.
 * Prediction:
 *   x = Ax + Bu
 *   P = APA^T + Q
 * Correction:
 *   K = PH^T(HPH^T + R)^(-1)
 *   x = x + K(z - Hx)
 *   P = (I - KH)P
 * 
 * \tparam x The size of the state vector (x).
 * \tparam u The size of the input vector (u).
 * \tparam z The size of the measurement vector (z).
 */
template <class V = double, unsigned int x = 1, unsigned int u = 1, unsigned int z = 1> class KalmanMultiDimensional : public Streamable
{
public:
  /// The estimated state (x).
  Eigen::Matrix<V, x, 1> state;
  /// The current covariance matrix (P).
  Eigen::Matrix<V, x, x> covarianceMatrix;
  
  /**
   * \class KalmanMatrices
   * 
   * \brief Fix matrices of the kalman filter.
   * 
   * Includes:
   * - system matrix (A)
   * - input matrix (B)
   * - measurement matrix (H)
   * 
   * - process noise covariance matrix (Q)
   * - measurement noise matrix (R)
   */
  struct KalmanMatrices : Streamable
  {
    /// The system matrix (A).
    Eigen::Matrix<V, x, x> systemMatrix;
    /// The input matrix (B).
    Eigen::Matrix<V, x, u> inputMatrix;
    /// The measurement matrix (H).
    Eigen::Matrix<V, z, x> measurementMatrix;

    /// The process noise covariance matrix (Q).
    Eigen::Matrix<V, x, x> processNoiseCovarianceMatrix;
    /// The measurement noise matrix (R).
    Eigen::Matrix<V, z, z> measurementNoiseMatrix;

    /** Streaming */
    virtual void serialize(In *in, Out *out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(systemMatrix);
      STREAM(inputMatrix);
      STREAM(measurementMatrix);
      STREAM(processNoiseCovarianceMatrix);
      STREAM(measurementNoiseMatrix);
      STREAM_REGISTER_FINISH;
    }
  };

  /// Fix matrices of the kalman filter.
  KalmanMatrices matrices;
  

  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(state);
    STREAM(covarianceMatrix);
    //STREAM(matrices);
    STREAM_REGISTER_FINISH;
  }


  /**
   * Default constructor.
   */
  KalmanMultiDimensional() : KalmanMultiDimensional(Eigen::Matrix<V, x, 1>(), Eigen::Matrix<V, x, x>()) {}

  /**
   * Constructor.
   * \param state The initial state.
   * \param covarianceMatrix The initial covariance matrix.
   */
  KalmanMultiDimensional(const Eigen::Matrix<V, x, 1>& state, const Eigen::Matrix<V, x, x>& covarianceMatrix)
  {
    initialize(state, covarianceMatrix);
  }

  /**
   * Initializes the filter.
   * \param state The initial state.
   * \param covarianceMatrix The initial variance matrix.
   */
  void initialize(const Eigen::Matrix<V, x, 1>& state, const Eigen::Matrix<V, x, x>& covarianceMatrix)
  {
    this->state = state;
    this->covarianceMatrix = covarianceMatrix;
  }

  /**
   * \brief Prediction: x = Ax + Bu, P = APA^T + Q
   * 
   * Performs a prediction step.
   * \param input The input vector (e.g. measurement of odometry since last iteration).
   */
  void predict(const Eigen::Matrix<V, u, 1>& input)
  {
    // x = Ax + Bu
    state = matrices.systemMatrix * state + matrices.inputMatrix * input;
    // P = APA^T + Q
    covarianceMatrix = matrices.systemMatrix * covarianceMatrix * matrices.systemMatrix.transpose() + matrices.processNoiseCovarianceMatrix;
  }

  /**
   * \brief Correction: x = x + K(z - Hx), P = (I - KH)P
   *                    K = P*H^T*(HPH^T + R)^(-1)
   * 
   * Performs a correction step.
   * \param [in] measurement The measurement vector.
   * \param [in] measurementNoiseFactor Increase measurement noise by 
   *                                    multiplication with this factor (> 0).
   */
  void correct(const Eigen::Matrix<V, z, 1>& measurement, V measurementNoiseFactor = 1)
  {
    // S = HPH^T + R
    Eigen::Matrix<V, z, z> innovationCovarianceMatrix = matrices.measurementMatrix * covarianceMatrix * matrices.measurementMatrix.transpose() + matrices.measurementNoiseMatrix * measurementNoiseFactor;
    // K = P*H^T*S^(-1)
    Eigen::Matrix<V, x, z> K = covarianceMatrix * matrices.measurementMatrix.transpose() * innovationCovarianceMatrix.inverse();
    // x = x + K(z - Hx)
    state = state + K * (measurement - (matrices.measurementMatrix * state));
    // P = (I - KH)P
    covarianceMatrix = (Eigen::Matrix<V, x, x>::Identity() - K * matrices.measurementMatrix) * covarianceMatrix;
  }
  
  /**
   * \brief Correction: x = x + K(z - Hx), P = (I - KH)P
   *                    K = P*H^T*(HPH^T + R)^(-1)
   * 
   * Performs a correction step with another measurement size \c z_tmp then the 
   * standard measurement size \c z of this template class object. For that 
   * reason a new measurement matrix (H) and a new measurement noise matrix (R) 
   * are required. They are only used for performing this correction step and 
   * are not saved withing this object.
   * \param [in] measurement The measurement vector.
   * \param [in] measurementMatrix_tmp Temporary measurement matrix (H).
   * \param [in] measurementNoiseMatrix_tmp Temporary measurement noise matrix (R).
   */
  template <unsigned int z_tmp>
  void correct(const Eigen::Matrix<V, z_tmp, 1>& measurement,
               const Eigen::Matrix<V, z_tmp, x> measurementMatrix_tmp,
               const Eigen::Matrix<V, z_tmp, z_tmp> measurementNoiseMatrix_tmp)
  {
    // S = HPH^T + R
    Eigen::Matrix<V, z_tmp, z_tmp> innovationCovarianceMatrix = measurementMatrix_tmp * covarianceMatrix * measurementMatrix_tmp.transpose() + measurementNoiseMatrix_tmp;
    // K = P*H^T*S^(-1)
    Eigen::Matrix<V, x, z_tmp> K = covarianceMatrix * measurementMatrix_tmp.transpose() * innovationCovarianceMatrix.inverse();
    // x = x + K(z - Hx)
    state = state + K * (measurement - (measurementMatrix_tmp * state));
    // P = (I - KH)P
    covarianceMatrix = (Eigen::Matrix<V, x, x>::Identity() - K * measurementMatrix_tmp) * covarianceMatrix;
  }
    
  /**
   * \struct KalmanStateTransformation
   *
   * Extension to a standard kalman filter.
   *
   * Encapsulates matrices describing a transformation (translation and rotation)
   * of the kalman state which can be used to compensate a changing reference frame.
   */
  struct KalmanStateTransformation
  {
    /// This rotation matrix (R) is applied to the state vector.
    Eigen::Matrix<V, x, x> stateRotationMatrix;

    /// This translation vector (t) is added to the state vector.
    Eigen::Matrix<V, x, 1> stateTranslationVector;
  };

  /**
   * Transforms the kalman filter state and covariance matrix to compensate a 
   * changing reference frame.
   * \param transformation Encapsulates matrices describing a transformation
   *                       (see struct \c KalmanStateTransformation).
   */
  void transformState(const KalmanStateTransformation &transformation)
  {
    // Transform state by the given transformation:
    // x' = Rx + t
    state = transformation.stateRotationMatrix * state + transformation.stateTranslationVector;
    // Transform covariance matrix according to the state transformation:
    // P' = RPR^T
    // This equation is derivated by substituting x by x' in the definition of 
    // the covariance matrix: P = E[(x-E(x))(x-E(x))^T]
    covarianceMatrix = transformation.stateRotationMatrix * covarianceMatrix * transformation.stateRotationMatrix.transpose();
  }
};

/**
 * \class KalmanPositionTracking2D
 * 
 * A specialization of the arbitrary kalman filter for tracking a two-dimensional 
 * position. The state vector consists of position and velocity.
 * Only the position is measured and the velocity is directly taken from state 
 * vector. Input vector u is used to modify the velocity (it is added to the 
 * velocity part (velX, velY) of the state vector).
 * 
 * State vector: x = (posX, posY, velX, velY) in mm or mm/s
 * Input vector: u = (velX_Delta, velY_Delta) in mm/s
 * Measurement vector: z = (posX_Measure, posY_Measure) in mm
 * 
 * The template parameter V defines only the data type of the kalman filter. All
 * input and output (e.g. position and velocity) is done with data type \c float.
 */
template <class V = double> class KalmanPositionTracking2D : public KalmanMultiDimensional<V, 4, 2, 2>
{
public:
  /** \return The position extracted from the state (in mm). */
  Vector2f position() const { return Vector2f(static_cast<float>(this->state(0)), static_cast<float>(this->state(1))); }
  /** Same as position().x
   * \return The x-coordinate of the position extracted from the state (in mm). */
  float positionX() const { return static_cast<float>(this->state(0)); }
  /** Same as position().y
   * \return The y-coordinate of the position extracted from the state (in mm). */
  float positionY() const { return static_cast<float>(this->state(1)); }

  /** \return The velocity extracted from the state (in mm/s). */
  Vector2f velocity() const { return Vector2f(static_cast<float>(this->state(2)), static_cast<float>(this->state(3))); }
  /** Same as \c velocity().x
   * \return The x-coordinate of the velocity extracted from the state (in mm/s). */
  float velocityX() const { return static_cast<float>(this->state(2)); }
  /** Same as \c velocity().y
   * \return The y-coordinate of the velocity extracted from the state (in mm/s). */
  float velocityY() const { return static_cast<float>(this->state(3)); }

  /**
   * Initializes the kalman filter with the given \c KalmanMatrices.
   * \param position The initial position (in mm).
   * \param velocity The initial velocity (in mm/s).
   * \param covarianceMatrix The initial variance matrix.
   * \param kalmanMatrices Fix matrices of the kalman filter.
   */
  void initialize(const Vector2f& position, const Vector2f& velocity, const Eigen::Matrix<V, 4, 4>& covarianceMatrix,
                  const typename KalmanMultiDimensional<V, 4, 2, 2>::KalmanMatrices& kalmanMatrices)
  {
	  // Initialize the base class with given state (x) and covariance matrix (P):
	  Eigen::Matrix<V, 4, 1> state;
	  state << position.x(), position.y(), velocity.x(), velocity.y();
	  KalmanMultiDimensional<V, 4, 2, 2>::initialize(state, covarianceMatrix);

	  // Set all matrices of the underlying kalman filter.
	  KalmanMultiDimensional<V, 4, 2, 2>::matrices = kalmanMatrices;
  }

  /**
   * Initializes the kalman filter with default matrices.
   * \param position The initial position (in mm).
   * \param velocity The initial velocity (in mm/s).
   * \param covarianceMatrix The initial variance matrix.
   */
  void initialize(const Vector2f& position, const Vector2f& velocity, const Eigen::Matrix<V, 4, 4>& covarianceMatrix)
  {
    // Initialize the base class with given state (x) and covariance matrix (P):
    Eigen::Matrix<V, 4, 1> state;
    state << position.x(), position.y(), velocity.x(), velocity.y();
    KalmanMultiDimensional<V, 4, 2, 2>::initialize(state, covarianceMatrix);

    // Fill all matrices of the underlying kalman filter.

    // The system matrix (A):
    // Add the last velocity (in mm/s) to the new position (in mm).
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix <<
      1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

    // The input matrix (B):
    // Set input as velocity change. Add it to the velocity part of the 
    // state vector.
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.inputMatrix <<
      0.0, 0.0,
      0.0, 0.0,
      1.0, 0.0,
      0.0, 1.0;

    // The measurement matrix (H):
    // Only the position part of the state vector is measured.
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.measurementMatrix <<
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

    // The two noise matrices have to be adjusted to the use case.
    // The process noise covariance matrix (Q):
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.processNoiseCovarianceMatrix <<
      1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0,
      1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0;
    // The measurement noise matrix (R):
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.measurementNoiseMatrix <<
      10.0, 0.0,
      0.0, 10.0;
  }

  /**
   * Performs a prediction step.
   * \param [in] acceleration This value is added to the velocity part 
   *             (in mm/frame) of the state vector.
   * \param [in] timeOffset The time in seconds since the last prediction.
   */
  void predict(const Vector2f& acceleration, float timeOffset)
  {
    // Run prediction with changed system matrix (A') to take the time since last 
    // prediction into account.
    // The velocity is added partial to the position depending on the time since 
    // last prediction.
    // Default: x = Ax  + Bu = x + (velX, velY, 0, 0)     + Bu
    // Here:    x = A'x + Bu = x + (t*velX, t*velY, 0, 0) + Bu
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(0,2) = static_cast<V>(timeOffset);
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(1,3) = static_cast<V>(timeOffset);

    // Run prediction.
    KalmanMultiDimensional<V, 4, 2, 2>::predict(acceleration.cast<V>());
  }

  /**
   * Performs a prediction step.
   * Don't changes the velocity.
   * \param [in] timeOffset The time in seconds since the last prediction.
   */
  void predict(float timeOffset)
  {
    Vector2f acceleration;
    acceleration << 0.f, 0.f;
    predict(acceleration, timeOffset);
  }

  /**
   * Performs a correction step with measured position.
   * \param [in] position The measured position (in mm).
   * \param [in] timeOffset The time in seconds since the last prediction.
   * \param [in] measurementNoiseFactor Increase measurement noise by 
   *                                    multiplication with this factor (> 0).
   */
  void correct(const Vector2f& position, float timeOffset, float measurementNoiseFactor = 1)
  {
    ASSERT(measurementNoiseFactor >= 0.f);

    // Run correction with changed system matrix to take the time since last 
    // correction into account.
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(0, 2) = static_cast<V>(timeOffset);
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(1, 3) = static_cast<V>(timeOffset);

    // Run correction.
    KalmanMultiDimensional<V, 4, 2, 2>::correct(position.cast<V>(), static_cast<V>(measurementNoiseFactor));
  }
  
  /**
   * Performs a correction step with measured position and velocity. Therefore
   * the measurement size has to be changed temporarily to 4.
   * \param [in] position The measured position (in mm).
   * \param [in] velocity The measured velocity (in mm/s).
   * \param [in] measurementNoiseFactor Increase measurement noise by
   *                                    multiplication with this factor (> 0).
   */
  void correct(const Vector2f& position, const Vector2f& velocity, float measurementNoiseFactor = 1)
  {
    ASSERT(measurementNoiseFactor >= 0.f);
    
    // Create measurement vector from position an dvelocity.
    Eigen::Matrix<V, 4, 1> measurement;
    measurement << position.x(), position.y(), velocity.x(), velocity.y();
    
    // Create temporary measurement matrix according to measurement size 4.
    Eigen::Matrix<V, 4, 4> measurementMatrix_tmp = Eigen::Matrix<V, 4, 4>::Identity();
    // Create temporary measurement noise matrix according to measurement size 4.
    Eigen::Matrix<V, 4, 4> measurementNoiseMatrix_tmp = Eigen::Matrix<V, 4, 4>::Identity();
    // Copy default 2x2 measurement noise matrix to top-left and bottom-right
    // corners of new 4x4 matrix.
    measurementNoiseMatrix_tmp.topLeftCorner(2, 2) = KalmanMultiDimensional<V, 4, 2, 2>::matrices.measurementNoiseMatrix;
    measurementNoiseMatrix_tmp.bottomRightCorner(2, 2) = KalmanMultiDimensional<V, 4, 2, 2>::matrices.measurementNoiseMatrix;
    // Apply measurementNoiseFactor.
    measurementNoiseMatrix_tmp *= measurementNoiseFactor;
    
    // Run correction with changed system matrix to make position and velocity
    // independent from eachother because they are both measured.
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(0, 2) = 0.f;
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(1, 3) = 0.f;

    // Run correction.
    KalmanMultiDimensional<V, 4, 2, 2>::template correct<4>(measurement, measurementMatrix_tmp, measurementNoiseMatrix_tmp);
  }
};
