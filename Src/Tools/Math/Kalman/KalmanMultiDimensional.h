/**
 * \file KalmanMultiDimensional.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Template class for an arbitrary kalman filter.
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
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
template <class V = double, unsigned int x = 1, unsigned int u = 1, unsigned int z = 1>
STREAMABLE(KalmanMultiDimensional,
  /**
   * \struct KalmanMatrices::Noise
   *
   * \brief Noise matrices of the kalman filter.
   *
   * Included matrices:
   * - process noise covariance matrix (Q)
   * - measurement noise matrix (R)
   */
  STREAMABLE(Noise,,
    /// The process noise covariance matrix (Q).
    (Eigen::Matrix<V, x, x>) processNoiseCovarianceMatrix,
    /// The measurement noise matrix (R).
    (Eigen::Matrix<V, z, z>) measurementNoiseMatrix,

    /// The maximum measurement noise as a scalar value.
    /// The actual values of the \c measurementNoiseMatrix can vary.
    (V)(0) maxMeasurementNoise
  );

  /**
   * \struct KalmanMatrices
   * 
   * \brief Fix matrices of the kalman filter.
   * 
   * Included matrices:
   * - system matrix (A)
   * - input matrix (B)
   * - measurement matrix (H)
   * - Noise:
   *   - process noise covariance matrix (Q)
   *   - measurement noise matrix (R)
   */
  STREAMABLE(KalmanMatrices,
    using Noise = Noise;
    ,
    /// The system matrix (A).
    (Eigen::Matrix<V, x, x>) systemMatrix,
    /// The input matrix (B).
    (Eigen::Matrix<V, x, u>) inputMatrix,
    /// The measurement matrix (H).
    (Eigen::Matrix<V, z, x>) measurementMatrix,
    /// Noise matrices of the kalman filter.
    (Noise) noise
  );

  /// Fix matrices of the kalman filter.
  KalmanMatrices matrices;

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
    covarianceMatrix = matrices.systemMatrix * covarianceMatrix * matrices.systemMatrix.transpose() + matrices.noise.processNoiseCovarianceMatrix;
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
  void correct(const Eigen::Matrix<V, z, 1>& measurement, V measurementNoiseFactor = 1);
  
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
               const Eigen::Matrix<V, z_tmp, z_tmp> measurementNoiseMatrix_tmp,
	           V measurementNoiseFactor = 1);
    
  /**
   * \struct KalmanStateTransformation
   *
   * Extension to a standard kalman filter.
   *
   * Encapsulates matrices describing a transformation (translation and rotation)
   * of the kalman state which can be used to compensate a changing reference frame.
   */
  STREAMABLE(KalmanStateTransformation,,
    /// This rotation matrix (R) is applied to the state vector.
    (Eigen::Matrix<V, x, x>) stateRotationMatrix,

    /// This translation vector (t) is added to the state vector.
    (Eigen::Matrix<V, x, 1>) stateTranslationVector
  );

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

  /**
   * \brief Computes the Mahalanobis distance to another kalman filter
   * 
   * This function is used to compare two kalman filter states based on the
   * state vector and the covariance matrix for checking, if they describe
   * the same phenomenon.
   * \param other Another kalman filter
   * \return Distance between \c *this and \c other (value without any unit).
   */
  V distanceTo(const KalmanMultiDimensional<V, x, u, z>& other) const;
  ,

  /// The estimated state (x).
  (Eigen::Matrix<V, x, 1>) state,
  /// The current covariance matrix (P).
  (Eigen::Matrix<V, x, x>) covarianceMatrix
);

template <class V, unsigned int x, unsigned int u, unsigned int z>
void KalmanMultiDimensional<V, x, u, z>::correct(const Eigen::Matrix<V, z, 1>& measurement, V measurementNoiseFactor)
{
  // S = HPH^T + R
  Eigen::Matrix<V, z, z> innovationCovarianceMatrix = matrices.measurementMatrix * covarianceMatrix * matrices.measurementMatrix.transpose() + matrices.noise.measurementNoiseMatrix * measurementNoiseFactor;
  // K = P*H^T*S^(-1)
  Eigen::Matrix<V, x, z> K = covarianceMatrix * matrices.measurementMatrix.transpose() * innovationCovarianceMatrix.inverse();
  // x = x + K(z - Hx)
  state = state + K * (measurement - (matrices.measurementMatrix * state));
  // P = (I - KH)P
  covarianceMatrix = (Eigen::Matrix<V, x, x>::Identity() - K * matrices.measurementMatrix) * covarianceMatrix;
}

template <class V, unsigned int x, unsigned int u, unsigned int z>
template <unsigned int z_tmp>
void KalmanMultiDimensional<V, x, u, z>::correct(
    const Eigen::Matrix<V, z_tmp, 1>& measurement, const Eigen::Matrix<V, z_tmp, x> measurementMatrix_tmp, const Eigen::Matrix<V, z_tmp, z_tmp> measurementNoiseMatrix_tmp, V measurementNoiseFactor)
{
  // S = HPH^T + R
  Eigen::Matrix<V, z_tmp, z_tmp> innovationCovarianceMatrix = measurementMatrix_tmp * covarianceMatrix * measurementMatrix_tmp.transpose() + measurementNoiseMatrix_tmp * measurementNoiseFactor;
  // K = P*H^T*S^(-1)
  Eigen::Matrix<V, x, z_tmp> K = covarianceMatrix * measurementMatrix_tmp.transpose() * innovationCovarianceMatrix.inverse();
  // x = x + K(z - Hx)
  state = state + K * (measurement - (measurementMatrix_tmp * state));
  // P = (I - KH)P
  covarianceMatrix = (Eigen::Matrix<V, x, x>::Identity() - K * measurementMatrix_tmp) * covarianceMatrix;
}

template <class V, unsigned int x, unsigned int u, unsigned int z> V KalmanMultiDimensional<V, x, u, z>::distanceTo(const KalmanMultiDimensional<V, x, u, z>& other) const
{
  Eigen::Matrix<V, x, 1> diff(state - other.state);
  Eigen::Matrix<V, x, x> cov(covarianceMatrix + other.covarianceMatrix);
  return diff.dot((cov.inverse() * diff));
}
