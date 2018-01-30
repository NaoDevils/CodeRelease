/**
 * \file KalmanPositionTracking.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Template class for kalman filters to track 2-dimensional positions and velocities.
 */

#pragma once

#include "KalmanMultiDimensional.h"

/**
 * \class KalmanPositionTracking2D
 * 
 * A specialization of the arbitrary kalman filter for tracking of a two-dimensional 
 * position. The state vector consists of position and velocity (each 2D).
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
template <class V = double>
class KalmanPositionTracking2D : public KalmanMultiDimensional<V, 4, 2, 2>
{
private:
  /** Measurement matrix for the special case of a 4-dimensional measurement.
   * This is used by the correct method with position and <b>\c velocity</b> arguments. */
  Eigen::Matrix<V, 4, 4> measurementMatrix4D;
  /** Measurement noise matrix for the special case of a 4-dimensional measurement.
   * This is used by the correct method with position and <b>\c velocity</b> arguments. */
  Eigen::Matrix<V, 4, 4> measurementNoiseMatrix4D;

  /**
   * Initialize measurement matrices for the special case of a 4-dimensional measurement.
   * This is used by the \c correct(..) method with \c position and <b>\c velocity</b> arguments.
   * 
   * IMPORTANT: Execute this method after initializing \c kalmanMultiDimensional<V, 4, 2, 2>::matrices
   *            by one of the \c KalmanPositionTracking2D::initialize(..) methods.
   */
  void initializeMeasurement4D()
  {
    // Create measurement matrix for 4-dimensional measurement.
    measurementMatrix4D = Eigen::Matrix<V, 4, 4>::Identity();
    // Create measurement noise matrix for 4-dimensional measurement.
    measurementNoiseMatrix4D = Eigen::Matrix<V, 4, 4>::Identity();
    // Copy default 2x2 measurement noise matrix to top-left and bottom-right
    // corners of new 4x4 matrix.
    measurementNoiseMatrix4D.topLeftCorner(2, 2) = KalmanMultiDimensional<V, 4, 2, 2>::matrices.noise.measurementNoiseMatrix;
    measurementNoiseMatrix4D.bottomRightCorner(2, 2) = KalmanMultiDimensional<V, 4, 2, 2>::matrices.noise.measurementNoiseMatrix;
  }

public:
  /** \return The position extracted from the state (in mm). */
  Vector2f position() const { return Vector2f(static_cast<float>(this->state(0)), static_cast<float>(this->state(1))); }
  /** Same as position().x
   * \return The x-coordinate of the position extracted from the state (in mm). */
  float positionX() const { return static_cast<float>(this->state(0)); }
  /** Same as position().y
   * \return The y-coordinate of the position extracted from the state (in mm). */
  float positionY() const { return static_cast<float>(this->state(1)); }
  
  /** \return The position part of the covariance matrix (in mm^2). */
  Matrix2f positionCovariance() const { return this->covarianceMatrix.topLeftCorner(2, 2).template cast<float>(); }
  
  /** \return The velocity extracted from the state (in mm/s). */
  Vector2f velocity() const { return Vector2f(static_cast<float>(this->state(2)), static_cast<float>(this->state(3))); }
  /** Same as \c velocity().x
   * \return The x-coordinate of the velocity extracted from the state (in mm/s). */
  float velocityX() const { return static_cast<float>(this->state(2)); }
  /** Same as \c velocity().y
   * \return The y-coordinate of the velocity extracted from the state (in mm/s). */
  float velocityY() const { return static_cast<float>(this->state(3)); }

  /** \return The velocity part of the covariance matrix (in (mm/s)^2). */
  Matrix2f velocityCovariance() const { return this->covarianceMatrix.bottomRightCorner(2, 2).template cast<float>(); }

  /**
   * Initializes the kalman filter with the given \c KalmanMatrices.
   * \param position The initial position (in mm).
   * \param velocity The initial velocity (in mm/s).
   * \param covarianceMatrix The initial variance matrix.
   * \param kalmanNoiseMatrices Fix matrices of the kalman filter.
   */
  void initialize(const Vector2f& position, const Vector2f& velocity, const Eigen::Matrix<V, 4, 4>& covarianceMatrix,
                  const typename KalmanMultiDimensional<V, 4, 2, 2>::KalmanMatrices::Noise& kalmanNoiseMatrices)
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

	  // Set noise matrices of the underlying kalman filter.
	  KalmanMultiDimensional<V, 4, 2, 2>::matrices.noise = kalmanNoiseMatrices;

    initializeMeasurement4D();
  }

  /**
   * Initializes the kalman filter with default matrices. Don't use this!
   * \param position The initial position (in mm).
   * \param velocity The initial velocity (in mm/s).
   * \param covarianceMatrix The initial variance matrix.
   */
  void initialize(const Vector2f& position, const Vector2f& velocity, const Eigen::Matrix<V, 4, 4>& covarianceMatrix)
  {
    // Fill noise matrices of the underlying kalman filter.
    // IMPORTANT: They have to be adjusted to the use case!
    typename KalmanPositionTracking2D<V>::KalmanMatrices::Noise noise;

    // The process noise covariance matrix (Q):
    noise.processNoiseCovarianceMatrix <<
      1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0,
      1.0, 0.0, 1.0, 0.0,
      0.0, 1.0, 0.0, 1.0;
    // The measurement noise matrix (R):
    noise.measurementNoiseMatrix <<
      10.0, 0.0,
      0.0, 10.0;

    initialize(position, velocity, covarianceMatrix, noise);
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
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(0, 2) = static_cast<V>(timeOffset);
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(1, 3) = static_cast<V>(timeOffset);

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
  void correct(const Vector2f& position, float timeOffset, float measurementNoiseFactor = 1.f)
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
   * the measurement size has to be changed temporarily to 4 dimensions.
   * 
   * \warning Only use this method if you realy need to correct the velocity.
   *          Otherwise use method <tt>correct(const Vector2f& position, float timeOffset, 
   *          float measurementNoiseFactor)</tt>.
   * \param [in] position The measured position (in mm).
   * \param [in] velocity The measured velocity (in mm/s).
   * \param [in] measurementNoiseFactor Increase measurement noise by
   *                                    multiplication with this factor (> 0).
   */
  void correct(const Vector2f& position, const Vector2f& velocity, float measurementNoiseFactor = 1.f)
  {
    ASSERT(measurementNoiseFactor >= 0.f);
    
    // Create measurement vector from position an dvelocity.
    Eigen::Matrix<V, 4, 1> measurement;
    measurement << position.x(), position.y(), velocity.x(), velocity.y();
    
    // Run correction with changed system matrix to make position and velocity
    // independent from eachother because they are both measured.
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(0, 2) = 0.f;
    KalmanMultiDimensional<V, 4, 2, 2>::matrices.systemMatrix(1, 3) = 0.f;

    // Run correction temporarily with 4 dimensional measurement (noise) matrix.
    KalmanMultiDimensional<V, 4, 2, 2>::template correct<4>(measurement,
		measurementMatrix4D, measurementNoiseMatrix4D, measurementNoiseFactor);
  }

  /**
   * \brief Computes the Mahalanobis distance to the position part of another kalman filter
   *
   * This function is used to compare two kalman filter states based on the
   * position part of the state vector (first two entries) and the covariance 
   * matrix (top-left 2x2 corner) for checking, if they describe the same 
   * phenomenon.
   * \param other Another kalman filter
   * \return Distance between \c *this and \c other (value without any unit).
   */
  float positionDistanceTo(const KalmanPositionTracking2D<V>& other) const
  {
    Eigen::Vector2f diff(position() - other.position());
    Eigen::Matrix2f cov(positionCovariance() + other.positionCovariance());
    return diff.dot((cov.inverse() * diff));
  }

  /**
   * \brief Computes the Mahalanobis distance to the velocity part of another kalman filter
   *
   * This function is used to compare two kalman filter states based on the
   * velocity part of the state vector (last two entries) and the covariance
   * matrix (bottom-right 2x2 corner) for checking, if they describe the same
   * phenomenon.
   * \param other Another kalman filter
   * \return Distance between \c *this and \c other (value without any unit).
   */
  float velocityDistanceTo(const KalmanPositionTracking2D<V>& other) const
  {
    Eigen::Vector2f diff(velocity() - other.velocity());
    Eigen::Matrix2f cov(velocityCovariance() + other.velocityCovariance());
    return diff.dot((cov.inverse() * diff));
  }
};
