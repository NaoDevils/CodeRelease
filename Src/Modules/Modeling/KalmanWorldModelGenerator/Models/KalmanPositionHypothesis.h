/**
 * \file KalmanPositionHypothesis.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Declaration of class \c KalmanPositionHypothesis.
 * This class represents the position state of an object (e.g. ball, obstacle)
 * and is used by the \c MultiKalmanModel.
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Kalman/KalmanPositionTracking2D.h"
#include "Tools/PerceptsPerSecond.h"

#include <map>

/**
  * \class KalmanPositionHypothesis
  *
  * This class represents the position state of an object (e.g. ball, obstacle)
  * including position and velocity filtered by a Kalman filter
  * (see \c KalmanPositionTracking2D). Multiple instances of this class
  * representing the same object(s) are stored in the \c MultiKalmanModel.
  * Thus objects with jumping percepts can be tracked.
  */
class KalmanPositionHypothesis : public virtual Streamable
{
public:
  KalmanPositionHypothesis()
      : lastPerception(Vector2f::Zero()), validity(0.f), timeWhenLastSeen(0), mergeFactor(1.f), m_lastMotionUpdateTimeStamp(0), m_numberOfSensorUpdates(0), m_perceptsPerSecond(1000)
  {
    Vector2f zeroVector = Vector2f::Zero();
    initialize(zeroVector, zeroVector);
  };

  /**
   * \brief Constructor with initialization.
   *
   * Creates a new \c KalmanPositionHypothesis object with a given initial position and
   * an optional initial velocity. Parameterizes the kalman filter with noise matrices.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter.
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The timestamp of the initial position measurement (in ms).
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] position The initial position (in mm).
   * \param [in] perceptDuration The duration for the PerceptPerSecond buffer.
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown (default).
   */
  KalmanPositionHypothesis(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      float initialValidity,
      unsigned timestamp,
      float perceptValidity,
      const Vector2f& position,
      unsigned perceptDuration,
      const Vector2f& velocity = Vector2f::Zero())
      : lastPerception(position), validity(initialValidity), timeWhenLastSeen(timestamp), mergeFactor(1.f), m_lastMotionUpdateTimeStamp(timestamp), m_numberOfSensorUpdates(1),
        m_perceptsPerSecond(perceptDuration)
  {
    initialize(kalmanNoiseMatrices, position, velocity);

    // Add percept to the pps tracker.
    m_perceptsPerSecond.addPercept(timestamp, perceptValidity);
  }

  /**
   * Destructor.
   */
  virtual ~KalmanPositionHypothesis(){};


  //MARK: Kalman filter related methods

protected:
  /**
   * Initializes this hypothesis and the underlying kalman filter with a given
   * initial position, an optional initial velocity. It does not initialize
   * the kalman filters noise matrices (default values). This should not be used!
   * \param [in] position The initial position (in mm).
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown.
   */
  void initialize(const Vector2f& position, const Vector2f& velocity = Vector2f::Zero());

public:
  /**
   * Initializes this hypothesis and the underlying kalman filter with a given
   * initial position and an optional initial velocity. It parameterizes the
   * kalman filter with custom noise matrices.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter.
   * \param [in] position The initial position (in mm).
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown.
   */
  void initialize(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices, const Vector2f& position, const Vector2f& velocity = Vector2f::Zero());


  /**
   * \brief Removes an odometry offset from the kalman filter state.
   *
   * Only required if this hypothesis tracks position and velocity in the relative
   * robot coordinate system: This method removes the influence of the robots odometry
   * offset on the kalman state and covariance.
   * \param [in] reverseOdometry The transformation which should be applied to the
   *                             kalman filter state to compensate the odometry offset.
   */
  void removeOdometry(const KalmanPositionTracking2D<double>::KalmanStateTransformation& reverseOdometry);

  /**
   * \brief Prediction
   *
   * This method performs the prediction step of the kalman filter.
   *
   * While moving the velocity of this hypothesis can be reduced due to friction.
   * The friction is approximated by subtracting the friction each second
   * from the velocity. This model is appropriate for a rolling ball.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \param [in] friction The deceleration due to friction (negative value; in m/s^2).
   */
  void motionUpdate(unsigned currentTimestamp, float friction);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the kalman filter.
   * \param [in] position The measured position (in mm).
   * \param [in] timestamp The timestamp when the position was measured (in ms).
   *                       The timestamp of the newest correction is always
   *                       saved to \c timeWhenLastSeen.
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] measurementNoiseFactor Scale measurement noise by
   *                                    multiplication with this factor (> 0).
   */
  void sensorUpdate(const Vector2f& position, unsigned timestamp, float perceptValidity, float measurementNoiseFactor = 1.f);


  /**
   * \brief Correction
   *
   * This method performs the correction step of the kalman filter when both
   * position and velocity are measured (used for remote modeling).
   * \param [in] position The measured position (in mm).
   * \param [in] velocity The measured position (in mm/s).
   * \param [in] timestamp The timestamp when the position was measured (in ms).
   *                       The timestamp of the newest correction is always
   *                       saved to \c timeWhenLastSeen.
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] measurementNoiseFactor Scale measurement noise by
   *                                    multiplication with this factor (> 0).
   */
  void sensorUpdate(const Vector2f& position, const Vector2f& velocity, unsigned timestamp, float perceptValidity, float measurementNoiseFactor = 1.f);


  //MARK: Validity related methods

  /**
   * Return the <tt>percepts per second</tt> value tracked over
   * the last second counted from the last prediction or correction.
   * \return Percepts per second.
   */
  float perceptsPerSecond() const;
  /**
   * Return the <tt>percepts per second</tt> value tracked over
   * the last second counted from \c currentTimestamp.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \return Percepts per second.
   */
  float perceptsPerSecond(unsigned currentTimestamp);
  /**
   * Return the mean percept validity tracked over the last second
   * counted from the last prediction or correction.
   * \return Mean percept validity.
   */
  float meanPerceptValidity() const;
  /**
   * Return the mean percept validity tracked over the last second
   * counted from \c currentTimestamp.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \return Mean percept validity.
   */
  float meanPerceptValidity(unsigned currentTimestamp);

  /**
   * Update the validity based on the percepts per second.
   * \see perceptsPerSecond()
   * \param [in] maxPerceptsPerSecond Number of percepts per second which result
   *                                  in a validity of 1.0.
   * \param [in] weightOfPreviousValidity Consider not only the pps but also the
   *                                      old validity if it was higher than the
   *                                      new one (based on pps). The weight of
   *                                      the new validity is set to 1.0.
   */
  void updateValidity(float maxPerceptsPerSecond, float weightOfPreviousValidity);

  /**
   * Update the maxMeasurementNoise
   * \param [in] maxMeasurementNoise Maximum measurement noise
   */
  void updateMaxMeasurementNoise(double maxMeasurementNoise);

  /**
   * Scale the Kalman filter covariance matrix by the given factor.
   * \param [in] factor The covariance matrix is multiplied by this factor.
   */
  void increaseFilterCovariance(double factor);

  /**
   * Scale the Kalman filter covariance matrix by the given factors.
   * \param [in] positionFactor The 2x2 position part of the covariance matrix
   *                            is multiplied by this factor.
   * \param [in] velocityFactor The 2x2 velocity part of the covariance matrix
   *                            is multiplied by this factor.
   */
  void increaseFilterCovariance(double positionFactor, double velocityFactor);

  /**
   * Return the number of sensor updates done on this hypothesis. This corresponds
   * to the number of matched percepts.
   * \return The number of sensor updates.
   */
  inline unsigned int numberOfSensorUpdates() const { return m_numberOfSensorUpdates; }

  /**
   * Increase the \c numberOfSensorUpdates by the given amount.
   * \param [in] add Increase the \c numberOfSensorUpdates by this value.
   */
  void addNumberOfSensorUpdates(unsigned int add) { m_numberOfSensorUpdates += add; }


  //MARK: Helper methods

  /**
   * This method is called when another hypothesis (\c source) gets merged into
   * this one. This method adds meta information from \c source into \c *this
   * (e.g. the history of the <tt>percepts per second</tt> tracking and the number
   * of sensor updates. It does not merge the Kalman filters.
   * \param [in] source The hypothesis which gets merged into \c *this.
   */
  virtual void merge(const KalmanPositionHypothesis& source);

protected:
  /**
   * This method is "merges" this Kalman filter with the other one.
   * \param [in] source The kalman filter which gets merged into \c *this.
   */
  void mergeKalmanPosition(const KalmanPositionTracking2D<double>& other);

public:
  /// A kalman filter used for filtering this ball hypothesis.
  KalmanPositionTracking2D<double> kalman;

  /// The unfiltered last position which was given to a \c sensorUpdate method.
  Vector2f lastPerception;

  /// The validity that this ball hypothesis describes the actual ball position.
  /// Value in range [0,1].
  float validity;

  /// The timestamp (in ms), when the ball was seen for the last time.
  unsigned timeWhenLastSeen;

  /// Factor to multiply the merge criteria with (make it less sensitive).
  float mergeFactor;

private:
  /// The timestamp (in ms) of the last motion update.
  unsigned m_lastMotionUpdateTimeStamp;

  /// Stores the number of sensor updates done on this hypothesis. This corresponds
  /// to the number of matched ball percepts.
  unsigned int m_numberOfSensorUpdates;

  /// Stores for the last 30 frames (1000 milliseconds) how many sensor updates were run.
  PerceptsPerSecond m_perceptsPerSecond;

public:
  /** Streaming */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(kalman);
    STREAM(validity);
    STREAM(timeWhenLastSeen);
    STREAM(m_numberOfSensorUpdates);
    STREAM_REGISTER_FINISH;
  }
};
