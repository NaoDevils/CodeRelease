/**
 * \file BallHypothesis.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * Declaration of class BallHypothesis.
 * This class represents a ball state used by the \c BallModelProvider.
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/KalmanMultiDimensional.h"
#include "Tools/RingBufferWithSum.h"

#include <map>

/**
 * \class BallHypothesis
 * 
 * Stores one hypothesis of the current ball state including position, 
 * velocity and the used filter properties.
 */
class BallHypothesis : public Streamable
{
public:

  /** 
   * Default constructor.
   */
  BallHypothesis() : validity(0.f), timeWhenLastSeen(0), m_numberOfSensorUpdates(0)
  {
    Vector2f zeroVector;
    zeroVector << 0.f, 0.f;
    initialize(zeroVector, &zeroVector);
  }

  /**
   * \brief Constructor.
   * 
   * Creates a new BallHypothesis object with a given initial ball position.
   * \param [in] position The initial ball position (in mm).
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The current time.
   * \param [in] kalmanMatrices Fix matrices of the kalman filter.
   */
  BallHypothesis(const Vector2f& position,
                 float initialValidity,
                 unsigned timestamp,
	               const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices)
    : validity(initialValidity), timeWhenLastSeen(timestamp), m_numberOfSensorUpdates(1)
  {
    // Create initial velocity vector.
    Vector2f velocity;
    velocity << 0.f, 0.f; // Velocity is not known at initialization.
    initialize(position, &velocity, kalmanMatrices);
    // Initialize ring buffer.
    perceptsPerSecondComputation_newFrame();
    perceptsPerSecondComputation_addPercept();
  }
  
  /**
   * \brief Constructor.
   * 
   * Creates a new BallHypothesis object with a given initial ball position and 
   * an optional initial ball velocity.
   * \param [in] position The initial ball position (in mm).
   * \param [in] velocity Pointer to the initial ball velocity (in mm/s) 
   *                      or \c nullptr if velocity is unknown.
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The current time.
   * \param [in] kalmanMatrices Fix matrices of the kalman filter.
   */
  BallHypothesis(const Vector2f& position,
                 const Vector2f* velocity,
                 float initialValidity,
                 unsigned timestamp,
	               const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices)
    : validity(initialValidity), timeWhenLastSeen(timestamp), m_numberOfSensorUpdates(1)
  {
    initialize(position, velocity, kalmanMatrices);
    // Initialize ring buffer.
    perceptsPerSecondComputation_newFrame();
    perceptsPerSecondComputation_addPercept();
  }

  /** 
   * Destructor.
   */
  ~BallHypothesis() {}

  /**
   * Initializes this ball hypothesis and the underlying kalman filter with a 
   * given initial ball position, an optional initial ball velocity and default 
   * kalman matrices.
   * \param [in] position The initial ball position (in mm).
   * \param [in] velocity Pointer to the initial ball velocity (in mm/s) 
   *                      or \c nullptr if velocity is unknown.
   */
  void initialize(const Vector2f& position, const Vector2f* velocity);

  /**
   * Initializes this ball hypothesis and the underlying kalman filter with a
   * given initial ball position and an optional initial ball velocity.
   * \param [in] position The initial ball position (in mm).
   * \param [in] velocity Pointer to the initial ball velocity (in mm/s) 
   *                      or \c nullptr if velocity is unknown.
   * \param [in] kalmanMatrices Fix matrices of the kalman filter.
   */
  void initialize(const Vector2f& position, const Vector2f* velocity,
	  const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices);


  /**
   * \brief Removes the odometry offset from the kalman filter state.
   *
   * This method removes the influence of the robots odometry offset on the 
   * kalman state and covariance.
   * \param [in] reverseOdometry The transformation which should be applied to the
   *                             kalman filter state to compensate the odometry offset.
   */
  void removeOdometry(const KalmanPositionTracking2D<double>::KalmanStateTransformation& reverseOdometry);

  /**
   * \brief Prediction
   * 
   * This method performs the prediction step of the kalman filter.
   * 
   * While the ball rolls, the velocity is reduced due to friction. This 
   * friction is approximated by subtracting the ball friction each second 
   * from the velocity.
   * \param [in] timeOffset The time in seconds since the last prediction.
   * \param [in] ballFriction The ball friction (negative force) (in m/s^2).
   */
  void motionUpdate(float timeOffset, float ballFriction);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the kalman filter.
   * \param [in] position The measured ball position (in mm).
   * \param [in] timestamp The (current) timestamp when the ball position was
   *                       measured. This timestamp will be saved to the
   *                       hypothesis which is updated by the measurement.
   * \param [in] measurementNoiseFactor Increase measurement noise by multiplication 
   *                                    with this factor (> 0).
   */
  void sensorUpdate(const Vector2f& position, unsigned timestamp, float measurementNoiseFactor = 1.f);


  /**
   * \brief Correction
   *
   * This method performs the correction step of the kalman filter when both 
   * position and velocity are measured (as used for remote ball model).
   * \param [in] position The measured ball position (in mm).
   * \param [in] velocity The measured ball position (in mm/s).
   * \param [in] timestamp The (current) timestamp when the ball position was
   *                       measured. This timestamp will be saved to the
   *                       hypothesis which is updated by the measurement.
   * \param [in] measurementNoiseFactor Increase measurement noise by multiplication 
   *                                    with this factor (> 0).
   */
  void sensorUpdate(const Vector2f& position, const Vector2f& velocity,
                    unsigned timestamp, float measurementNoiseFactor = 1.f);

  /**
   * Add a new frame without percept to the ring buffer (value 0 of first element).
   */
  void perceptsPerSecondComputation_newFrame();
  /**
   * Adds a percept to the current frame (first element) in the ring buffer.
   */
  void perceptsPerSecondComputation_addPercept();
  /**
   * \return the number of percepts in the last second.
   */
  unsigned int perceptsPerSecond() const;
  
  /**
   * Updates the validity based on the percepts per second.
   * \param [in] maxPerceptsPerSecond
   * \param [in] weightOfPreviousValidity
   */
  void updateValidity(unsigned int maxPerceptsPerSecond, float weightOfPreviousValidity);
  
  /**
   * Updates the given \c BallState to be used in \c BallModel::estimate. The 
   * \c BallState is filled with the contents of this \c BallHypothesis which 
   * are extracted from the kalman filter state.
   * \param [out] ballState The \c BallState which should be updated
   */
  void updateEstimatedBallState(BallState& ballState) const;

  /**
   * Returnsthe number of sensor updates done on this hypothesis. This corresponds 
   * to the number of matched ball percepts.
   * \return the number of sensor updates.
   */
  unsigned int numberOfSensorUpdates() const { return m_numberOfSensorUpdates; }
  
  /**
   * Increase the \c numberOfSensorUpdates by the given amount.
   * \param [in] add Increase the \c numberOfSensorUpdates by this value.
   */
  void addNumberOfSensorUpdates(unsigned int add) { m_numberOfSensorUpdates += add; }
  

  // Labeling methods.

  /**
   * \struct BallHypothesisTeammateInfo
   * 
   * One or multiple teammates can be assigned to a ball hypothesis. This contains 
   * the validity of the teammates ball model.
   */
  struct BallHypothesisTeammateInfo
  {
    BallHypothesisTeammateInfo() : validity(0.f), timeWhenLastUpdated(0) {}
    BallHypothesisTeammateInfo(float validity_, unsigned timeWhenLastUpdated_)
      : validity(validity_), timeWhenLastUpdated(timeWhenLastUpdated_) {}
    float validity;
    unsigned timeWhenLastUpdated;
  };

  /**
   * Add the given teammate to this ball hypothesis. If the \c playerNumber is 
   * already in use, the teammate is updated if the timestamp is newer.
   * \param [in] playerNumber The number of the teammate to be assigned.
   * \param [in] newTeammate The teammate info to be assigned.
   */
  void addTeammateInfo(int playerNumber, const BallHypothesisTeammateInfo& newTeammate);

  /**
   * Add the given set of teammates to this ball hypothesis. For any teammate in 
   * \c newTeammates the method \c addTeammateInfo(..) is called.
   * \param [in] newTeammates The set of teammates to be assigned.
   */
  void addTeammates(const std::map<int, BallHypothesis::BallHypothesisTeammateInfo>& newTeammates);

  /**
   * Returns the map of teammates assigned to this ball hypothesis indexed by the playernumber.
   * \return The map of assigned teammates.
   */
  const std::map<int, BallHypothesis::BallHypothesisTeammateInfo>& teammates() const { return m_teammates; }

  /**
   * Returns the last timestamp when the player with given number has seen the ball and its 
   * percept has updated this hypothesis. Return 0 if \c playerNumber never updated this hypothesis.
   * \param [in] playerNumber The number of the teammate to be assigned.
   * \return time when \c playerNumber last updated this hypothesis with its ball percept.
   */
  unsigned timeWhenUpdatedByTeammate(int playerNumber) const;

  /**
   * Returns the player numbers of all teammates which updated this ball hypothesis 
   * as string (e.g. "1,2,3").
   * \return The set of assigned player numbers.
   * \see m_teammates
   */
  std::string teammatesString() const;

public:
  /// A kalman filter used for filtering this ball hypothesis.
  KalmanPositionTracking2D<double> kalman;

  /// The validity that this ball hypothesis describes the actual ball position.
  /// Value in range [0,1].
  float validity;

  /// The timestamp, when the ball was seen for the last time.
  unsigned timeWhenLastSeen;

private:
  /// Stores various information from teammates whos ball models are merged into this hypothesis.
  /// The teammates are indexed by the player number.
  /// See struct \c BallHypothesisTeammateInfo.
  std::map<int, BallHypothesis::BallHypothesisTeammateInfo> m_teammates;

  /// Stores the number of sensor updates done on this hypothesis. This corresponds 
  /// to the number of matched ball percepts.
  unsigned int m_numberOfSensorUpdates;
  
  /// Stores for the last 30 frames (1 second) whether a sensor update was run or
  /// not. 0 ^= no sensor update. 1 ^= sensor update (percept).
  RingBufferWithSum<unsigned short, 30> m_perceptsPerSecond;

public:
  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(kalman);
    STREAM(validity);
    STREAM(timeWhenLastSeen);
    //STREAM(m_labels); // doesn't work correctly
    STREAM(m_numberOfSensorUpdates);
    //STREAM(m_perceptsPerSecond); // doesn't compile
    STREAM_REGISTER_FINISH;
  }
};
