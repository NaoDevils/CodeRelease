/**
 * \file MultiKalmanModel.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * 
 * Declaration of class MultiKalmanModel.
 * This class represents a set of multiple \c KalmanPositionHypotheses and provides 
 * update methods.
 */

#pragma once

#include <type_traits>
#include <vector>
#include <memory>

#include "Tools/Streams/AutoStreamable.h"
#include "KalmanPositionHypothesis.h"

#include "Representations/Configuration/FieldDimensions.h" // field dimensions
#include "Representations/Modeling/RobotPose.h"

/**
 * \class MultiKalmanModel
 * 
 * This class stores a set of multiple \c KalmanPositionHypotheses. Each
 * hypothesis represents position and velocity of an object (e.g. ball, obstacle)
 * filtered by a Kalman filter.
 * The hypotheses also have a validity which can be used to select the best one
 * as final model. Thus the \c MultiKalmanModel can deal with jumping percepts 
 * by selecting the most valid percepts via the best hypothesis.
 *
 * Local models stores positions and velocities in relative robot coordinates
 * while remote and team models store them in global field coordinates.
 * \see KalmanPositionHypothesis, RemoteMultiKalmanModel
 *
 * \tparam hypothesis_t The type of each hypothesis of this model.
 *                      It must be derived from \c KalmanPositionHypothesis.
 * \tparam towardsOneModel Defines whether the MultiKalmanModel converges to
 *                       a single model (e.g. ball) not (e.g. robot map)
 */
template <typename hypothesis_t = KalmanPositionHypothesis, bool towardsOneModel = true>
STREAMABLE(MultiKalmanModel,
  // Check at compile-time that hypothesis_t is derived from class KalmanPositionHypothesis.
  static_assert(std::is_base_of<KalmanPositionHypothesis, hypothesis_t>::value,
                "hypothesis_t not derived from class KalmanPositionHypothesis");
  
  /**
   * The type of hypothesis stored in the kalman model
   */
  using HypothesisType = hypothesis_t;

  /**
   * Default constructor creates a new and empty \c MultiKalmanModel object.
   * The default perceptDuration is 1000ms.
   */
  MultiKalmanModel() {}

  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  MultiKalmanModel(unsigned perceptDuration) : m_perceptDuration (perceptDuration) {}
  
  //MARK: Kalman filter related methods

  /**
   * \brief Removes an odometry offset from the kalman filter state of each hypothesis.
   *
   * This method removes the influence of the robots odometry offset on the
   * kalman filter from each hypothesis.
   * This method is only needed for local models (relative robot coordinates).
   * \param odometryOffset The offset of the odometry data since last iteration.
   */
  void removeOdometry(const Pose2f& odometryOffset);

  /**
   * \brief Prediction
   *
   * This method performs the prediction step of each hypothesis.
   *
   * While moving the velocity can be reduced due to friction. The friction is
   * approximated by subtracting the friction each second from the velocity.
   * This model is appropriate for a rolling ball.
   * \param [in] currentTimestamp The current timestamp (in ms).
   * \param [in] friction The deceleration due to friction (negative value; in m/s^2).
   */
  void motionUpdate(unsigned currentTimestamp, float friction);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best 
   * to the given measurement. All other hypotheses are not changed, so their 
   * validity decreases because of the motion update. If no fitting hypothesis 
   * exists (see minDistanceForNewHypothesis), a new one one is created at the 
   * measured position.
   * \param [in] measuredPosition The measured position.
   * \param [in] measuredDistance Distance on field between robot and 
   *                              \c measuredPosition (in mm).
   * \param [in] timestamp The timestamp of the percept. This timestamp will be
   *                       saved to the hypothesis which is updated by the measurement.
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] minDistanceForNewHypothesis If \c measuredPosition has at least this distance
   *                                         to all existing hypotheses, a new hypothesis is 
   *                                         created with the percept position.
   * \param [in] initialValidityForNewHypothesis Set this validity for a new hypothesis.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter for a new hypothesis.
   * \return The updated or new created hypothesis.
   */
  hypothesis_t* sensorUpdate(
    const Vector2f& measuredPosition,
    float measuredDistance,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best
   * to the given measurement. All other hypotheses are not changed, so their
   * validity decreases because of the motion update. If no fitting hypothesis
   * exists (see minDistanceForNewHypothesis), a new one one is created at the
   * measured position.
   * \param [in] measuredPosition The measured position.
   * \param [in] measuredVelocity Optional: A pointer to the measurement of the 
   *                              velocity or \c nullptr if the velocity is not 
   *                              measured.
   * \param [in] measuredDistance Distance on field between robot and
   *                              \c measuredPosition (in mm).
   * \param [in] timestamp The timestamp of the percept. This timestamp will be
   *                       saved to the hypothesis which is updated by the measurement.
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] minDistanceForNewHypothesis If \c measuredPosition has at least this distance
   *                                         to all existing hypotheses, a new hypothesis is
   *                                         created with the percept position.
   * \param [in] initialValidityForNewHypothesis Set this validity for a new hypothesis.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter for a new hypothesis.
   * \return The updated or new created hypothesis.
   */
  hypothesis_t* sensorUpdate(
    const Vector2f& measuredPosition,
    float measuredDistance,
    const Vector2f* measuredVelocity,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices);


  //MARK: Validity methods
  
  /**
   * Update the validity of all hypotheses based on the number of <tt>percepts
   * per second</tt>. Hypotheses with a high validity (good hypotheses) are handled
   * differently.
   * \param [in] maxPerceptsPerSecond Number of percepts per second which result
   *                                  in a validity of 1.0.
   * \param [in] goodValidityThreshold An old validity above this threshold indicates
   *                                   a good hypothesis. See parameter
   *                                   \c weightOfPreviousValidity_goodHypotheses.
   * \param [in] weightOfPreviousValidity Consider not only the pps but also the
   *                                      old validity if it was higher than the
   *                                      new one (based on pps). The weight of
   *                                      the new validity is set to 1.0.
   * \param [in] weightOfPreviousValidity_goodHypotheses Use this value instead of
   *                                      \c weightOfPreviousValidity for good 
   *                                      hypotheses.
   */
  void updateValidity(
    float maxPerceptsPerSecond,
    float goodValidityThreshold,
    float weightOfPreviousValidity,
    float weightOfPreviousValidity_goodHypotheses);


  //MARK: Helper methods
  
  /**
   * Search for the hypothesis nearest to the percept in the set of all hypotheses
   * and return a pointer to it. If the hypotheses set is empty this method returns
   * a \c nullptr and \c distance is set to -1.
   * \param [in] measuredPosition Position of the percept (in mm).
   * \param [out] distance The distance from the percept to the nearest hypothesis.
   * \return A pointer to the nearest hypothesis.
   */
  hypothesis_t* findNearestHypothesis(
    const Vector2f& measuredPosition, float& distance);

  /**
   * Return a pointer to the best hypothesis.
   * If there is no hypothesis at all a \c nullptr is returned.
   * \return A pointer to the hypothesis with the highest validity.
   */
  const hypothesis_t* bestHypothesis();
  
  /**
   * Set parameters for internal calls of \c updateBestHypothesis().
   * \param [in] minValidityForChangingBestHypothesis Only hypotheses with at least this
   *                                                  validity can become the best hypothesis.
   *                                                  If all hypotheses are below this threshold
   *                                                  the last best hypothesis is retained.
   * \param [in] minNumberOfSensorUpdatesForBestHypothesis Only hypotheses with had at least
   *                                                       this amount of sensor updates can
   *                                                       become the best hypothesis.
   * \param [in] decreaseValidityOnChangingBestHypothesis If the best hypothesis changes,
   *                                                      the last ones validity is decreased
   *                                                      by this value.
   */
  void setParametersForUpdateBestHypothesis(float minValidityForChangingBestHypothesis,
                                            std::size_t minNumberOfSensorUpdatesForBestHypothesis,
                                            float decreaseValidityOnChangingBestHypothesis);
  
  /**
   * Reset the index of the best hypothesis to an invalid value. It must be 
   * recalculated at the next call of bestHypothesis().
   */
  void resetBestHypothesisIndex();
  
private:
  /**
   * Search for the hypothesis with the highest validity and saves it for later
   * invocations of the method \c bestHypothesis(). A hypothesis must had at least
   * \c minNumberOfSensorUpdates previous sensor updates to be accepted as best
   * hypothesis (less likely a false positive).
   */
  void updateBestHypothesis();
  
  /**
   * Checks if the index of teh best hypothesis was already computed (in the current 
   * frame). If not \c updateBestHypothesis() is called before returning the index.
   * \return The index of the best hypothesis or <tt>static_cast<size_t>(-1)</tt> if no
   *         hypothesis exists.
   */
  std::size_t updateBestHypothesisIndexIfNecessary();

  void removeHypothesis(size_t index);
  
public:
  /**
   * Increase both part of the Kalman filter covariance by the given
   * factor. This can be used to improve the kalman filter behavior on predictable
   * sudden movement changes.
   * \param [in] positionFactor The position part of the covariance matrix is multiplied
   *                    by this factor.
   * \param [in] velocityFactor The velocity part of the covariance matrix is multiplied 
   *                    by this factor.
   * \param [in] onlyBestHypothesis If \c true  only the covariance matrix of
   *                                the best hypothesis is changed.
   *                                Otherwise all hypotheses are changed.
   */
  void increaseUncertainty(double positionFactor, double velocityFactor, bool onlyBestHypothesis);
  
  /**
   * This method removes all hypotheses which are outside of the field (with
   * \c fieldBorderThreshold).
   * \param [in] theFieldDimensions The field dimensions used for checking, if
   *                                the hypothesis is outside of the field.
   * \param [in] theRobotPose The current pose of the robot used to calculate
   *                          absolute positions of the hypotheses on the field.
   * \param [in] fieldBorderThreshold The field (lines) is enlarged by this size
   *                                  (in millimeter) for removing outlying hypotheses.
   *                                  See \c cleanUpOutsideField.
   */
  void cleanUpHypothesesOutsideField(const FieldDimensions& theFieldDimensions,
                                     const RobotPose& theRobotPose,
                                     float fieldBorderThreshold);
  
  /**
   * This method removes all hypotheses with a too small validity (not the
   * best one!).
   * \param [in] validityThreshold Remove hypotheses with less than this validity.
   * \param [in] saveBestHypothesis If \c true the best hypothesis will not be removed.
   */
  void cleanUpHypothesesLowValidity(float validityThreshold, bool saveBestHypothesis);
  
  /**
   * This method removes hypotheses which are too close to the best one.
   * \param [in] minDistanceForSeparateHypotheses Hypotheses which are closer
   *                                              than this distance to the best
   *                                              hypothesis (distance in mm)
   *                                              and ...
   * \param [in] minAngleForSeparateHypotheses    ... which has a velocity with
   *                                              less than this angular distance
   *                                              from the best ones will be
   *                                              removed (angle in radian).
   */
  void cleanUpHypothesesSimilarToBestOne(float minDistanceForSeparateHypotheses,
                                         float minAngleForSeparateHypotheses);

  /**
   * Remove all hypotheses.
   */
  void clear();
  
  /**
   * Add a copy of the given hypothesis to this \c MultiKalmanModel.
   * \warning This should be treated with caution! Only add a hypothesis if
   *          you are sure you know what you are doing.
   * \param [in] newHypothesis The hypothesis to add.
   */
  void addHypothesis(const hypothesis_t& newHypothesis);
  
  /**
   * Add the given hypothesis to this \c MultiKalmanModel.
   * \warning This should be treated with caution! Only add a hypothesis if
   *          you are sure you know what you are doing.
   * \param [in] newHypothesis The hypothesis to add.
   */
  void addHypothesis(hypothesis_t&& newHypothesis);
  
  /**
   * Return the number of hypotheses which are currently stored by this
   * \c MultiKalmanModel. This make information about the hypotheses set 
   * available from outside of this class.
   * \return Number of hypotheses.
   */
  std::size_t size() const { return m_hypotheses.size(); }
  
  /**
   * Return a pointer to the hypothesis with the given index.
   * \param [in] i Index of the requested hypothesis.
   * \return A pointer to the hypothesis with index \c i.
   */
  const hypothesis_t& operator[](size_t i) const { return m_hypotheses[i]; };
  
  /**
   * Return a pointer to the hypothesis with the given index.
   * \param [in] i Index of the requested hypothesis.
   * \return A pointer to the hypothesis with index \c i.
   */
  hypothesis_t& operator[](size_t i) { return m_hypotheses[i]; };

  /**
   * Returns a reference to the last hypothesis (the latest added).
   * \return A reference to the last hypothesis (the latest added).
   */
  const hypothesis_t& back() const { return m_hypotheses.back(); }

  /**
   * Returns a reference to the last hypothesis (the latest added).
   * \return A reference to the last hypothesis (the latest added).
   */
  hypothesis_t& back() { return m_hypotheses.back(); }
  
  /**
   * States whether this model use relative robot coordinates or global field 
   * coordinates. Local models use always relative robot coordinates while remote
   * models always use global field coordinates.
   *
   * This method is pure virtual and must be overridden by derived classes!
   *
   * \return \c true if the hypotheses within this model are represented in 
   *         relative robot coordinates,
   *         \c false otherwise (global field coordinates).
   */
  virtual bool usesRelativeCoordinates() const = 0;

  /**
   * Gets the duration percepts (in ms) get buffered for identifying the validity of a hypothesis.
   * \return The duration percepts (in ms) get buffered for identifying the validity of a hypothesis.
   */
  virtual unsigned getPerceptDuration() const { return m_perceptDuration; }

private:
  /// Stores the index of the best hypothesis.
  /// This is updated by the method \c updateBestHypothesis().
  mutable std::size_t m_bestHypothesisIndex = std::numeric_limits<size_t>::max();
  /// Stores the index of the best hypothesis from the last iteration.
  std::size_t m_lastBestHypothesisIndex = m_bestHypothesisIndex;
  
  /// Parameter for \c updateBestHypothesis().
  float minValidityForChangingBestHypothesis = 0.f;
  /// Parameter for \c updateBestHypothesis().
  std::size_t minNumberOfSensorUpdatesForBestHypothesis = 1;
  /// Parameter for \c updateBestHypothesis().
  float decreaseValidityOnChangingBestHypothesis = 0.f;
  /// The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
  unsigned m_perceptDuration = 1000;

  ,
  /// Stores a set of kalman hypotheses.
  (std::vector<hypothesis_t>) m_hypotheses
);

#include "MultiKalmanModel_impl.h"
