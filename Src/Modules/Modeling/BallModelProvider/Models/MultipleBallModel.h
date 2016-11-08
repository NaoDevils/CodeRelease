/**
 * \file MultipleBallModel.h
 * 
 * Declaration of class MultipleBallModel.
 * This class represents a set of multiple ball hypotheses and provides 
 * update methods. See class \c BallHypothesis.
 * 
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "BallHypothesis.h"

#include "Representations/Configuration/FieldDimensions.h" // field dimensions

/**
 * \class MultipleBallModel
 * 
 * Stores a set of multiple ball hypotheses. Each hypothesis stores position and 
 * velocity of the ball using a kalman filter. The hypotheses also have a 
 * validity which is used to select the best one as BallModel.
 * The local ball model stores ball positions and velocities in relative robot
 * coordinates.
 * \see BallHypothesis, BallModel
 */
class MultipleBallModel : public Streamable
{
public:

  /** 
   * Default constructor.
   */
  MultipleBallModel() : m_bestHypothesisIndex(-1) {}
  /** 
   * Destructor.
   */
  ~MultipleBallModel() {}
  

  // Kalman filter methods.

  /**
   * \brief Removes the odometry offset from the kalman filter state of each
   *        ball hypothesis.
   *
   * This method removes the influence of the robots odometry offset on the
   * kalman filter of each ball hypothesis.
   * This method is only needed for the local ball model.
   * \param odometryOffset The offset of the odometry data since last iteration.
   */
  void removeOdometry(const Pose2f& odometryOffset);

  /**
   * \brief Prediction
   * 
   * This method performs the prediction step of each ball hypothesis.
   * 
   * While the ball rolls, the velocity is reduced due to friction. This 
   * friction is approximated by subtracting the ball friction each 100 ms 
   * from the velocity.
   * 
   * \param [in] timeOffset The time in seconds since the last prediction.
   * \param [in] ballFriction The ball friction (negative force) (in m/s^2).
   */
  void motionUpdate(float timeOffset, float ballFriction);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best 
   * to the given measurement. All other hypotheses are not changed, so their 
   * validity decreases because of the motion update. If no fitting hypothesis 
   * exists (see minDistanceForNewHypothesis), a new one one is created at the 
   * measured position.
   * \param [in] measuredBallPosition The measurement of the ball position.
   * \param [in] measuredDistanceToBall Disrance on field between robot and ball 
   *                                    perception in mm.
   * \param [in] timestamp The (current) timestamp when the ball position was 
   *                       measured. This timestamp will be saved to the 
   *                       hypothesis which is updated by the measurement.
   * \param [in] ballPerceptValidity The validity received from the \c BallPercept
   *                                 in range [0,1]. This value is multiplied to 
   *                                 the influence parameters before validity update.
   * \param [in] minDistanceForNewHypothesis If \c measuredBallPosition has at 
   *                                         least this distance to all existing 
   *                                         ball hypotheses, a new hypothesis is 
   *                                         created with position from ball percept.
   * \param [in] initialValidityForNewHypothesis Set this validity for a new hypothesis.
   * \param [in] kalmanMatrices Fix matrices of the kalman filter for a new hypothesis.
   * \param [in] playernumber Optional: The number of the teammate to add.
   * \param [in] teammate Optional: Pointer to the teammate to add.
   */
  void sensorUpdate(
    const Vector2f& measuredBallPosition,
    float measuredDistanceToBall,
    unsigned timestamp,
    float ballPerceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices,
    int playernumber = -1, const BallHypothesis::BallHypothesisTeammateInfo* teammate = nullptr);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best
   * to the given measurement. All other hypotheses are not changed, so their
   * validity decreases because of the motion update. If no fitting hypothesis
   * exists (see minDistanceForNewHypothesis), a new one one is created at the
   * measured position.
   * \param [in] measuredBallPosition The measurement of the ball position.
   * \param [in] measuredDistanceToBall Disrance on field between robot and ball 
   *                                    perception in mm.
   * \param [in] measuredBallVelocity A pointer to the measurement of the ball velocity
   *                                  or a \c nullptr the velocity is not measured.
   * \param [in] timestamp The (current) timestamp when the ball position was
   *                       measured. This timestamp will be saved to the
   *                       hypothesis which is updated by the measurement.
   * \param [in] ballPerceptValidity The validity received from the \c BallPercept
   *                                 in range [0,1]. This value is multiplied to 
   *                                 the influence parameters before validity update.
   * \param [in] minDistanceForNewHypothesis If \c measuredBallPosition has at
   *                                         least this distance to all existing
   *                                         ball hypotheses, a new hypothesis is
   *                                         created with position from ball percept.
   * \param [in] initialValidityForNewHypothesis Set this validity for a new hypothesis.
   * \param [in] kalmanMatrices Fix matrices of the kalman filter for a new hypothesis.
   * \param [in] playernumber Optional: The number of the teammate to add.
   * \param [in] teammate Optional: Pointer to the teammate to add.
   */
  void sensorUpdate(
    const Vector2f& measuredBallPosition,
    float measuredDistanceToBall,
    const Vector2f* measuredBallVelocity,
    unsigned timestamp,
    float ballPerceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices,
    int playernumber = -1, const BallHypothesis::BallHypothesisTeammateInfo* teammate = nullptr);


  // Validity methods.
  
  /**
   * Initializes a new frame on all hypotheses. This creates a new frame in the 
   * ring buffer for computing the percepts/s.
   */
  void initializeFrameForValidity();
  /**
   * Updates the validity of all hypotheses based on the percepts per second.
   * \param [in] maxPerceptsPerSecond
   * \param [in] weightOfPreviousValidity
   */
  void updateValidity(
    unsigned int maxPerceptsPerSecond,
    float goodValidityThreshold,
    float weightOfPreviousValidity,
    float weightOfPreviousValidity_goodHypotheses);


  // Help methods.

  /**
   * Searches for the nearest hypothesis to the seen ball position in the 
   * set of ball hypotheses (\c m_ballHypotheses) and returns a pointer to it.
   * If the hypotheses set is empty, this method returns nullptr and distance
   * is set to -1.
   * \param [in] measuredBallPosition The seen ball position relative to the robot.
   * \param [out] distance The distance from the perception to the nearest hypothesis.
   * \return A pointer to the nearest ball hypothesis.
   */
  BallHypothesis* findNearestHypothesis(
    const Vector2f& measuredBallPosition, float& distance);

  /**
   * Returns a pointer to the best hypothesis found by method \c updateBestHypothesis(). 
   * If there is no ball hypothesis, a nullptr is returned.
   * \return A pointer to the ball hypothesis with the highest validity.
   */
  const BallHypothesis* bestHypothesis() const;

  /**
   * Searches for the ball hypothesis with the highest validity and saves it 
   * for invocations of method \c bestHypothesis(). A hypothesis must had at least two 
   * sensor updates to be accepted as best hypothesis (less likely a false positive).
   * \param [in] minValidity Only hypotheses with at least this validity can become the 
   *                         best hypothesis. If all hypotheses are below this threshold 
   *                         the last best hypothesis is retained.
   * \param [in] minNumberOfSensorUpdates Only hypotheses with had at least this amount
   *                         of sensor updates can become the best hypothesis.
   * \param [in] decreaseValidityOnChangingBestHypothesis If the best hypothesis changes, 
   *                                                      the last ones validity is decreased
   *                                                      by this value.
   */
  void updateBestHypothesis(float minValidity, size_t minNumberOfSensorUpdates,
                            float decreaseValidityOnChangingBestHypothesis);

  /**
   * This method removes all ball hypotheses with a too small validity (not the 
   * best one!). It also removes hypotheses which are too close to the best one.
   * \param [in] theFieldDimensions The field dimensions used for checking, if 
   *                                the ball is outside of the field.
   * \param [in] theRobotPose The current pose of the robot used to calculate 
   *                          absolute positions of the hypotheses on field.
   * \param [in] validityThreshold Remove hypotheses with less than this validity.
   * \param [in] minDistanceForSeparateHypotheses Hypotheses which are closer 
   *                                              than this distance to the best 
   *                                              hypothesis (distance in mm)
   *                                              and ...
   * \param [in] minAngleForSeparateHypotheses    ... which velocities are less 
   *                                              than this distance different 
   *                                              from the best ones will be 
   *                                              removed (angle in radian).
   */
  void cleanUpHypotheses(const FieldDimensions& theFieldDimensions, const RobotPose& theRobotPose,
    float validityThreshold, float minDistanceForSeparateHypotheses, float minAngleForSeparateHypotheses);
  
  /**
   * Removes all hypotheses in this \c MultipleBallModel.
   */
  void clear();
  
  /**
   * Add a copy of the given hypothesis to this \c MultipleBallModel.
   * This should be treated with caution! Only add a hypothesis if you are sure 
   * you know what you are doing.
   * \param [in] newHypothesis The hypothesis to add.
   */
  void addHypothesis(const BallHypothesis& newHypothesis);
  
  /**
   * Returns the number of ball hypotheses which are currently stored by this 
   * \c MultipleBallModel. This make information about the hypotheses set 
   * available from outside of this class.
   * \return Number of ball hypotheses.
   */
  std::size_t numberOfHypotheses() const { return m_ballHypotheses.size(); }


  // Debug methods.

  /**
   * Draws all ball hypotheses as local ball model hypotheses.
   */
  virtual void draw() const;


  // ---------- Streaming ----------

  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(m_ballHypotheses);
    STREAM_REGISTER_FINISH;
  }


protected:
  /**
   * Stores a set of ball hypotheses.
   */
  std::vector<BallHypothesis> m_ballHypotheses;
  /**
   * Stores the index of the best hypothesis.
   * This is updated by the method \c updateBestHypothesis().
   */
  size_t m_bestHypothesisIndex;
};


/**
 * \class RemoteMultipleBallModel
 *
 * This class is a \c MultipleBallModel for modeling ball hypotheses 
 * from teammates. Functional it is identical to the \c MultipleBallModel,
 * but it has another drawing method for distinguishing between local and 
 * remote debug drawings.
 * Unlike the local ball model the remote ball model stores ball positions and 
 * velocities in absolute field coordinates.
 * \see MultipleBallModel
 */
class RemoteMultipleBallModel : public MultipleBallModel
{
public:
  // Debug methods.

  /**
   * Draws all ball hypotheses as remote ball model hypotheses.
   */
  void draw() const;
};
