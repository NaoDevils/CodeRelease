/**
 * \file RemoteKalmanPositionHypothesis.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Declaration of class \c RemoteKalmanPositionHypothesis.
 * This class is an extension of class \c KalmanPositionHypothesis which
 * adds functionality for handling of remote models received from teammates.
 */

#pragma once

#include "KalmanPositionHypothesis.h"

/**
 * \class RemoteKalmanPositionHypothesis
 * 
 * Extension of class \c KalmanPositionHypothesis which adds functionality
 * for handling of remote models received from teammates.
 */
class RemoteKalmanPositionHypothesis : public KalmanPositionHypothesis
{
public:
  RemoteKalmanPositionHypothesis() : KalmanPositionHypothesis(){};

  /**
   * \brief Constructor with initialization.
   *
   * Creates a new \c RemoteKalmanPositionHypothesis object with a given initial
   * position and an optional initial velocity. Parameterizes the kalman filter
   * with noise matrices.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter.
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The timestamp of the initial position measurement (in ms).
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] position The initial position (in mm).
   * \param [in] perceptDuration The duration for the PerceptPerSecond buffer.
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown.
   */
  RemoteKalmanPositionHypothesis(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      float initialValidity,
      unsigned timestamp,
      float perceptValidity,
      const Vector2f& position,
      unsigned perceptDuration,
      const Vector2f& velocity = Vector2f::Zero())
      : KalmanPositionHypothesis(kalmanNoiseMatrices, initialValidity, timestamp, perceptValidity, position, perceptDuration, velocity){};

  /**
   * Destructor.
   */
  ~RemoteKalmanPositionHypothesis() override{};


  //MARK: Labeling methods

  /**
   * \struct TeammateInfo
   * 
   * The struct <tt>KalmanPositionHypothesis::TeammateInfo</tt> represents a teammate
   * that has contributed with its local model to this remote hypothesis.
   * One or multiple teammates can be assigned to a single hypothesis.
   * \see KalmanPositionHypothesis::addTeammateInfo(..), teammates()
   */
  struct TeammateInfo : Streamable
  {
    /**
     * \brief Default constructor
     *
     * Creates an invalid \c TeammateInfo object.
     */
    TeammateInfo() : validity(0.f), timeWhenLastUpdated(0) {}
    /**
     * Creates a new \c TeammateInfo object with validity and .
     * \param [in] validity The validity of the teammates local model (range: [0,1]).
     * \param [in] timeWhenLastUpdated Timestamp (in ms) when the teammate updated
     *                                 the hypothesis for the last time.
     */
    TeammateInfo(float validity, unsigned timeWhenLastUpdated) : validity(validity), timeWhenLastUpdated(timeWhenLastUpdated) {}

    virtual ~TeammateInfo() {}
    /// The validity of the teammates local model (range: [0,1]).
    float validity;
    /// Timestamp (in ms) when the teammate updated the hypothesis for the last time.
    unsigned timeWhenLastUpdated;

    /** Streaming */
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(validity);
      STREAM(timeWhenLastUpdated);
      STREAM_REGISTER_FINISH;
    }
  };

  /**
   * Add the given teammate to this hypothesis. If the \c playerNumber is
   * already in use, the teammate is updated if the timestamp of \c newTeammate
   * is newer.
   * \param [in] playerNumber The number of the teammate to be assigned.
   * \param [in] newTeammate The teammate info to be assigned.
   */
  void addTeammateInfo(int playerNumber, const RemoteKalmanPositionHypothesis::TeammateInfo& newTeammate);

  /**
   * Add the given set of teammates to this hypothesis. For any teammate in
   * \c newTeammates the method \c addTeammateInfo(..) is called.
   * \param [in] newTeammates The set of teammates to be assigned.
   */
  void addTeammates(const std::map<int, RemoteKalmanPositionHypothesis::TeammateInfo>& newTeammates);

  /**
   * Returns the map of teammates assigned to this hypothesis indexed by the 
   * player numbers.
   * \return The map of assigned teammates.
   */
  const std::map<int, RemoteKalmanPositionHypothesis::TeammateInfo>& teammates() const { return m_teammates; }

  /**
   * Returns the last time when this hypothesis was updated by the player with
   * the given number. Return 0 if \c playerNumber never updated this hypothesis.
   * \param [in] playerNumber The number of the teammate to be assigned.
   * \return Timespamp of last update by teammate with \c playerNumber (in ms).
   */
  unsigned timeWhenUpdatedByTeammate(int playerNumber) const;

  /**
   * Returns the player numbers of all teammates which contributed to this
   * hypothesis as string (e.g. "1,2,3").
   * \return The set of assigned player numbers.
   */
  std::string teammatesString() const;


  //MARK: Helper methods

  /// Fixes compiler warning for overloaded virtual function merge(const RemoteKalmanPositionHypothesis&)
  using KalmanPositionHypothesis::merge;
  /**
   * This method is called when another hypothesis (\c source) gets merged into
   * this one. This method adds meta information from \c source into \c *this
   * (e.g. the history of the <tt>percepts per second</tt> tracking and the number
   * of sensor updates. It does not merge the Kalman filters.
   * \param [in] source The hypothesis which gets merged into \c *this.
   */
  virtual void merge(const RemoteKalmanPositionHypothesis& source);

protected:
  /**
   * Stores various information from teammates who has contributed to this
   * hypothesis with their local models. The teammates are indexed by player
   * numbers.
   * \see RemoteKalmanPositionHypothesis::TeammateInfo
   */
  std::map<int, RemoteKalmanPositionHypothesis::TeammateInfo> m_teammates;
};
