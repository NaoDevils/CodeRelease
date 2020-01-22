/**
 * \file MultipleBallModel.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * 
 * Declaration of classes \c LocalMultipleBallModel and \c RemoteMultipleBallModel.
 * They specialize <tt>LocalMultiKalmanModel</tt> and <tt>RemoteMultiKalmanModel</tt>
 * for ball modeling.
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "../../Models/LocalMultiKalmanModel.h"
#include "../../Models/RemoteMultiKalmanModel.h"


/**
 * Updates the given \c BallState from a \c KalmanPositionHypothesis. The ball
 * state object is used in \c BallModel::estimate and it is filled with content
 * extracted from the kalman hypothesis.
 * \param [in] hypothesis The \c KalmanPositionHypothesis from which the ball 
 *                        state is extracted.
 * \param [out] ballState The \c BallState which should be updated
 */
void updateEstimatedBallState(const KalmanPositionHypothesis& hypothesis, BallState& ballState);


/**
 * \class LocalMultipleBallModel
 * 
 * Specialization of \c LocalMultiKalmanModel for local ball modeling.
 */
class LocalMultipleBallModel : public LocalMultiKalmanModel<>
{
public:

  /** 
   * Default constructor creates a new and empty \c LocalMultipleBallModel object.
   */
  LocalMultipleBallModel() : LocalMultiKalmanModel<>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  LocalMultipleBallModel(unsigned perceptDuration) : LocalMultiKalmanModel<>(perceptDuration) {}
  /** 
   * Destructor.
   */
  ~LocalMultipleBallModel() {}

  /**
   * Draws all ball hypotheses as local ball model hypotheses.
   */
  virtual void draw() const;
};


/**
 * \class RemoteMultipleBallModel
 *
 * Specialization of \c RemoteMultiKalmanModel for remote ball modeling.
 */
class RemoteMultipleBallModel : public RemoteMultiKalmanModel<>
{
public:
  /** 
   * Default constructor creates a new and empty \c MultipleBallModel object.
   */
  RemoteMultipleBallModel() : RemoteMultiKalmanModel<>(), perceptDuration(4000) {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  RemoteMultipleBallModel(unsigned perceptDuration)
    : RemoteMultiKalmanModel<>(perceptDuration)
    , perceptDuration(perceptDuration) {}
  /** 
   * Destructor.
   */
  ~RemoteMultipleBallModel() {}

  /**
   * The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  unsigned perceptDuration;

  /**
   * Draws all ball hypotheses as remote ball model hypotheses.
   */
  virtual void draw();
};
