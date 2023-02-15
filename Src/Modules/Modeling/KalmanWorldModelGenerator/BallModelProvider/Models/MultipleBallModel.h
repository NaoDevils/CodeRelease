/**
 * \file MultipleBallModel.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * \author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * 
 * Declaration of classes \c LocalMultipleBallModel and \c RemoteMultipleBallModel.
 * They specialize <tt>LocalMultiKalmanModel</tt> and <tt>RemoteMultiKalmanModel</tt>
 * for ball modeling.
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "../../AngleModels/LocalMultiKalmanModelAngle.h"
#include "../../AngleModels/RemoteMultiKalmanModelAngle.h"

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
class LocalMultipleBallModel : public LocalMultiKalmanModelAngle<KalmanPositionHypothesis, true>
{
public:
  /** 
   * Default constructor creates a new and empty \c LocalMultipleBallModel object.
   */
  LocalMultipleBallModel() : LocalMultiKalmanModelAngle<KalmanPositionHypothesis, true>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  LocalMultipleBallModel(unsigned perceptDuration) : LocalMultiKalmanModelAngle<KalmanPositionHypothesis, true>(perceptDuration) {}
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
class RemoteMultipleBallModel : public RemoteMultiKalmanModelAngle<RemoteKalmanPositionHypothesis, true>
{
public:
  /** 
   * Default constructor creates a new and empty \c MultipleBallModel object.
   */
  RemoteMultipleBallModel() : RemoteMultiKalmanModelAngle<RemoteKalmanPositionHypothesis, true>(), perceptDuration(4000) {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  RemoteMultipleBallModel(unsigned perceptDuration) : RemoteMultiKalmanModelAngle<RemoteKalmanPositionHypothesis, true>(perceptDuration), perceptDuration(perceptDuration) {}
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
