/**
 * \file LocalMultiKalmanModel.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * 
 * Declaration of class LocalMultiKalmanModel.
 * This class is an extension to \c MultiKalmanModel
 */

#pragma once

#include "MultiKalmanModel.h"


/**
 * \class LocalMultiKalmanModel
 *
 * This class is an extension to \c MultiKalmanModel for handling of local
 * models received from teammates.
 * Local models stores positions and velocities in relative robot coordinates.
 * \see MultiKalmanModel, KalmanPositionHypothesis
 */
template <typename hypothesis_t = KalmanPositionHypothesis, bool towardsOneModel = true> class LocalMultiKalmanModel : public MultiKalmanModel<hypothesis_t, towardsOneModel>
{
public:
  /**
   * Default constructor creates a new and empty \c LocalMultiKalmanModel object.
   */
  LocalMultiKalmanModel() : MultiKalmanModel<hypothesis_t, towardsOneModel>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  LocalMultiKalmanModel(unsigned perceptDuration) : MultiKalmanModel<hypothesis_t, towardsOneModel>(perceptDuration) {}
  /**
   * Destructor.
   */
  ~LocalMultiKalmanModel() {}

  /**
   * States whether this model use relative robot coordinates or global field 
   * coordinates. Local models use always relative robot coordinates while remote
   * models always use global field coordinates.
   * \return \c true if the hypotheses within this model are represented in 
   *         relative robot coordinates,
   *         \c false otherwise (global field coordinates).
   */
  bool usesRelativeCoordinates() const override { return true; }
};
