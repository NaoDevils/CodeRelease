/**
 * \file GlobalMultiKalmanModel.h
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 * 
 * Declaration of class GlobalMultiKalmanModel.
 * This class is an extension to \c MultiKalmanModel
 */

#pragma once

#include "MultiKalmanModel.h"


/**
 * \class GlobalMultiKalmanModel
 *
 * This class is an extension to \c MultiKalmanModel for handling of Kalman hypotheses.
 * Local models stores positions and velocities in global coordinates.
 * \see MultiKalmanModel, KalmanPositionHypothesis
 */
template <typename hypothesis_t = KalmanPositionHypothesis, bool towardsOneModel = true> class GlobalMultiKalmanModel : public MultiKalmanModel<hypothesis_t, towardsOneModel>
{
public:
  /**
   * Default constructor creates a new and empty \c GlobalMultiKalmanModel object.
   */
  GlobalMultiKalmanModel() : MultiKalmanModel<hypothesis_t, towardsOneModel>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  GlobalMultiKalmanModel(unsigned perceptDuration) : MultiKalmanModel<hypothesis_t, towardsOneModel>(perceptDuration) {}
  /**
   * Destructor.
   */
  ~GlobalMultiKalmanModel() {}

  /**
   * States whether this model use relative robot coordinates or global field 
   * coordinates. Local models use always relative robot coordinates while remote
   * models always use global field coordinates.
   * \return \c false if the hypotheses within this model are represented in 
   *         relative robot coordinates,
   *         \c false otherwise (global field coordinates).
   */
  bool usesRelativeCoordinates() const override { return false; }
};
