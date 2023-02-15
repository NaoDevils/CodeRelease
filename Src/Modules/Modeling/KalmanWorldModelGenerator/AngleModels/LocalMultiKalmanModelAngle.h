/**
 * \file LocalMultiKalmanModelAngle.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * \author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * 
 * Declaration of class LocalMultiKalmanModelAngle.
 * This class is an extension to \c MultiKalmanModelAngle
 */

#pragma once

#include "MultiKalmanModelAngle.h"


/**
 * \class LocalMultiKalmanModelAngle
 *
 * This class is an extension to \c MultiKalmanModelAngle for handling of local
 * models received from teammates.
 * Local models stores positions and velocities in relative robot coordinates.
 * \see MultiKalmanModelAngle, KalmanPositionHypothesis
 */

template <typename hypothesis_t, bool towardsOneModel> class LocalMultiKalmanModelAngle : public MultiKalmanModelAngle<hypothesis_t, towardsOneModel>
{
public:
  /**
   * Default constructor creates a new and empty \c LocalMultiKalmanModelAngle object.
   */
  LocalMultiKalmanModelAngle() : MultiKalmanModelAngle<hypothesis_t, towardsOneModel>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  LocalMultiKalmanModelAngle(unsigned perceptDuration) : MultiKalmanModelAngle<hypothesis_t, towardsOneModel>(perceptDuration) {}
  /**
   * Destructor.
   */
  ~LocalMultiKalmanModelAngle() {}

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
