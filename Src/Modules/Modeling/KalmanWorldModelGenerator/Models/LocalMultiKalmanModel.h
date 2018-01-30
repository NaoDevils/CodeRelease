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
template <typename hypothesis_t = KalmanPositionHypothesis>
class LocalMultiKalmanModel : public MultiKalmanModel<hypothesis_t>
{
public:
  /**
   * Default constructor creates a new and empty \c LocalMultiKalmanModel object.
   */
  LocalMultiKalmanModel() : MultiKalmanModel<hypothesis_t>() {}
  /**
   * Destructor.
   */
  ~LocalMultiKalmanModel() {}
  
  
  //MARK: Helper methods

  /**
   * States whether this model use relative robot coordinates or global field 
   * coordinates. Local models use always relative robot coordinates while remote
   * models always use global field coordinates.
   * \return \c true if the hypotheses within this model are represented in 
   *         relative robot coordinates,
   *         \c false otherwise (global field coordinates).
   */
  virtual bool usesRelativeCoordinates() const { return true; }
};
