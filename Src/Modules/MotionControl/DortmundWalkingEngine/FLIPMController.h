/**
 * @file FLIPMController.h
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 * @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK


#include <list>
#include <stdio.h>
#include "StepData.h"
#include "Point.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Modules/MotionControl/DortmundWalkingEngine/FLIPMObserver.h"

MODULE(FLIPMController,
  REQUIRES(WalkingEngineParams),
  REQUIRES(FLIPMParameter),
  REQUIRES(FLIPMControllerParameter),
  REQUIRES(ObservedFLIPMError),
  REQUIRES(RefZMP2018),
  REQUIRES(ZMPModel),
  REQUIRES(FootSteps),
  REQUIRES(Footpositions),
  REQUIRES(SpeedInfo),
  USES(WalkingInfo),
  PROVIDES(TargetCoM),
  LOADS_PARAMETERS(,
    (float)(0.26) min_z_h,
    (float)(0.27) max_z_h,
    (float)(0.001) epsilon_z_h
  )
);


class FLIPMController : public FLIPMControllerBase
{
public:
  /** Constructor with all needed source data structures.
	 * @param theRefZMP The desired ZMP.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
  FLIPMController();

  ~FLIPMController() {} /**< Destructor */

  /** Start the controller. */
  void Start() { reset(); }

  /** Tells the controller to stop moving after the last added step. */
  void End() { reset(); }

  /**
	 * Calculate one target position of the CoM.
	 * @param targetCoM The representation to fill with the position.
	 */
  void update(TargetCoM& targetCoM);

private:
  Eigen::Matrix<Eigen::Matrix<double, 1, PREVIEW_LENGTH>, 2, 1> lastRefZMP; /**< Reference ZMP of last frame */
  Eigen::Matrix<Eigen::Matrix<double, 1, PREVIEW_LENGTH>, 2, 1> refZMP; /**< Current reference ZMP */

  Eigen::Matrix<Vector6d, 2, 1> x; /**< Observer state vector*/
  Vector2d v; /**< Controller internal value. */

  float dynamic_z_h;

  Eigen::Matrix<Vector6d, 2, 1> x_RCS; /**< Observer state vector*/
  Vector2d v_RCS; /**< Controller internal value. */

  bool isRunning; /**< Is the controller running? */

  int framesToInterpolate = 0;

  void reset(); /**< Resets the controller. */
  Point controllerStep(); /** Calculate one step of the system. */

  void executeController(Dimension d, const Eigen::Matrix<double, 1, PREVIEW_LENGTH>& refZMP);
  void executeRCSController(Dimension d, const Eigen::Matrix<double, 1, PREVIEW_LENGTH>& refZMP);
};
