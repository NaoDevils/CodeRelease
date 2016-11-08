/**
 * @file ZMPIPController2012.h
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK


#include <list>
#include <stdio.h>
#include "StepData.h"
#include "Point.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ReferenceModificator.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/RobotParameters.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"

const int static_N = 50; /**< Length of the preview phase */
#define SEPERATEDXY 1

MODULE(FLIPMController,
{ ,
REQUIRES(WalkingEngineParams),
REQUIRES(ObservedFLIPMError),
REQUIRES(RefZMP),
REQUIRES(InertialSensorData),
REQUIRES(ZMPModel),
PROVIDES(TargetCoM),
});

class FLIPMController : public FLIPMControllerBase
{
  ROBOT_PARAMETER_CLASS(FLIPMContX, FLIPMController)
	  PARAM(float, m)
	  PARAM(float, M)
	  PARAM(float, g)
	  PARAM(float, z_h)
	  PARAM(float, dt)
	  PARAM(float, D)
	  PARAM(float, E)
	  PARAM(float, Qe)
	  PARAM(float, Qx)
	  PARAM(float, R)
	  PARAM(int, N)
	  PARAM(Matrix6f, A)
	  PARAM(Vector6f, b)
	  PARAM(Matrix1x6f, c)
	  PARAM(float, Gi)
	  PARAM(Matrix1x6f, Gx)
	  PARAM(Vector50f, Gd) /// WARNING STATIC N!!!!
  END_ROBOT_PARAMETER_CLASS(FLIPMContX)

  ROBOT_PARAMETER_CLASS(FLIPMContY, FLIPMController)
    PARAM(float, m)
    PARAM(float, M)
    PARAM(float, g)
    PARAM(float, z_h)
    PARAM(float, dt)
    PARAM(float, D)
    PARAM(float, E)
    PARAM(float, Qe)
    PARAM(float, Qx)
    PARAM(float, R)
    PARAM(int, N)
    PARAM(Matrix6f, A)
    PARAM(Vector6f, b)
    PARAM(Matrix1x6f, c)
    PARAM(float, Gi)
    PARAM(Matrix1x6f, Gx)
    PARAM(Vector50f, Gd) /// WARNING STATIC N!!!!
  END_ROBOT_PARAMETER_CLASS(FLIPMContY)

public:
  
	/** Constructor with all needed source data structures.
	 * @param theRefZMP The desired ZMP.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
	FLIPMController();
  
	~FLIPMController() { } /**< Destructor */
  
	/** Start the controller. */
	void Start() { }
  
	/** Tells the controller to stop moving after the last added step. */
	void End() { reset(); }
  
	ZMP getReferenceZMP();
  
	/**
	 * Calculate one target position of the CoM.
	 * @param targetCoM The representation to fill with the position.
	 */
  void update(TargetCoM & targetCoM);

private:
	typedef std::list<ZMP> ZMPList;
	/** Current element in the list. */
	ZMPList::iterator kElement;
	/** List of reference ZMP. */
	ZMPList pRef;
  
  Eigen::Matrix<Vector6f, 2, 1 > x;	/**< Observer state vector*/
  Vector2f v;                  		/**< Controller internal value. */
  
	bool isRunning,			/**< Is the controller running? */
  kElementEmpty;		/**< Is there a valid current ZMPList element? */
  
	void reset();			/**< Resets the controller. */
	void Shrink();			/** Deletes no more needed elements in lists. */
	void addRefZMP(ZMP zmp);/** Add a ZMP position to the list. */
	Point controllerStep(); /** Calculate one step of the system. */
  void executeController(Dimension d, const Eigen::Matrix< float, 1, static_N>  &refZMP);
};