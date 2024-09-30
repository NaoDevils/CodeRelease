
/** 
* @file Modules/MotionControl/NaoKinematic.h
* This file implements the inverse kinematic.
* @author <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include <fstream>

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/RingBufferWithSum.h"


MODULE(NaoKinematic,
  REQUIRES(RobotDimensions),
  REQUIRES(KinematicRequest),
  PROVIDES(KinematicOutput),
  LOADS_PARAMETERS(,
    (bool) useBHKinematics
  )
);

/**
 * @class Robot
 * The inverse kinematic for the Nao robot.
 */
class NaoKinematic : public NaoKinematicBase
{
public:
  /** Constructor */
  NaoKinematic();
  /** Desctructor */
  ~NaoKinematic();

  /**
	 * Calculates the angles the first leg. It depends on the request 
	 * which leg is the first leg. E.g. in single support phase the standing
	 * leg is the first leg.
	 * \param whichSideJoint0 Index of first joint of leg.
	 * \param position The desired foot position.
	 * \param rotation The desired foot rotation.
	 * \param jointRequest Filled with the calculated angles.
	 */
  static bool calcLegJoints(Joints::Joint whichSideJoint0, const Vector3f& position, const Vector3f& rotation, JointRequest& jointRequest, const RobotDimensions& robotDimensions);

  /** This method implemenets a mixed kinematic. Here you can set t0, given by the calcLegJoints, to be
	 * compatible to the real Joint 0 (where the angle of both legs has to be equal)
	 * This resuts in a z rotation of the foot.
	 * \param whichSideJoint0 Index of first joint of leg.
	 * \param position The desired foot position.
	 * \param rotation The desired foot rotation. z is ignored here.
	 * \param jointRequest Filled with the calculated angles.
	 */
  static bool calcLegJoints(Joints::Joint whichSideJoint0, const Vector3f& position, const Vector3f& rotation, float t0, JointRequest& jointRequest, const RobotDimensions& robotDimensions);

  /**
   * Calculates the angles from the given foot positions in theKinematicRequest.
   * \param walkingEngineOutput Filled with the angles.
   */
  void update(KinematicOutput& walkingEngineOutput);

private:
  Vector3f checkConstraints(Vector3f lf, float lfr, Vector3f rf, float rfr, bool left);
  std::ofstream logfile;
};
