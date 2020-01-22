/**
* @file TorsoMatrixProvider.h
* Declaration of module TorsoMatrixProvider.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Modeling/IMUModel.h"

STREAMABLE(TorsoMatrixEnums,
{
  ENUM(AngleSource,
  { ,
    inertialData,
    inertialSensorData,
    imuModel,
  }),
});

MODULE(TorsoMatrixProvider,
{ ,
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(InertialSensorData),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(IMUModel),
  PROVIDES(TorsoMatrix),
  USES(TorsoMatrix),
  PROVIDES(OdometryData),
  LOADS_PARAMETERS(
  { ,
   ((TorsoMatrixEnums) AngleSource)(inertialData) anglesource,
  }),
});

/**
 * @class TorsoMatrixProvider
 * A module that provides the (estimated) position and velocity of the inertia board.
 */
class TorsoMatrixProvider : public TorsoMatrixProviderBase
{

private:
  float lastLeftFootZRotation; /**< The last z-rotation of the left foot. */
  float lastRightFootZRotation; /**< The last z-rotation of the right foot. */

  Vector3f lastFootSpan = Vector3f::Zero(); /**< The last span between both feet. */
  Pose3f lastTorsoMatrix; /**< The last torso matrix for calculating the odometry offset. */

  /** Updates the TorsoMatrix representation.
   * @param torsoMatrix The inertia matrix representation which is updated by this module.
   */
  void update(TorsoMatrix& torsoMatrix);

  /** Updates the OdometryData representation.
   * @param odometryData The odometry data representation which is updated by this module.
   */
  void update(OdometryData& odometryData);
};
