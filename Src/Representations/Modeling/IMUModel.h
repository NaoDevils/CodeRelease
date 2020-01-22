/**
* @file IMUModel.h
*
* Declaration of representation IMUModel
*
* @author <a href="mailto:kaden@informatik.hu-berlin.de">Steffen Kaden</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*/

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(IMUModel,
{,
  (Vector3f)(Vector3f::Zero()) location,             // simply integrated velocity
  (Vector3f)(Vector3f::Zero()) velocity,             // simply integrated acceleration
  (Vector3f)(Vector3f::Zero()) acceleration,         // gravity adjusted, UKF-filtered acceleration of body frame in "global" inertial frame
  (Vector3f)(Vector3f::Zero()) acceleration_sensor,  // sensor values (TODO: offset adjustment)

  // full 3D rotation, no information lost
  (Vector3f)(Vector3f::Zero()) rotation,                   // rotation vector representation (angle * unit_length_axis_vector) of UKF-filtered body rotation in inertial frame,
                                                           // it rotates body coordinates into the "global" inertial frame
  (Vector3f)(Vector3f::Zero()) rotational_velocity,        // 3d vector representing UKF-filtered rotational velocity around each body frame axis
  (Vector3f)(Vector3f::Zero()) rotational_velocity_sensor, // 3d vector representing rotational velocity around each sensor frame axis


  // clasical '2d projection' similar to what aldebaran provides
  (Vector2a)(Vector2a::Zero()) orientation, // angles around x and y axis, only for high level decisions, deprecated for usage in coordinate transformations, use orientation_rotvec or rotation instead

  // NOTE: this it the good stuff!!!
  // wie 'rotation' aber ohne den z-Anteil
  (Vector3f)(Vector3f::Zero()) orientation_rotvec,  // rotation vector representation (angle * unit_length_axis_vector) of rotation considering only rotation around x and y axis,
                                                    // it rotates body coordinates into the "global" inertial frame neglecting any rotation around the global z axis
});
