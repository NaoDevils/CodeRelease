/**
* @file ZMPModel.h
*
* Declaration of class ZMPModel
*
* @author <A href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"

STREAMABLE(FootPose,,
  (Pose3f) left, 
  (Pose3f) right
);

/**
 * @class ZMPModel
 *
 * Contains information about ZMP.
 */
STREAMABLE(ZMPModel,
  /** Draws the ZMP */
  void drawFoot(bool left, const Pose2f& baseInImage, float scale) const;
  void draw() const,
  (FootPose) feetPose, // feet pose in Robot Coordinate System
  (Vector2f)(Vector2f::Zero()) ZMP_FCS, // ZMP in Foot Coordinate System
  (Vector3f)(Vector3f::Zero()) ZMP_RCS, // ZMP in Robot Coordinate System
  (Vector3f)(Vector3f::Zero()) ZMP_WCS  // ZMP in Walking Engine World Coordinate System
);
