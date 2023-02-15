/**
* \file BehaviorHelper.h
* Thomas Klute
*/

#pragma once
#include <optional>
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include <Representations/Modeling/RobotMap.h>
#include <Representations/Configuration/FieldDimensions.h>


/**
* @class BehaviorHelper
*/
class BehaviorHelper
{
public:
  static Pose2f getOpponentKeeperPose(const RobotMap& map, const FieldDimensions& fieldDimensions);
};
