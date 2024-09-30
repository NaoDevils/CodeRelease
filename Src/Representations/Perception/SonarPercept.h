// Simple representation of what the sonar percepts

#pragma once

#include <vector>
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/ColorRGBA.h"

STREAMABLE(SonarEstimate,
  ENUM(SensorPosition,
    left,
    middle,
    right
  );
  ,
  (SensorPosition)(-1)sensorPosition,            /* Position of the sonar sensor */
  (unsigned)(0)timestamp,
  (float)(0.f)distance,
  (float)(0.f)validity,
  (Pose2f)(Pose2f(0,0,0))relativePosition        /* Position of the robot in local robot coordinates. */
);

STREAMABLE(SonarPercept,
  void draw() const;
  void drawArc(const SonarEstimate& sonarEstimate, const RobotPose& theRobotPose, const Angle& sonarDetectionCone, const Vector2f& sonarSensorRelativePosition, const ColorRGBA& arcColor) const;
  ,
  (std::vector<SonarEstimate>)sonarEstimates
);
