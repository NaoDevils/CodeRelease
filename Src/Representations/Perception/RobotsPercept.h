// Simple representation of the robot percepts used in world model
// Dortmund version

#pragma once
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include <algorithm>
#include <iomanip>

STREAMABLE(ObstacleBasePoints,
  STREAMABLE(ObstacleBasePoint,
    ENUM(Direction,
      up,
      down,
      left,
      right
    ),
    (bool)(false) upperCam, /**< whether obstacle was found in upper cam. */
    (bool)(false) certain, /**< whether preprocessor is sure there is an obstacle, usually if obstacle is very near. */
    (Direction) direction, /**< where the obstacle lies relative to base point. */
    (Vector2f)(Vector2f::Zero()) pointInImage /**< The nearest obstacle point in image coordinates. */
  );
  void draw() const,
  (std::vector<ObstacleBasePoint>) basePoints
);

STREAMABLE(RobotEstimate,
  ENUM(RobotType,
    unknownRobot,
    teammateRobot,
    opponentRobot,
    noRobot,
    invalid
  );
  ENUM(DetectionSource,
    yoloHypothesis,
    recallAcceptance,
    closeRobotDetection
  );
  void getUpperImageCoordinates(Vector2i& ulRes, Vector2i& lrRes) const;,
  (bool)(false) keeper,
  (Pose2f) locationOnField,                   /**< Position of the robot in local robot coordinates. */
  (float)(10000.f) distance,
  (float)(0.f) validity,
  (Vector2i) imageUpperLeft,
  (Vector2i) imageLowerRight,
  (bool)(false) fromUpperImage,
  (RobotType) robotType,
  (float) teamAssignmentConfidence,
  (unsigned int) timestampFromImage,
  (std::vector<unsigned char>) patch,
  (DetectionSource)(DetectionSource::yoloHypothesis) source,
  (int)(0) trackingAge
);

STREAMABLE(RobotsPercept,
  RobotsPercept() { robots.reserve(15); }
  void draw() const,
  (std::vector<RobotEstimate>) robots
);

STREAMABLE_WITH_BASE(RobotsPerceptClassified, RobotsPercept, );

STREAMABLE_WITH_BASE(RobotsPerceptTeam, RobotsPercept, );

STREAMABLE_WITH_BASE(RobotsPerceptOrientation, RobotsPercept, );

STREAMABLE_WITH_BASE(ProcessedRobotsHypotheses, RobotsPercept, );

STREAMABLE_WITH_BASE(RobotsHypothesesYolo, RobotsPercept, );
STREAMABLE_WITH_BASE(RobotsHypothesesYoloUpper, RobotsPercept, );

STREAMABLE(RobotsPerceptCompressed,
  RobotsPerceptCompressed() = default;
  RobotsPerceptCompressed(const RobotsPercept &other)
  {
    robots.reserve(other.robots.size());
    for (const RobotEstimate& otherRobot : other.robots)
      robots.emplace_back(otherRobot);
  }

  operator RobotsPercept() const
  {
    RobotsPercept robotsPercept;
    robotsPercept.robots.reserve(robots.size());
    for (const RobotEstimateCompressed& robot : robots)
      robotsPercept.robots.emplace_back(robot);

    return robotsPercept;
  }

  STREAMABLE(RobotEstimateCompressed,
    RobotEstimateCompressed() = default;
    RobotEstimateCompressed(const RobotEstimate &other)
    {
      locationOnField.x() = static_cast<short>(other.locationOnField.translation.x());
      locationOnField.y() = static_cast<short>(other.locationOnField.translation.y());
      distance = static_cast<short>(other.distance);
      validity = static_cast<unsigned char>(other.validity * 255.f);
      robotType = other.robotType;
    }

    operator RobotEstimate() const
    {
      RobotEstimate re;
      re.locationOnField.translation.x() = static_cast<float>(locationOnField.x());
      re.locationOnField.translation.y() = static_cast<float>(locationOnField.y());
      re.locationOnField.rotation = 0.f;
      re.distance = static_cast<float>(distance);
      re.validity = static_cast<float>(validity) / 255.f;
      re.robotType = robotType;
      return re;
    }
    ,
    (Vector2s) locationOnField, /**< Position of the robot in local robot coordinates. */
    (short)(10000) distance,
    (unsigned char)(0) validity,
    ((RobotEstimate) RobotType) robotType
  ),

  (std::vector<RobotEstimateCompressed>) robots
);
