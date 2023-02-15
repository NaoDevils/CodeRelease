// Simple representation of the robot percepts used in world model
// Dortmund version

#pragma once
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include <algorithm>

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
  ),
  (Pose2f) locationOnField,                   /**< Position of the robot in local robot coordinates (rotation not filled yet (2017)). */
  (float)(10000.f) distance,
  (float)(0.f) validity,
  (Vector2i) imageUpperLeft,
  (Vector2i) imageLowerRight,
  (bool)(false) fromUpperImage,
  (RobotType) robotType,
  (float) teamAssignmentConfidence,
  (unsigned int) timestampFromImage
);

STREAMABLE(RobotsPercept,
  RobotsPercept() = default;

  void draw() const,
  (std::vector<RobotEstimate>) robots
);

STREAMABLE(RobotsPerceptUpper,
  RobotsPerceptUpper() = default;

  void draw() const,
  (std::vector<RobotEstimate>) robots
);

STREAMABLE(RobotsPerceptCompressed,
  RobotsPerceptCompressed() = default;
  RobotsPerceptCompressed(const RobotsPercept &other)
  {
    robots.clear();
    for (unsigned int i = 0; i < other.robots.size(); i++)
    {
      robots.push_back(RobotEstimateCompressed(other.robots[i]));
    }
  }

  operator RobotsPercept() const
  {
    RobotsPercept robotsPercept;
    robotsPercept.robots.clear();
    for (unsigned int i = 0; i < robots.size(); i++)
    {
      RobotEstimate re;
      re.locationOnField.translation.x() = static_cast<float>(robots[i].locationOnField.x());
      re.locationOnField.translation.y() = static_cast<float>(robots[i].locationOnField.y());
      re.locationOnField.rotation = 0.f;
      re.distance = static_cast<float>(robots[i].distance);
      re.validity = static_cast<float>(robots[i].validity) / 255.f;
      re.robotType = robots[i].robotType;
      robotsPercept.robots.push_back(re);
    }
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
    },
    (Vector2s) locationOnField, /**< Position of the robot in local robot coordinates. */
    (short)(10000) distance,
    (unsigned char)(0) validity,
    ((RobotEstimate) RobotType) robotType
  ),

  (std::vector<RobotEstimateCompressed>) robots
);


STREAMABLE(RobotsPerceptUpperCompressed,
  RobotsPerceptUpperCompressed() = default;
  RobotsPerceptUpperCompressed(const RobotsPerceptUpper &other)
  {
    robots.clear();
    for (unsigned int i = 0; i < other.robots.size(); i++)
    {
      robots.push_back(RobotEstimateUpperCompressed(other.robots[i]));
    }
  }

  operator RobotsPerceptUpper() const
  {
    RobotsPerceptUpper robotsPercept;
    robotsPercept.robots.clear();
    for (unsigned int i = 0; i < robots.size(); i++)
    {
      RobotEstimate re;
      re.locationOnField.translation.x() = static_cast<float>(robots[i].locationOnField.x());
      re.locationOnField.translation.y() = static_cast<float>(robots[i].locationOnField.y());
      re.locationOnField.rotation = 0.f;
      re.distance = static_cast<float>(robots[i].distance);
      re.validity = static_cast<float>(robots[i].validity) / 255.f;
      re.robotType = robots[i].robotType;
      robotsPercept.robots.push_back(re);
    }
    return robotsPercept;
  }

  STREAMABLE(RobotEstimateUpperCompressed,
    RobotEstimateUpperCompressed() = default;
    RobotEstimateUpperCompressed(const RobotEstimate &other)
    {
      locationOnField.x() = static_cast<short>(other.locationOnField.translation.x());
      locationOnField.y() = static_cast<short>(other.locationOnField.translation.y());
      distance = static_cast<short>(other.distance);
      validity = static_cast<unsigned char>(other.validity * 255.f);
      robotType = other.robotType;
    },
    (Vector2s) locationOnField, /**< Position of the robot in local robot coordinates. */
    (short)(10000) distance,
    (unsigned char)(0) validity,
    ((RobotEstimate) RobotType) robotType
  ),

  (std::vector<RobotEstimateUpperCompressed>) robots
);
