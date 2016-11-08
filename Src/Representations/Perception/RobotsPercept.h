// Simple representation of the robot percepts used in world model
// Dortmund version

#ifndef __RobotsPercept_h_
#define __RobotsPercept_h_

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include <algorithm>

STREAMABLE(ObstacleBasePoints,
{
  STREAMABLE(ObstacleBasePoint,
  {
    ENUM(Direction,
    {,
      up,
      down,
      left,
      right,
    }),
    (bool)(false) upperCam, // whether obstacle was found in upper cam
    (bool)(false) certain, // whether preprocessor is sure there is an obstacle, usually if obstacle is very near
    (Direction) direction, // where the obstacle lies relative to base point
    (Vector2f)(Vector2f::Zero()) pointInImage, /**< The nearest obstacle point in image coordinates. */
  });
  void draw() const,
  (std::vector<ObstacleBasePoint>) basePoints,
});

STREAMABLE(RobotEstimate,
{
  ENUM(RobotType,
  {,
    unknownRobot,
    teammateRobot,
    opponentRobot,
    noRobot,
    invalid,
  }),

  (Pose2f) locationOnField,
  (float)(10000.f) distance,
  (float)(0.f) validity,
  (Vector2i) imageUpperLeft,
  (Vector2i) imageLowerRight,
  (bool)(false) fromUpperImage,
  (RobotType) robotType,
});

STREAMABLE(RobotsPercept,
{
  RobotsPercept() = default;
  
  void draw() const;
  std::vector<RobotEstimate> robots,
});

#endif // __RobotsPercept_h_