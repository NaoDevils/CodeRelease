/**
* \file BallSearch.h
* The file declares a class that containts data for the BallSearch.
*/

#pragma once
#include "BehaviorData.h"
#include "PositioningSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
* \class BallSearch
* The file declares a class that containts data for the BallSearch.
*/

STREAMABLE(BallSearch,
  STREAMABLE(BallSearchPositions,,
    (std::vector<Pose2f>) poses
  )
  ,
  ((BehaviorData) RobotRoleAssignmentVector) rolesInBallSearch,
  (std::vector<BallSearchPositions>) ballSearchPositions, // ordered in the same way as rolesInBallSearch!

  (Pose2f)(Pose2f()) nearestMarkPosition, // robot to pressure, if nothing else useful to do
  (bool)(false) threatNear
);