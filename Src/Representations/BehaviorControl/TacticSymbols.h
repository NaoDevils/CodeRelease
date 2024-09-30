/**
* \file TacticSymbols.h
* The file declares a class that containts data about shared tactics employed by groups of robots.
* \author Ingmar Schwarz
*/

#pragma once

#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Ranges/Cone.h"

/**
* \class TacticSymbols
* A class that containts data about shared tactics employed by groups of robots.
*/
STREAMABLE(TacticSymbols,
  ENUM(Danger,
    IMPOSSIBLE,
    NONE,
    LOW,
    MEDIUM,
    HIGH
  );
  ENUM(GameState,
    STATE_PLAYING_KICKOFF_OWN
  );

  [[nodiscard]] bool hasClosestRobot() const
  { return ballToRobotDistance >= 0.f;
  };
  [[nodiscard]] bool hasClosestOpponentRobot() const
  { return ballToOpponentRobotDistance >= 0.f;
  };

  void draw() const,

  // Values for tactical decisions
  (unsigned)(1) numberOfActiveFieldPlayers, // myself and active teammates excluding keeper
  (bool)(false) iAmSupported, // TODO Not filled yet
  (bool)(false) interceptBall, // TODO Not filled yet
  (bool)(true) defensiveBehavior,
  (std::string)("init") currentSide,
  (std::string)("init") currentDirection,
  (float)(0.5f) activity,
  (bool)(true) kickoffToTheLeft,
  (int)(0) numberOfLeftOwnKickOffSuccess,
  (int)(0) numberOfRightOwnKickOffSuccess,
  (bool)(false) keepRoleAssignment,

  (float)(0) ballToRobotDistance, // negative if no robot
  (float)(0) ballToOpponentRobotDistance, // negative if no opponent robot
  (Pose2f)(Pose2f()) closestToBallRobot,
  (Pose2f)(Pose2f()) closestToBallOpponentRobot,
  (float)(0) untilOpponentStealsBallTime, // Negative if no opponent is close
  (Danger)(Danger::IMPOSSIBLE) ballInDanger,

  (Cone)(Cone()) defensiveCone
);
