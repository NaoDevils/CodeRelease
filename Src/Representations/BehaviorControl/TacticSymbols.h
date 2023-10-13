/**
* \file TacticSymbols.h
* The file declares a class that containts data about shared tactics employed by groups of robots.
* \author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Math/Eigen.h"
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/Cone.h>

/**
* \class TacticSymbols
* A class that containts data about shared tactics employed by groups of robots.
*/
STREAMABLE(TacticSymbols,
  void draw() const,

  // Values for tactical decisions
  (unsigned)(1) numberOfActiveFieldPlayers, // myself and active teammates excluding keeper
  (bool)(false) iAmSupported, // TODO Not filled yet
  (bool)(false) interceptBall, // TODO Not filled yet
  (bool)(true) defensiveBehavior,
  (float)(0.5f) activity,
  (bool)(true) kickoffToTheLeft,
  (int)(0) numberOfLeftOwnKickOffSuccess,
  (int)(0) numberOfRightOwnKickOffSuccess,
  (bool)(false) keepRoleAssignment,

  (int)(0) closeToBallRobotNumber,
  (int)(0) closeToBallOpponentRobotNumber,
  (Pose2f)(Pose2f()) closeToBallRobot, // Not closest since there is a threshold to count as close
  (Pose2f)(Pose2f()) closeToBallOpponentRobot, // Not closest since there is a threshold to count as close

  (Cone)(Cone()) defensiveCone
);
