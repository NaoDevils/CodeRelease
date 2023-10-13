/**
* \file RoleSymbols.h
* The file declares a class that containts data about the selected player role.
* \author Ingmar Schwarz
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Math/Eigen.h"

/**
* \class RoleSymbols
* A class that containts data about the selected player role and the ball chaser.
*/

STREAMABLE(RoleSymbols,
  void draw() const,
  ((BehaviorData) RobotRoleAssignmentVector) roleSuggestions, /**< Suggested roles for everyone. */
  ((BehaviorData) RoleAssignment)(noRole) role, /**< Selected role for myself. */
  ((BehaviorData) RoleAssignment)(noRole) lastRole, /**< Last selected role for myself. */
  (bool)(true) dynamic
);
