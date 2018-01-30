/**
* \file RoleSymbols.h
* The file declares a class that containts data about the selected player role and the ball chaser.
* \author Ingmar Schwarz
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Tools/Math/Eigen.h"

/**
* \class RoleSymbols
* A class that containts data about the selected player role and the ball chaser.
*/
STREAMABLE(RoleSymbols,
{
  void draw() const,
  ((BehaviorData) RoleAssignment)(undefined) role, /**< Selected role. */
  ((BehaviorData) RoleAssignment)(undefined) lastRole, /**< Last selected role, only interesting for role switching. */
  (bool)(false) isBallMine, /**< True, if this robot should be ball chaser. */
});
