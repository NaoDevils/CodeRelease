#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/BehaviorControl/BehaviorData.h"

/**
* \class RoleSelection
* A class that containts the current selection of active player roles.
*/
STREAMABLE(RoleSelection,
  ,
  ((BehaviorData) RobotRoleAssignmentVector) selectedRoles
);