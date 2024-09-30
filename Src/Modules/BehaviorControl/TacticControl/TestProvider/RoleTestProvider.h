/**
* @file RoleTestProvider.h
*
* Declaration of class RoleTestProvider.
* The RoleTestProvider takes the roles that are specified in the associated config file and assignes them in an appropriate way (e.g. number 1 is keeper).
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/RoleSymbols.h"


MODULE(RoleTestProvider,
    REQUIRES(RobotInfo),
    REQUIRES(TeammateData),
    PROVIDES(RoleSymbols),
    LOADS_PARAMETERS(,
      ((BehaviorData) RobotRoleAssignmentVector) rolesToTest
    )
  );

/**
* @class RoleTestProvider
* Symbols for role decision
*/
class RoleTestProvider : public RoleTestProviderBase
{
public:
  /** Constructor */
  RoleTestProvider() {}

private:
  /** Updates some of the symbols */
  void update(RoleSymbols& roleSymbols);
  /** Find the player numbers and current positions of the active players in the team. */
  void getCurrentTeamStatus(std::vector<int>& playerNumbers);
};
