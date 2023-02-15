/**
 * @file TeamCommReceiver.h
 * This module receives raw team communication data.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/TeamCommSocket.h"

MODULE(TeamCommReceiver,
  REQUIRES(TeamCommSocket),
  PROVIDES(TeamCommInput)
);

class TeamCommReceiver : public TeamCommReceiverBase
{
private:
  void update(TeamCommInput& teamCommData);
};
