/**
 * @file TeamCommSender.h
 * This module sends raw team communication data.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/TeamCommSocket.h"
#include "Representations/Infrastructure/TeamCommSenderOutput.h"

MODULE(TeamCommSender,
  REQUIRES(RobotInfo),
  REQUIRES(TeamCommOutput),
  REQUIRES(TeamCommSocket),
  REQUIRES(FrameInfo),
  PROVIDES(TeamCommSenderOutput)
);

class TeamCommSender : public TeamCommSenderBase
{
private:
  void update(TeamCommSenderOutput& teamCommData);
};
