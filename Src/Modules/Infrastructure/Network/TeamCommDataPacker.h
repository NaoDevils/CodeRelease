/**
 * @file TeamCommDataPacker.h
 * Declaration of module TeamCommDataPacker
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author Colin Graf
 */

#pragma once

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols/RemoteControl.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RefereeGesture.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"

MODULE(TeamCommDataPacker,
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TimeSynchronization),
  REQUIRES(RobotPose),
  USES(RemoteControl),
  REQUIRES(BallModel),
  REQUIRES(BehaviorData),
  REQUIRES(TeamCommEvents),
  REQUIRES(WhistleDortmund),
  REQUIRES(SpeedInfo),
  REQUIRES(LocalRobotMap),
  REQUIRES(FallDownState),
  REQUIRES(RefereeGesture),
  PROVIDES(TeamCommOutput)
);

/**
* @class TeamCommDataPacker
* A modules for sending some representation to teammates
*/
class TeamCommDataPacker : public TeamCommDataPackerBase
{
public:
private:
  /**
  * The update function called in each cognition process cycle
  * @param TeamCommDataPackerOutput An empty output representation
  */
  virtual void update(TeamCommOutput& teamCommOutput);

  /**
  * Fills the SPLStandardMessage Header.
  */
  void fillStandardMessage(MessageQueue& queue, TeamCommData& message);
};
