/**
 * @file TeamCommDataPacker.h
 * Declaration of module TeamCommDataPacker
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author Colin Graf
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/SimpleRobotsDistributed.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"

MODULE(TeamCommDataPacker,
  REQUIRES(BallModel),
  REQUIRES(BehaviorData),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(Ballchaser),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(SpeedInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PositioningSymbols),
  REQUIRES(RobotHealth),
  REQUIRES(RobotInfo),
  REQUIRES(RobotsPercept),
  REQUIRES(RobotsPerceptUpper),
  REQUIRES(LocalRobotMap),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(SideConfidence),
  REQUIRES(SimpleRobotsDistributed),
  REQUIRES(TeammateData),
  REQUIRES(WhistleDortmund),
  REQUIRES(TeamCommEvents),
  REQUIRES(TimeSynchronization),
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
  TeamCommOutput fillStandardMessage(MessageQueue& queue);
};
