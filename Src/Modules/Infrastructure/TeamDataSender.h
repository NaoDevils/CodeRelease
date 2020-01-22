/**
* @file TeamDataSender.h
* Declaration of module TeamDataSender
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
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

MODULE(TeamDataSender,
{ ,
  REQUIRES(BallModel),
  REQUIRES(BehaviorData),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(GroundContactState),
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
  PROVIDES_WITHOUT_MODIFY(TeamDataSenderOutput),
  LOADS_PARAMETERS(
  {,
    (unsigned)(0) avgWalkingSpeed,
    (unsigned)(0) maxKickDistance,
    (unsigned)(0) maxNumberOfObstaclesToSend, /**< Do not send more obstacles than this. */
  }),
});

/**
* @class TeamDataSender
* A modules for sending some representation to teammates
*/
class TeamDataSender : public TeamDataSenderBase
{
public:

  /** Default constructor */
  TeamDataSender() : TeamDataSenderBase("teamDataSender.cfg"), sendFrames(0) { }

  ~TeamDataSender();

private:
  unsigned int sendFrames; /** Quantity of frames in which team data was sent */

  MessageQueue* outMessage = 0; // MessageQueue for the team output

  size_t dsmSize = 0;

  /**
  * The update function called in each cognition process cycle
  * @param teamDataSenderOutput An empty output representation
  */
  virtual void update(TeamDataSenderOutput& teamDataSenderOutput);

  /**
  * Fills the SPLStandardMessage Header of the TeamDataOut.message
  */
  void fillStandardMessage();

  void fillMixedTeamMessage(RoboCup::SPLStandardMessage& message);
};
