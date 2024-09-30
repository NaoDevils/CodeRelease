/**
 * @file TeamCommDataPacker.cpp
 * Implementation of module TeamCommDataPacker
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author Colin Graf
 */
#include "TeamCommDataPacker.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(TeamCommDataPacker, cognitionInfrastructure)

void TeamCommDataPacker::update(TeamCommOutput& teamCommOutput)
{
  DECLARE_PLOT("module:TeamCommDataPacker:messageSize");

  teamCommOutput.sendThisFrame = false;

  if (theTeamCommEvents.sendThisFrame)
  {
    Teammate teammate;
    teammate.playerNumber = static_cast<unsigned char>(theRobotInfo.number);
    teammate.teamNumber = static_cast<unsigned char>(theOwnTeamInfo.teamNumber);
    teammate.sendTimestamp = SystemCall::getCurrentSystemTime();
    teammate.fallen = theFallDownState.state != FallDownState::State::upright;
    teammate.timeSynchronization = theTimeSynchronization;
    teammate.robotPose = RobotPoseCompressed(theRobotPose);
    teammate.ballModel = BallModelCompressed(theBallModel);
    teammate.behaviorData = BehaviorDataCompressed(theBehaviorData);
    teammate.teamCommEvents = TeamCommEventsCompressed(theTeamCommEvents);
    teammate.whistle = WhistleDortmundCompressed(theWhistleDortmund);
    teammate.refereeGesture = theRefereeGesture.gesture;
    teammate.speedInfo = SpeedInfoCompressed(theSpeedInfo);
    teammate.localRobotMap = RobotMapCompressed(theLocalRobotMap);

    teamCommOutput = teammate.toTeamCommData();
    PLOT("module:TeamCommDataPacker:messageSize", teamCommOutput.data.size());
  }
}
