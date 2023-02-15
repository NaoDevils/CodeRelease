/**
 * @file TeamCommDataPacker.cpp
 * Implementation of module TeamCommDataPacker
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author Colin Graf
 */
#include "TeamCommDataPacker.h"
#include "Tools/Global.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include <iostream>

MAKE_MODULE(TeamCommDataPacker, cognitionInfrastructure)

void TeamCommDataPacker::update(TeamCommOutput& teamCommOutput)
{
  MessageQueue queue;
  queue.setSize(sizeof(RoboCup::SPLStandardMessage)); // isn't this too big?

  teamCommOutput.sendThisFrame = false;

  if (theTeamCommEvents.sendThisFrame)
  {
    const auto teamOutput = [&](MessageID id, const auto& representation)
    {
      queue.out.bin << representation;
      queue.out.finishMessage(id);
    };

    // Ordering is important here: if message is too large, data is removed from last to first!

    // Own pose information and ball observation:
    teamOutput(idTimeSynchronization, theTimeSynchronization);
    teamOutput(idRobotPose, RobotPoseCompressed(theRobotPose));
    teamOutput(idSideConfidence, theSideConfidence);
    teamOutput(idBallModel, BallModelCompressed(theBallModel));

    // Information about the behavior (i.e. the robot's state and intentions)
    teamOutput(idBehaviorData, theBehaviorData);
    teamOutput(idTeamCommEvents, theTeamCommEvents);
    teamOutput(idMotionRequest, WalkRequestCompressed(theMotionInfo.walkRequest));

    // Robot status
    //teamOutput(idRobotHealth, theRobotHealth); // Quite big and currently completely unused

    // Whistle detection state
    teamOutput(idWhistleDortmund, theWhistleDortmund);

    // Speed info for remote robot map creation
    teamOutput(idSpeedInfo, SpeedInfoCompressed(theSpeedInfo));

    // Obstacle stuff last, since size is unknown and possibly large
    teamOutput(idRobotMap, RobotMapCompressed(theRobotMap));
    teamOutput(idLocalRobotMap, LocalRobotMapCompressed(theLocalRobotMap));

    // fill SPLStandardMessage header
    teamCommOutput = fillStandardMessage(queue);
    teamCommOutput.sendThisFrame = true;
  }
}

TeamCommOutput TeamCommDataPacker::fillStandardMessage(MessageQueue& queue)
{
  TeamCommOutput message;

  // header time stamps will be filled in team handler
  OutBinarySize sizeStream;
  sizeStream << queue;
  message.numOfDataBytes = static_cast<uint16_t>(sizeStream.getSize() + sizeof(NaoDevilsHeader));
  while (message.numOfDataBytes > SPL_STANDARD_MESSAGE_DATA_SIZE)
  {
    queue.removeLastMessage();
    OutBinarySize sizeTest;
    sizeTest << queue;
    message.numOfDataBytes = static_cast<uint16_t>(sizeTest.getSize() + sizeof(NaoDevilsHeader));
    OUTPUT_ERROR("TeamCommDataPacker: TeamComm package too large: removed one message");
  }

  BH_TRACE_MSG("after remove packages stuff");
  {
    OutBinaryMemory memory(message.getNDData());
    memory << queue;
  }

  message.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  message.teamNum = static_cast<uint8_t>(theOwnTeamInfo.teamNumber);
  message.ballAge = theBallModel.timeWhenLastSeen ? static_cast<float>(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) / 1000.f : -1.f; // time in seconds
  message.ball[0] = theBallModel.estimate.position.x();
  message.ball[1] = theBallModel.estimate.position.y();
  message.pose[0] = theRobotPose.translation.x();
  message.pose[1] = theRobotPose.translation.y();
  message.pose[2] = theRobotPose.rotation;
  message.fallen = theFallDownState.state != FallDownState::upright;

  return message;
}
