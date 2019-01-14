/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/
#include "TeamDataSender.h"
#include "Tools/ProcessFramework/TeamHandler.h"
#include "Tools/Global.h"

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type,format,expression) \
{ Global::getTeamOut().out.format << expression;\
Global::getTeamOut().out.finishMessage(type); }


MAKE_MODULE(TeamDataSender, cognitionInfrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if(theTeammateData.sendThisFrame)
  {
    ++sendFrames;

    // Own pose information and ball observation:
    TEAM_OUTPUT(idRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idBallModel, bin, BallModelCompressed(theBallModel));
    //TEAM_OUTPUT(idGoalPercept, bin, theGoalPercept);

    // Obstacle stuff
    TEAM_OUTPUT(idRobotsPercept, bin, RobotsPerceptCompressed(theRobotsPercept));
    TEAM_OUTPUT(idRobotMap, bin, RobotMapCompressed(theRobotMap));
    //TEAM_OUTPUT(idSimpleRobotsDistributed, bin, theSimpleRobotsDistributed);

    // Information about the behavior (i.e. the robot's state and intentions)
    TEAM_OUTPUT(idBehaviorData, bin, theBehaviorData);
    TEAM_OUTPUT(idMotionRequest, bin, WalkRequestCompressed(theMotionInfo.walkRequest));

    // Robot status
    TEAM_OUTPUT(idTeammateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    TEAM_OUTPUT(idTeammateIsUpright, bin, (theFallDownState.state == theFallDownState.upright));
    if(theGroundContactState.contact)
    {
      TEAM_OUTPUT(idTeammateTimeOfLastGroundContact, bin, theFrameInfo.time);
    }

    TEAM_OUTPUT(idWhistleDortmund, bin, theWhistleDortmund);
    TEAM_OUTPUT(idWhistle, bin, theGameInfo.whistleCausedPlay);
    
    if(sendFrames % 20 == 0)
      TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);

    // fill SPLStandardMessage header
    fillStandardMessage();
  }
}

void TeamDataSender::fillStandardMessage()
{
  RoboCup::SPLStandardMessage& message = Global::getTeamOut().message;

  NDevilsHeader& header = (NDevilsHeader&)*message.data;
  header.teamID = 12;
#ifdef TARGET_ROBOT
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  header.timeStampSent = static_cast<uint64_t>(time);
#else
  header.timeStampSent = static_cast<uint64_t>(SystemCall::getCurrentSystemTime());
#endif
  header.intention = (theBehaviorData.soccerState == BehaviorData::controlBall) ? 3 : 0;
  header.isPenalized = (theRobotInfo.penalty != PENALTY_NONE);
  header.whistleDetected = theWhistleDortmund.detected;
  OutBinarySize sizeStream;
  sizeStream << Global::getTeamOut().queue;
  message.numOfDataBytes = static_cast<uint16_t>(sizeStream.getSize() + ndevilsHeaderSize);

  if (message.numOfDataBytes <= SPL_STANDARD_MESSAGE_DATA_SIZE)
  {
    OutBinaryMemory memory(message.data + ndevilsHeaderSize);
    memory << Global::getTeamOut().queue;
  }
  else
  {
    message.numOfDataBytes = static_cast<uint16_t>(ndevilsHeaderSize);
    OUTPUT_ERROR("SPL_STANDARD_MESSAGE_DATA_SIZE exceeded!");
    ASSERT(false);
  }
  
  message.playerNum = static_cast<int8_t>(theRobotInfo.number);
  message.teamNum = theOwnTeamInfo.teamNumber;
  message.ballAge = theBallModel.timeWhenLastSeen ? static_cast<float>(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) / 1000.f : -1.f; // time in seconds
  message.ball[0] = theBallModel.estimate.position.x();
  message.ball[1] = theBallModel.estimate.position.y();
  message.pose[0] = theRobotPose.translation.x();
  message.pose[1] = theRobotPose.translation.y();
  message.pose[2] = theRobotPose.rotation;
  message.fallen = theFallDownState.state != FallDownState::upright;
}
