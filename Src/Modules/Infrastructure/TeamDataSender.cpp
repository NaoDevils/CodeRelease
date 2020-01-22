/**
* @file TeamDataSender.cpp
* Implementation of module TeamDataSender
* @author Colin Graf
*/
#include "TeamDataSender.h"
#include "Tools/ProcessFramework/TeamHandler.h"
#include "Tools/Global.h"
#include "Representations/Infrastructure/DevilSmashStandardMessage.h"
#include "Tools/Network/NTP.h"
#include <iostream>

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type,format,expression) \
{ out.format << expression;\
  out.finishMessage(type);\
} \


MAKE_MODULE(TeamDataSender, cognitionInfrastructure)

void TeamDataSender::update(TeamDataSenderOutput& teamDataSenderOutput)
{
  if (!outMessage)
  {
    outMessage = new MessageQueue();
    outMessage->setSize(sizeof(RoboCup::SPLStandardMessage));
  }
  DEBUG_RESPONSE_ONCE("module:TeamDataSender:ntpOffsets")
  {
    for (int offset : Global::getNTP().getOffsets())
    {
      OUTPUT_TEXT("NTP offset " << offset);
    }
  }
  DEBUG_RESPONSE_ONCE("module:TeamDataSender:ntpRTTs")
  {
    for (int rtt : Global::getNTP().getRTTs())
    {
      OUTPUT_TEXT("NTP RTT " << rtt);
    }
  }
  if(theTeammateData.sendThisFrame)
  {
    ++sendFrames;
    if(!outMessage->isEmpty()) outMessage->clear();
    OutMessage& out = outMessage->out;

    // Ordering is important here: if message is too large, data is removed from last to first!

    // Own pose information and ball observation:
    TEAM_OUTPUT(idRobotPose, bin, RobotPoseCompressed(theRobotPose));
    TEAM_OUTPUT(idSideConfidence, bin, theSideConfidence);
    TEAM_OUTPUT(idBallModel, bin, BallModelCompressed(theBallModel));

        
    // Information about the behavior (i.e. the robot's state and intentions)
    TEAM_OUTPUT(idBehaviorData, bin, theBehaviorData);
    TEAM_OUTPUT(idMotionRequest, bin, WalkRequestCompressed(theMotionInfo.walkRequest));

    // Robot status
    TEAM_OUTPUT(idRobotHealth, bin, theRobotHealth);
    TEAM_OUTPUT(idTeammateIsPenalized, bin, (theRobotInfo.penalty != PENALTY_NONE));
    
    TEAM_OUTPUT(idWhistleDortmund, bin, DistributedWhistleDortmund(theWhistleDortmund, theGameInfo.whistleCausedPlay));
    //TEAM_OUTPUT(idWhistle, bin, theGameInfo.whistleCausedPlay); Added to WhistleDortmund

    // Speed info for remote robot map creation
    TEAM_OUTPUT(idSpeedInfo, bin, SpeedInfoCompressed(theSpeedInfo));
    
    // Obstacle stuff last, since size is unknown and possibly large
    TEAM_OUTPUT(idRobotMap, bin, RobotMapCompressed(theRobotMap));
    TEAM_OUTPUT(idLocalRobotMap, bin, LocalRobotMapCompressed(theLocalRobotMap));

    // fill SPLStandardMessage header
    fillStandardMessage();
  }
}

void TeamDataSender::fillStandardMessage()
{
  RoboCup::SPLStandardMessage& message = Global::getTeamOut().message;

  size_t dataOffset = 0;
  if (theGameInfo.competitionType == COMPETITION_TYPE_MIXEDTEAM || Global::getSettings().gameMode == Settings::mixedTeam)
  {
    fillMixedTeamMessage(message);
    dataOffset = dsmSize;
    BH_TRACE_MSG("after fill mixed team stuff");
  }

  NDevilsHeader& header = (NDevilsHeader&) * (&message.data[dataOffset]);
  header.teamName[0] = 'N';
  header.teamName[1] = 'D';
  header.teamName[2] = '0';
  header.teamName[3] = '8';
  header.version = NDEVILS_TC_VERSION;
  // header time stamps will be filled in team handler
  OutBinarySize sizeStream;
  sizeStream << *outMessage;
  message.numOfDataBytes = static_cast<uint16_t>(sizeStream.getSize() + ndevilsHeaderSize + dataOffset);
  while (message.numOfDataBytes > (SPL_STANDARD_MESSAGE_DATA_SIZE - teamCommHeaderSize))
  {
    outMessage->removeLastMessage();
    OutBinarySize sizeTest;
    sizeTest << *outMessage;
    message.numOfDataBytes = static_cast<uint16_t>(sizeTest.getSize() + ndevilsHeaderSize + dataOffset);
    OUTPUT_ERROR("TeamDataSender: TeamComm package too large: removed one message");
  }
  BH_TRACE_MSG("after remove packages stuff");
  {
    OutBinaryMemory memory(message.data + ndevilsHeaderSize + dataOffset);
    memory << *outMessage;
  }
  /*else
  {
    message.numOfDataBytes = static_cast<uint16_t>(ndevilsHeaderSize);
    OUTPUT_ERROR("SPL_STANDARD_MESSAGE_DATA_SIZE exceeded!");
    ASSERT(false);
  }*/
  
  message.playerNum = static_cast<uint8_t>(Global::getSettings().playerNumber);
  message.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  message.ballAge = theBallModel.timeWhenLastSeen ? static_cast<float>(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) / 1000.f : -1.f; // time in seconds
  message.ball[0] = theBallModel.estimate.position.x();
  message.ball[1] = theBallModel.estimate.position.y();
  message.pose[0] = theRobotPose.translation.x();
  message.pose[1] = theRobotPose.translation.y();
  message.pose[2] = theRobotPose.rotation;
  message.fallen = theFallDownState.state != FallDownState::upright;
}

void TeamDataSender::fillMixedTeamMessage(RoboCup::SPLStandardMessage& message)
{
  DevilSmash::StandardMessage dsm;
  dsmSize = 0;
  
  // ball
  dsm.ballValidity = theBallModel.validity;
  dsm.ballVelocity[0] = theBallModel.estimate.velocity.x();
  dsm.ballVelocity[1] = theBallModel.estimate.velocity.y();
  dsm.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;


  // sync
  dsm.timestamp = SystemCall::getCurrentSystemTime();
  dsm.timestampLastJumped = 0;
  dsm.lastTimeWhistleDetected = theWhistleDortmund.lastDetectionTime;
  dsm.ntpMessages.clear();
  const std::array<NTPData, MAX_NUM_PLAYERS>& ntpResponses = Global::getNTP().getNTPResponses();
  for(size_t i=0;i<ntpResponses.size();i++)
  {
    const NTPData& ntpResponse = ntpResponses[i];

    if ((int)i == Global::getSettings().playerNumber-1)
      continue;
    if (ntpResponse.receivedTimeStamp == 0 || ntpResponse.sentTimeStamp == 0)
      continue;

    DevilSmash::NTPMessage ntpMessage;
    ntpMessage.receiver = static_cast<uint8_t>(i+1);
    ntpMessage.requestOrigination = ntpResponse.sentTimeStamp;
    ntpMessage.requestReceipt = ntpResponse.receivedTimeStamp;
    dsm.ntpMessages.push_back(ntpMessage);
  }
  dsm.requestsNTPMessage = true;

  // rest
  dsm.member = DEVIL_MEMBER;
  dsm.headYawAngle = 0.f;
  dsm.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  dsm.isRobotPoseValid = theRobotPose.validity > 0.6f;

  dsm.write(message.data);
  dsmSize = dsm.sizeOfDSMessage();
}

TeamDataSender::~TeamDataSender()
{
  if (outMessage) delete outMessage;
}
