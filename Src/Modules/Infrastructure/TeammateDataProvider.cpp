/**
 * @file Modules/Infrastructure/TeammateDataProvider.cpp
 * This file implements a temporary wrapper from the old to the new TeammateData structure.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "TeammateDataProvider.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Infrastructure/DevilSmashStandardMessage.h"
#include "Tools/Network/NTP.h"

#include <iostream>

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * member.
 */
#define UNPACK(representation, member) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  currentTeammate->member = the##representation##Compressed;

PROCESS_LOCAL TeammateDataProvider* TeammateDataProvider::theInstance = 0;

TeammateDataProvider::TeammateDataProvider() : lastSentTimestamp(0), lastReceivedTimestamp(0),
                                               currentTeammate(0), teammateDataPtr(0), wlanQuality(1.f)
{
  theInstance = this;
}

TeammateDataProvider::~TeammateDataProvider()
{
  theInstance = 0;
}

void TeammateDataProvider::update(TeammateData& teammateData)
{
  // Initialize:
  teammateDataPtr = &teammateData;

  // Iterate over deprecated list of teammate information and update some convenience information
  // (new information has already been coming via handleMessages)
  for(auto & teammate : teammateData.teammates)
  {
    if(teammate.behaviorData.soccerState == BehaviorData::penalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE || teammate.isPenalized)
      teammate.status = Teammate::INACTIVE;
    else if(!teammate.isUpright)
      teammate.status = Teammate::ACTIVE;
    else
      teammate.status = Teammate::FULLY_ACTIVE;
    teammate.isGoalkeeper = teammate.number == 1;
    teammate.sanity = std::max(0.f, teammate.sanity - 0.0025f);
  }

  // Remove elements that are too old:
  auto teammate = teammateData.teammates.begin();
  while(teammate != teammateData.teammates.end())
  {
    if(theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
      teammate = teammateData.teammates.erase(teammate);
    else
      ++teammate;
  }

  // Other stuff
  teammateData.numberOfActiveTeammates = 0;
  teammate = teammateData.teammates.begin();
  while(teammate != teammateData.teammates.end())
  {
    if(teammate->status != Teammate::INACTIVE)
      teammateData.numberOfActiveTeammates++;
    ++teammate;
  }

  // Sending interval and NTP synchronization:
  teammateData.sendThisFrame = 
#ifdef TARGET_ROBOT
    theRobotInfo.transitionToBhuman == 1.f &&
#endif
    theFrameInfo.getTimeSince(lastSentTimestamp) >= sendInterval;
  if(teammateData.sendThisFrame)
  {
    // Check, if NTP has to respond
    //ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut());
    // Update timestamp
    if(theFrameInfo.getTimeSince(lastSentTimestamp) >= 2 * sendInterval)
      lastSentTimestamp = theFrameInfo.time;
    else
      lastSentTimestamp += sendInterval;
  }

  // check wlanQuality
  if (wlanQuality > 0.7f)
    teammateData.wlanOK = true;
  else if (wlanQuality < 0.5f)
    teammateData.wlanOK = false;
}

void TeammateDataProvider::setCurrentTeammate(int robotNumber)
{
  // The robot itself is not a valid teammate.
  // Numbers outside the predefined scope are not valid, too.
  if((robotNumber == theRobotInfo.number) || (robotNumber < Global::getSettings().lowestValidPlayerNumber) ||
     (robotNumber > Global::getSettings().highestValidPlayerNumber))
  {
    currentTeammate = 0;
    return;
  }
  // do not use package if it is too old (-> slow wireless), does not work with drop in players
  Global::getNTP().convertRemoteTimeInLocalTime(currentTeammateSentTimestamp, robotNumber);
  if (std::abs(static_cast<int64_t>(theFrameInfo.time) - static_cast<int64_t>(currentTeammateSentTimestamp)) > badWifiTimeout)
  {
    currentTeammate = 0;
    wlanQuality = std::max(0.f, wlanQuality - 0.2f);
    return;
  }
  else
    wlanQuality = std::min(1.f,wlanQuality + 0.1f);
  // Try to find the robot in the current list of robots:
  for(auto& teammate : (*teammateDataPtr).teammates)
  {
    if(teammate.number == robotNumber)
    {
      currentTeammate = &teammate;
      return;
    }
  }
  // This seems to be a new robot that is not part of the list yet:
  Teammate newTeammate;
  newTeammate.number = robotNumber;
  teammateDataPtr->teammates.push_back(newTeammate);
  currentTeammate = &(teammateDataPtr->teammates[teammateDataPtr->teammates.size()-1]);
}

void TeammateDataProvider::handleMessages(TeamDataIn& teamReceiver)
{
  // set data from standard message
  if (theInstance)
  {
    for (std::vector<RoboCup::SPLStandardMessage>::const_iterator i = teamReceiver.messages.begin(); i != teamReceiver.messages.end(); ++i)
    {
      theInstance->handleNTPMessage((*i));

      theInstance->handleStandardMessage((*i));
      if (!theInstance->currentTeammate)
        continue;
      // create queue and handle the streamed data part
      MessageQueue queue;
      queue.setSize(sizeof(RoboCup::SPLStandardMessage));
      bool mixedTeam = (Global::getSettings().gameMode == Settings::mixedTeam || theInstance->theGameInfo.competitionType == COMPETITION_TYPE_MIXEDTEAM);

      size_t dataOffset = 0;
      bool isNDevilsPlayer = true;
      if (mixedTeam)
      {
        DevilSmash::StandardMessage dsm;
        dsm.read(i->data);
        if (dsm.member != DEVIL_MEMBER)
        {
          isNDevilsPlayer = false;
          theInstance->handleMixedTeamPackage(*i);
        }
        dataOffset = dsm.sizeOfDSMessage();
      }

      theInstance->currentTeammate->timeWhenLastPacketReceived = theInstance->lastReceivedTimestamp;
      theInstance->currentTeammate->timeWhenSent = theInstance->currentTeammateSentTimestamp;
      theInstance->currentTeammate->isNDevilsPlayer = isNDevilsPlayer;
      if (isNDevilsPlayer)
      {
        InBinaryMemory memory((*i).data + dataOffset + ndevilsHeaderSize, static_cast<size_t>((int)((*i).numOfDataBytes) - dataOffset - ndevilsHeaderSize));
        memory >> queue;
        queue.handleAllMessages(*theInstance);
      }
    }
    
  }
  teamReceiver.clear();
}

bool TeammateDataProvider::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    // Robot status:
    case idTeammateIsPenalized:
      if(currentTeammate)
        message.bin >> currentTeammate->isPenalized;
      return true;
    // "Normal" representations:
    case idRobotPose:
      if (currentTeammate)
      {
        UNPACK(RobotPose, pose);
      }
      return true;
    case idWhistleDortmund:
      if(currentTeammate)
      {
        DistributedWhistleDortmund distributedWhistle;
        message.bin >> distributedWhistle;
        currentTeammate->whistle = distributedWhistle.whistle;
        currentTeammate->whistleCausedPlay = distributedWhistle.whistleCausedPlay;
        Global::getNTP().convertRemoteTimeInLocalTime(currentTeammate->whistle.lastDetectionTime, currentTeammate->number);
      }
      return true;
    case idBallModel:
      if(currentTeammate)
      {
        BallModel lastBallModel = currentTeammate->ball;
        UNPACK(BallModel, ball);
        Global::getNTP().convertRemoteTimeInLocalTime(currentTeammate->ball.timeWhenLastSeen, currentTeammate->number);
        Global::getNTP().convertRemoteTimeInLocalTime(currentTeammate->ball.timeWhenDisappeared, currentTeammate->number);
        if(!currentTeammate->isNDevilsPlayer && (currentTeammate->ball.timeWhenLastSeen - lastBallModel.timeWhenLastSeen) < 16)
          currentTeammate->ball.lastPerception = lastBallModel.lastPerception;
      }
      return true;
    case idRobotsPercept:
      if (currentTeammate)
      {
        UNPACK(RobotsPercept, robotsPercept);
      }
      return true;
    case idRobotsPerceptUpper:
      if (currentTeammate)
      {
        UNPACK(RobotsPerceptUpper, robotsPerceptUpper);
      }
      return true;
    case idRobotMap:
      if (currentTeammate)
      { 
        UNPACK(RobotMap, robotMap);
      }
      return true;
    case idLocalRobotMap:
      if (currentTeammate)
      { 
        UNPACK(LocalRobotMap, localRobotMap);
      }
      return true;
    case idSideConfidence:
      if(currentTeammate)
        message.bin >> currentTeammate->sideConfidence;
      return true;
    case idBehaviorData:
      if(currentTeammate)
        message.bin >> currentTeammate->behaviorData;
      return true;
    case idSimpleRobotsDistributed:
      if(currentTeammate)
        message.bin >> currentTeammate->simpleRobotsDistributed;
      return true;
    case idMotionRequest:
      if (currentTeammate)
      {
        UNPACK(WalkRequest, walkRequest);
      }
      return true;
    case idSpeedInfo:
      if (currentTeammate)
      {
        UNPACK(SpeedInfo, speedInfo);
      }
      return true;
    default:
      return true;
  }
}

void TeammateDataProvider::handleMixedTeamPackage(const RoboCup::SPLStandardMessage &msg)
{
  // TODO: check all this
  if (!currentTeammate)
    return;
  DevilSmash::StandardMessage dsm;
  if (!dsm.read(msg.data))
  {
    OUTPUT_WARNING("DevilSMASH message invalid!");
    return;
  }
  
  currentTeammate->pose.translation.x() = msg.pose[0];
  currentTeammate->pose.translation.y() = msg.pose[1];
  currentTeammate->pose.rotation = msg.pose[2];
  currentTeammate->pose.validity = dsm.isRobotPoseValid ? 0.8f : 0.f;
  currentTeammate->ball.estimate.position.x() = msg.ball[0];
  currentTeammate->ball.estimate.position.y() = msg.ball[1];
  currentTeammate->ball.estimate.velocity.x() = dsm.ballVelocity[0];
  currentTeammate->ball.estimate.velocity.y() = dsm.ballVelocity[1];
  currentTeammate->ball.timeWhenLastSeen = dsm.timeWhenBallLastSeen;
  currentTeammate->ball.timeWhenDisappeared = dsm.timeWhenBallLastSeen;
  Global::getNTP().convertRemoteTimeInLocalTime(currentTeammate->ball.timeWhenLastSeen, currentTeammate->number);
  Global::getNTP().convertRemoteTimeInLocalTime(currentTeammate->ball.timeWhenDisappeared, currentTeammate->number);
  currentTeammate->ball.validity = dsm.ballValidity;
  currentTeammate->ball.lastPerception = currentTeammate->ball.estimate.position;
  currentTeammate->isPenalized = dsm.isPenalized;
  currentTeammate->isNDevilsPlayer = false;
  currentTeammate->hasGroundContact = true;
  currentTeammate->behaviorData.ballPositionField = Transformation::robotToField(currentTeammate->pose, currentTeammate->ball.estimate.position).cast<short>();
  currentTeammate->behaviorData.ballPositionFieldPredicted = currentTeammate->behaviorData.ballPositionField.cast<short>();
  currentTeammate->behaviorData.ballPositionRelative = currentTeammate->ball.estimate.position.cast<short>();
  currentTeammate->behaviorData.lastRole = currentTeammate->behaviorData.role;
  currentTeammate->behaviorData.timeSinceBallWasSeen = (msg.ballAge > 0) ? static_cast<int>(msg.ballAge * 1000) : 1000000;
  currentTeammate->headPan = dsm.headYawAngle;
  currentTeammate->isGoalkeeper = dsm.currentlyPerformingRole == DevilSmash::Role::KEEPER;
  currentTeammate->sanity = 1.f;
  currentTeammate->sideConfidence.confidenceState = SideConfidence::CONFIDENT;
  currentTeammate->status = dsm.isPenalized ? Teammate::INACTIVE : (msg.fallen ? Teammate::ACTIVE : Teammate::FULLY_ACTIVE);
  currentTeammate->whistle.lastDetectionTime = dsm.lastTimeWhistleDetected;
  currentTeammate->whistle.detectionState =
    (theFrameInfo.getTimeSince(currentTeammate->whistle.lastDetectionTime) < 1000) ?
    WhistleDortmund::DetectionState::isDetected : WhistleDortmund::DetectionState::notDetected;
}

// Only sets upright and pose information, since ball information is in ball model.
// Could be merged in the future.
void TeammateDataProvider::handleStandardMessage(const RoboCup::SPLStandardMessage& msg)
{
  currentTeammate = 0;
  if (msg.teamNum == theOwnTeamInfo.teamNumber)
    setCurrentTeammate(msg.playerNum);
  if (!currentTeammate)
    return;
  currentTeammate->isUpright = !msg.fallen;
  currentTeammate->pose.translation.x() = msg.pose[0];
  currentTeammate->pose.translation.y() = msg.pose[1];
  currentTeammate->pose.rotation = Angle::normalize(msg.pose[2]);
}

void TeammateDataProvider::handleNTPMessage(const RoboCup::SPLStandardMessage& msg)
{
  bool mixedTeam = (Global::getSettings().gameMode == Settings::mixedTeam || theInstance->theGameInfo.competitionType == COMPETITION_TYPE_MIXEDTEAM);

  size_t dataOffset = 0;

  if (mixedTeam)
  {
    DevilSmash::StandardMessage dsm;
    dsm.read(msg.data);

    dataOffset = dsm.sizeOfDSMessage();
    const NDevilsHeader& header = (const NDevilsHeader&)(msg).data[dataOffset];

    theInstance->lastReceivedTimestamp = header.ntpResponses[msg.playerNum - 1].receivedTimeStamp;
    theInstance->currentTeammateSentTimestamp = dsm.timestamp;
    if (dsm.member != DEVIL_MEMBER)
    {
      // get ntp stuff from HULKS
      for (auto ntpMessage : dsm.ntpMessages)
      {
        if (ntpMessage.receiver == Global::getSettings().playerNumber)
        {
          Global::getNTP().receivedResponse(
            msg.playerNum,
            { ntpMessage.requestOrigination, ntpMessage.requestReceipt },
            { dsm.timestamp, theInstance->lastReceivedTimestamp }
          );
        }
      }

      //if (dsm.requestsNTPMessage)
      Global::getNTP().sendResponse(
        msg.playerNum,
        { dsm.timestamp, theInstance->lastReceivedTimestamp }
      );
      return;
    }
  }
  else
  {
    const NDevilsHeader& header = (const NDevilsHeader&)(msg).data[dataOffset];
    theInstance->lastReceivedTimestamp = header.ntpResponses[msg.playerNum - 1].receivedTimeStamp;
    theInstance->currentTeammateSentTimestamp = header.ntpResponses[msg.playerNum - 1].sentTimeStamp;
  }


}

MAKE_MODULE(TeammateDataProvider, cognitionInfrastructure)
