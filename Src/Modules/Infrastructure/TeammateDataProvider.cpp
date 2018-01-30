/**
 * @file Modules/Infrastructure/TeammateDataProvider.cpp
 * This file implements a temporary wrapper from the old to the new TeammateData structure.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "TeammateDataProvider.h"
#include "Tools/Math/Transformation.h"

/**
 * This macro converts a timestamp (from another robot) into local time via NTP.
 */
#define REMOTE_TO_LOCAL_TIME(timestamp, received) \
  if(currentTeammate->isNDevilsPlayer && timestamp) \
    timestamp = ntp.getRemoteTimeInLocalTime(timestamp, received);

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
    if(teammate.behaviorData.soccerState == BehaviorData::penalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
      teammate.status = Teammate::INACTIVE;
    else if(!teammate.isUpright)
      teammate.status = Teammate::ACTIVE;
    else
      teammate.status = Teammate::FULLY_ACTIVE;
    teammate.isGoalkeeper = teammate.number == 1;
    teammate.sanity = std::max(0.f, teammate.sanity - 0.005f);
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
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
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
  currentTeammateSentTimestamp = ntp.getRemoteTimeInLocalTime(currentTeammateSentTimestamp, lastReceivedTimestamp);
  if (theFrameInfo.getTimeSince(currentTeammateSentTimestamp) > badWifiTimeout)
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
  if(theInstance)
    teamReceiver.queue.handleAllMessages(*theInstance);
  if (theInstance && theInstance->currentTeammate && !(theInstance->currentTeammate->isNDevilsPlayer) &&
    (Global::getSettings().gameMode == Settings::mixedTeam
    || theInstance->theGameInfo.gameType == GAME_MIXEDTEAM_PLAYOFF
    || theInstance->theGameInfo.gameType == GAME_MIXEDTEAM_ROUNDROBIN))
  {
    for (std::vector<RoboCup::SPLStandardMessage>::const_iterator i = teamReceiver.messages.begin(); i != teamReceiver.messages.end(); ++i)
    {
      theInstance->handleMixedTeamPackage((*i));
    }
  }
  teamReceiver.clear();
}

bool TeammateDataProvider::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    // NDevils Header - TODO: remoteIp not used atm (could be checked?)
    case idNTPHeader:
    {
      int robotNumber;
      message.bin >> robotNumber;
      ntp.setCurrentRemoteID(robotNumber);
      message.bin >> currentTeammateSentTimestamp;
      message.bin >> lastReceivedTimestamp;
      setCurrentTeammate(robotNumber);
      unsigned char teamID = 12;
      message.bin >> teamID;
      if (currentTeammate)
      {
        currentTeammate->timeWhenLastPacketReceived = lastReceivedTimestamp;
        message.bin >> currentTeammate->isPenalized;
        message.bin >> currentTeammate->whistle.detected;
        currentTeammate->isNDevilsPlayer = (teamID == 12);
        if (!currentTeammate->isNDevilsPlayer)
        {
          currentTeammate->whistle.detected = false;
          currentTeammate->whistleCausedPlay = false;
        }
        currentTeammate->timeWhenSent = currentTeammateSentTimestamp;
      }
      else
      {
        // dummy
        message.bin >> teamID;
        message.bin >> teamID;
      }
      return true;
    }
    // Robot status:
    case idTeammateIsPenalized:
      if(currentTeammate)
          message.bin >> currentTeammate->isPenalized;
      return true;
    case idTeammateHasGroundContact:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->hasGroundContact;
        // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of team mates
        // at many different places in our code! For a detailed problem description, ask Tim.
        if(!currentTeammate->hasGroundContact && currentTeammate->isNDevilsPlayer)
          currentTeammate->hasGroundContact = theFrameInfo.getTimeSince(currentTeammate->timeOfLastGroundContact) < 2000;
      }
      return true;
    case idTeammateIsUpright:
      if(currentTeammate)
        message.bin >> currentTeammate->isUpright;
      return true;
    case idTeammateTimeOfLastGroundContact:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->timeOfLastGroundContact;
        REMOTE_TO_LOCAL_TIME(currentTeammate->timeOfLastGroundContact, lastReceivedTimestamp);
      }
      return true;
    // "Normal" representations:
    case idWhistle:
      if (currentTeammate)
      {
        message.bin >> currentTeammate->whistleCausedPlay;
      }
      return true;
    case idWhistleDortmund:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->whistle;
      }
      return true;
    case idBallModel:
      if(currentTeammate)
      {
        BallModel lastBallModel = currentTeammate->ball;
        UNPACK(BallModel, ball);
        REMOTE_TO_LOCAL_TIME(currentTeammate->ball.timeWhenLastSeen, lastReceivedTimestamp);
        REMOTE_TO_LOCAL_TIME(currentTeammate->ball.timeWhenDisappeared, lastReceivedTimestamp);
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
    case idRobotMap:
      if (currentTeammate)
      { 
        UNPACK(RobotMap, robotMap);
      }
      return true;
    case idRobotPose:
      if(currentTeammate)
      {
        UNPACK(RobotPose, pose);
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
    default:
      return true;
  }
}

void TeammateDataProvider::handleMixedTeamPackage(const RoboCup::SPLStandardMessage &msg)
{
  currentTeammate = 0;
  if (msg.teamNum == theOwnTeamInfo.teamNumber)
    setCurrentTeammate(msg.playerNum);
  if (!currentTeammate)
    return;
  currentTeammate->timeWhenLastPacketReceived = theFrameInfo.time;
  currentTeammate->status = Teammate::INACTIVE; // default -> inactive
  currentTeammate->pose.translation.x() = msg.pose[0];
  currentTeammate->pose.translation.y() = msg.pose[1];
  currentTeammate->pose.rotation = Angle::normalize(msg.pose[2]);
  currentTeammate->ball.estimate.position.x() = msg.ball[0];
  currentTeammate->ball.estimate.position.y() = msg.ball[1];
  currentTeammate->ball.estimate.velocity.x() = msg.ballVel[0];
  currentTeammate->ball.estimate.velocity.y() = msg.ballVel[1];
  currentTeammate->ball.validity = 0.5f; // TODO
  currentTeammate->behaviorData.timeSinceBallWasSeen = static_cast<int>(msg.ballAge * 1000.f);
  if (currentTeammate->behaviorData.timeSinceBallWasSeen >= 0)
  {
    currentTeammate->behaviorData.ballPositionRelative.x() = static_cast<short>(msg.ball[0]);
    currentTeammate->behaviorData.ballPositionRelative.y() = static_cast<short>(msg.ball[1]);
    currentTeammate->behaviorData.ballPositionField = Transformation::robotToField(currentTeammate->pose, currentTeammate->ball.estimate.position).cast<short>();
    currentTeammate->behaviorData.ballPositionFieldPredicted = currentTeammate->behaviorData.ballPositionField;
    currentTeammate->ball.timeWhenLastSeenByTeamMate = SystemCall::getCurrentSystemTime();
  }
  else
    currentTeammate->behaviorData.timeSinceBallWasSeen = 100000;
  currentTeammate->ball.timeWhenLastSeen = currentTeammateSentTimestamp - currentTeammate->behaviorData.timeSinceBallWasSeen;
  currentTeammate->ball.lastPerception = currentTeammate->ball.estimate.position;
  
  currentTeammate->isUpright = !msg.fallen;
  currentTeammate->behaviorData.soccerState = BehaviorData::positioning;
  // sanity checks for other drop in players
  if (!theFieldDimensions.isInsideCarpet(currentTeammate->pose.translation) ||
    theFieldDimensions.isInsideCarpet(currentTeammate->ball.estimate.position) ||
    currentTeammate->ball.estimate.velocity.norm() > 1000 ||
    msg.currentSideConfidence > 100 ||
    msg.currentSideConfidence < 0 ||
    msg.currentPositionConfidence > 100 ||
    msg.currentPositionConfidence < 0 ||
    msg.numOfDataBytes > SPL_STANDARD_MESSAGE_DATA_SIZE ||
    msg.version != SPL_STANDARD_MESSAGE_STRUCT_VERSION ||
    !theFieldDimensions.isInsideCarpet(Vector2f(msg.walkingTo[0], msg.walkingTo[1])))
  {
    currentTeammate->sanity = 0.f;
    return;
  }
  // update sanity value
  // TODO: need local ball model here for comparison!!
  if ((Transformation::robotToField(currentTeammate->pose, currentTeammate->ball.estimate.position) -
    Transformation::robotToField(theRobotPose, theBallModel.estimate.position)).norm() < 500.f)
  {
    float oldSanity = currentTeammate->sanity;
    currentTeammate->sanity += 0.1f;
    if (oldSanity < minSanityForTeammates && currentTeammate->sanity > minSanityForTeammates)
      currentTeammate->sanity += 0.1f; //hysteresis
  }
  if (currentTeammate->sanity > minSanityForTeammates && 
    msg.currentPositionConfidence > 50 && msg.currentSideConfidence > 50)
  {
    currentTeammate->status = msg.fallen ? Teammate::ACTIVE : Teammate::FULLY_ACTIVE;
    currentTeammate->pose.validity = msg.currentPositionConfidence / 100.f;
    currentTeammate->behaviorData.soccerState = BehaviorData::positioning;
    switch (msg.intention)
    {
    case 1:
      currentTeammate->behaviorData.role = BehaviorData::keeper;
      break;
    case 2:
      currentTeammate->behaviorData.role = BehaviorData::defender;
      break;
    case 3:
      currentTeammate->behaviorData.role = BehaviorData::striker;
      if (currentTeammate->behaviorData.timeSinceBallWasSeen < 5000)
        currentTeammate->behaviorData.soccerState = BehaviorData::controlBall;
      break;
    default:
      currentTeammate->behaviorData.role = BehaviorData::supporterDef;
    }
    currentTeammate->sideConfidence.sideConfidence = msg.currentSideConfidence / 100.f;
    if (theOwnTeamInfo.players[msg.playerNum].penalty != PENALTY_NONE)
      currentTeammate->behaviorData.soccerState = BehaviorData::penalized;
  }
}

MAKE_MODULE(TeammateDataProvider, cognitionInfrastructure)
