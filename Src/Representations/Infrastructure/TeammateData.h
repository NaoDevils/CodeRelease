/**
 * @file TeammateData.h
 *
 * Representation of information received from my teammates
 *
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include <cstdint>
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Modeling/SimpleRobotsDistributed.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/WhistleDortmund.h"

STREAMABLE(Teammate,
  // Increase version number whenever something changes!
  static constexpr unsigned char thisVersion = 0;

  // Sum version numbers of all streamables
  static constexpr unsigned char totalVersion = thisVersion
    + TimeSynchronization::version
    + RobotPoseCompressed::version
    + BallModelCompressed::version
    + BehaviorDataCompressed::version
    + TeamCommEventsCompressed::version
    + WhistleDortmundCompressed::version
    + SpeedInfoCompressed::version
    + RobotMapCompressed::version;

  TeamCommOutput toTeamCommData() const;

  ,
  (unsigned char)(totalVersion) version,
  (unsigned char)(0) playerNumber,
  (unsigned char)(0) teamNumber,
  (unsigned)(0) sendTimestamp,
  (bool)(false) fallen, // TC: we could use soccerState instead if filled correctly at all times
  (TimeSynchronization) timeSynchronization,
  (RobotPoseCompressed) robotPose,
  (BallModelCompressed) ballModel,
  (BehaviorDataCompressed) behaviorData,
  (TeamCommEventsCompressed) teamCommEvents,
  (WhistleDortmundCompressed) whistle,
  (SpeedInfoCompressed) speedInfo,
  (RobotMapCompressed) localRobotMap
);

struct OwnTeamInfo;
struct TimeOffsets;

// Add receiveTimestamp for each sub streamable?
STREAMABLE_WITH_BASE(TeammateReceived, Teammate,
  /**
   * @enum Status
   * Activity status of the teammate.
   */
  ENUM(Status,
    INACTIVE,    /** OK   : I receive packets, but robot is penalized */
    ACTIVE,      /** GOOD : OK + Teammate is not penalized */
    FULLY_ACTIVE /** BEST : GOOD + Teammate is standing/walking :-) */
  );

  bool fromTeamCommData(const TeamCommDataReceived& teamCommData);
  void updateStatus(const OwnTeamInfo& ownTeamInfo);
  void updateTimestamps(const TimeOffsets& timeOffsets);

  ,
  (Status)(Status::INACTIVE) status,
  (unsigned)(0) receiveTimestamp,
  (std::array<uint8_t, 4>)({0}) remoteIp
);

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeammateData,
  STREAMABLE(TeammateEvent,,
    ((TeamCommEvents) SendReason)(TeamCommEvents::SendReason::numOfSendReasons) reason,
    (TeammateReceived) message
  );

  STREAMABLE(Statistic,,
    (std::array<unsigned short, MAX_NUM_PLAYERS>)({0}) players,
    (std::array<unsigned short, TeamCommEvents::SendReason::numOfSendReasons>)({0}) events,
    (std::array<unsigned short, STATE_FINISHED>)({0}) states
  );

  /** Drawing function for representation */
  void draw() const;

  TeammateReceived* getPlayer(uint8_t number);
  const TeammateReceived* getPlayer(uint8_t number) const;

  TeammateReceived* getNewestTeammate();
  const TeammateReceived* getNewestTeammate() const;

  TeammateReceived* getNewestEventMessage(TeamCommEvents::SendReason reason);
  const TeammateReceived* getNewestEventMessage(TeamCommEvents::SendReason reason) const;

  ,

  (std::vector<TeammateReceived>) teammates,        /**< An unordered(!) list of all teammates that are currently communicating with me */
  (TeammateReceived) myself,                        /**< My last teammate data */
  (int)(0) numberOfActiveTeammates,         /**< The number of teammates (in the list) that are at not INACTIVE */
  (int)(1200) messageBudget,                /**< The remaining message budget, updated by GC. */
  (float)(1.f) messageBudgetFactor,
  (bool)(true) wlanOK,                      /**< Is the wireless ok (if not, use behavior w/o wireless) */
  (std::vector<TeammateEvent>) newestEventMessages,
  (Statistic) statistic
);
