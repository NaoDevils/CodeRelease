/**
 * @file TeammateData.h
 *
 * Representation of information received from my teammates
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include <cstdint>
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Representations/MotionControl/WalkRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/SimpleRobotsDistributed.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/WhistleDortmund.h"

/**
 * @struct Teammate
 * Description of all information about/from a teammate
 */
STREAMABLE(Teammate,
  /**
   * @enum Status
   * Activity status of the teammate.
   */
  ENUM(Status,
    INACTIVE,                              /** OK   : I receive packets, but robot is penalized */
    ACTIVE,                                /** GOOD : OK + Teammate is not penalized */
    FULLY_ACTIVE                          /** BEST : GOOD + Teammate is standing/walking :-) */
  ),

  (int)(-1) number,                                   /**< The number of this player */
  (bool)(true) isNDevilsPlayer,                       /**< The name says it all */
  (bool)(true) isUpright,                             /**< The name says it all */
  (unsigned)(0) timeWhenSent,                         /**< Time when the teammate sent the message. */
  (unsigned)(0) timeWhenLastPacketReceived,           /**< Time when this player received the message. */
  (unsigned)(0) timeOfLastGroundContact,              /**< The name says it all */
  (bool)(true) hasGroundContact,                      /**< The name says it all */
  (Status)(INACTIVE) status,                          /**< @see enum \c Status */
  (RobotPose) pose,                                   /**< The pose in global field coordinates, no preview! */
  (BallModel) ball,                                   /**< Model of the ball (in coordinates relative to my teammate's pose), no preview! */
  (RobotsPercept) robotsPercept,                      /**< Unfiltered robots percept of seen teammates and opponents. */
  (RobotsPerceptUpper) robotsPerceptUpper,            /**< Unfiltered robots percept of seen teammates and opponents. */
  (LocalRobotMap) localRobotMap,                      /**< Map of locally observed obstacles in world coordinates */
  (RobotMap) robotMap,                                /**< Map of observed obstacles in world coordinates */
  (WalkRequest) walkRequest,                          /**< The name says it all. */
  (SpeedInfo) speedInfo,                              /**< The speed info to know where this robot is moving. Only speed and currentCustomStep is set at the moment!!! (event: GO 2019) */
  (float)(0.f) headPan,                               /**< for field coverage */
  (SideConfidence) sideConfidence,                    /**< The belief about playing in the correct direction */
  (BehaviorData) behaviorData,                        /**< Information about the behavior */
  (TeamCommEvents) teamCommEvents,                    /**< Message event informations */
  (WhistleDortmund) whistle,                          /**< Output of the WhistleDetector */
  (SimpleRobotsDistributed) simpleRobotsDistributed  /**< Possible robot map entries. */
);

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeammateData,
  STREAMABLE(TeammateEvent,,
    ((TeamCommEvents) SendReason)(TeamCommEvents::SendReason::numOfSendReasons) reason,
    (Teammate) message
  );
  /** Drawing function for representation */
  void draw() const;
  const Teammate* getNewestTeammate() const;
  Teammate* getNewestEventMessage(TeamCommEvents::SendReason reason);
  const Teammate* getNewestEventMessage(TeamCommEvents::SendReason reason) const,

  (std::vector<Teammate>) teammates,        /**< An unordered(!) list of all teammates that are currently communicating with me */
  (Teammate) myself,                        /**< My last teammate data */
  (int)(0) numberOfActiveTeammates,         /**< The number of teammates (in the list) that are at not INACTIVE */
  (int)(1200) messageBudget,                /**< The remaining message budget, updated by GC. */
  (float)(1.f) messageBudgetFactor,
  (bool)(true) wlanOK,                      /**< Is the wireless ok (if not, use behavior w/o wireless) */
  (std::vector<TeammateEvent>) newestEventMessages
);
