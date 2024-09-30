/**
 * @file EventManager.h
 * This module handles the events leading to a sent package.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Representations/Infrastructure/TeamCommSenderOutput.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/RefereeGesture.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/MotionControl/SpeedInfo.h"


MODULE(EventManager,
  REQUIRES(BallSymbols),
  REQUIRES(BallModel),
  REQUIRES(BehaviorData),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSymbols),
  REQUIRES(SideConfidence),
  REQUIRES(TeammateData),
  REQUIRES(TimeSynchronization),
  REQUIRES(BallChaserDecision),
  REQUIRES(TacticSymbols),
  REQUIRES(RefereeGesture),
  REQUIRES(VisualRefereeBehaviorSymbols),
  REQUIRES(SpeedInfo),
  USES(TeamCommSenderOutput),
  PROVIDES(TeamCommEvents),
  LOADS_PARAMETERS(
    STREAMABLE(EventInterval,,
      ((TeamCommEvents) SendReason) reason,
      (bool)(false) perTeam,
      (unsigned)(1000) interval,
      (unsigned)(1) limit
    );
    STREAMABLE(EventConfig,,
      (float)(1500.f) playerMovedEventDistance,
      (float)(500.f) playerMovedEventDistanceForBallchaser,
      (float)(500.f) playerMovedEventDistanceForNearBall,
      (float)(1500.f) playerMovedNearBallDistance,
      (float)(200.f) playerMovedEventDistanceWhenSlow,
      (float)(50.f) playerMovedEventSlowTransSpeed,
      (Angle)(20_deg) playerMovedEventSlowRotSpeed,
      (unsigned)(5000) playerMovedEventIntervalInitial,
      (unsigned)(1000) playerMovedEventIntervalGoalArea,
      (float)(0.7f) playerMovedEventMinPoseValidity,
      (float)(0.4f) playerMovedEventMinPoseValidityInitial,
      (unsigned)(20000) goalDetectedMinTimeDiff,
      (float)(1000.f) ballMovedEventDistance,
      (float)(0.8f) ballMovedMinValidity
    );

    const EventInterval* getInterval(TeamCommEvents::SendReason reason, bool perTeam) const;
    ,
    (EventConfig) eventConfig,
    (std::vector<EventInterval>) eventIntervals,
    (unsigned)(1100) globalSendLimitWhenMessageRateTooHigh
  )
);

class EventManager : public EventManagerBase
{
private:
  void update(TeamCommEvents& events);

  bool isTeamEventOldEnough(const std::vector<TeamCommEvents::SendReason>& sendReasons) const;
  bool isEventOldEnough(const std::vector<TeamCommEvents::SendReason>& sendReasons) const;

  std::vector<TeamCommEvents::SendReason> getSendReasons();
  bool checkForNewBallchaser();
  bool checkForNewRoleAssignment();
  bool checkForPlayerMoved();
  bool checkForGoalDetected();
  bool checkForSymmetryLost();
  bool checkForSymmetryUpdate();
  bool checkForBallMoved();
  bool checkForBallchaserFallDown();
  bool checkForTimeResponses();
  bool checkForRefereeGesture();
  bool checkForUprightAgain();

  // member variables
  std::array<std::list<unsigned>, TeamCommEvents::SendReason::numOfSendReasons> newestLocalUpdates, newestTeamUpdates;
  Pose2f lastSendPosition;
  bool wasKickOffInProgress = false;
};
