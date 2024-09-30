// very simple robot map provider
// until the real one is fixed

#pragma once

#include <algorithm>
#include "Tools/Module/Module.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SimpleRobotsDistributed.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/BehaviorControl/GameSymbols.h"

MODULE(SimpleRobotMapProvider,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(RobotsPercept),
  REQUIRES(TeammateData),
  REQUIRES(GameInfo),
  USES(GameSymbols),
  PROVIDES(RobotMap),
  PROVIDES(LocalRobotMap),
  PROVIDES(SimpleRobotsDistributed),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    (bool) global, // use team mate data?
    (bool) useTeamMatePercepts, // use percepts from team mates? PROBLEM: preview!
    (float)(0.8f) teamMateMinValidity, // use percepts from team mates? PROBLEM: preview!
    (bool) useVelocity,// if false, robot positions will not change without new percepts

    (float)(0.5f) mergeLocationFactor,
    (float) mergeLocationDiff,
    (float) mergeRotDiff,
    (float) mergeSpeedDiff,
    (Angle) mergeAngleXDiff,
    (Angle) mergeAngleYDiff,
    (float) minValidity, // 0..1
    (float) maxStartValidity, // max validity, when robot map entry is added
    (float) validityUpdate, // per frame
    (int) maxLocalLifetimeOfUnseenRobot,
    (int) maxLifetimeOfUnseenRobot,

    (int) minColorCount, // min colored percepts for mate/opp decision

    (unsigned int) maxRobotsToSend // Guess what.
  )
);

class SimpleRobotMapProvider : public SimpleRobotMapProviderBase
{
public:
  struct SimpleRobot
  {
    Pose2f pose;
    Vector2f velocity;
    int teamMateCounter; // -x = opponent, +x = teammate
    bool fromTeamMatePoses;
    float validity;
    unsigned timeOfLastUpdate;
    unsigned timeOfLastLocalUpdate;
  };

  enum
  {
    maxRobots = 15
  };

private:
  void update(RobotMap& robotMap);
  void update(LocalRobotMap& localRobotMap);
  void update(SimpleRobotsDistributed& simpleRobotsDistributed);
  void execute(tf::Subflow&);
  void updateWithLocalData();
  void updateWithTeamMatePoses();
  void updateWithGlobalData();

  void mergeSimpleRobots();
  void prune();
  void predict();
  void fillRobotMap();

  std::vector<SimpleRobot> simpleRobots;
  RobotMap internalRobotMap;
  Pose2f lastRobotPose;
};
