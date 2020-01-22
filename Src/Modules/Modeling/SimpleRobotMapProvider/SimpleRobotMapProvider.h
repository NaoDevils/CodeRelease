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
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"

MODULE(SimpleRobotMapProvider,
{ ,
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(RobotsPercept),
  REQUIRES(RobotsPerceptUpper),
  REQUIRES(TeammateData),
  PROVIDES(RobotMap),
  PROVIDES(LocalRobotMap),
  PROVIDES(SimpleRobotsDistributed),
  LOADS_PARAMETERS(
  {,
    (bool) global, // use team mate data?
    (bool) useTeamMatePercepts, // use percepts from team mates? PROBLEM: preview!
    (bool) useVelocity,// if false, robot positions will not change without new percepts

    (float) mergeLocationDiff,
    (float) mergeRotDiff,
    (float) mergeSpeedDiff,
    (Angle) mergeAngleXDiff,
    (Angle) mergeAngleYDiff,
    (float) minValidity, // 0..1
    (float) maxStartValidity, // max validity, when robot map entry is added
    (float) validityUpdate, // per frame

    (int) minColorCount, // min colored percepts for mate/opp decision

    (unsigned int) maxRobotsToSend, // Guess what.
  }),
});

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

  SimpleRobotMapProvider();

private:

  void update(RobotMap &robotMap);
  void update(LocalRobotMap &localRobotMap);
  void update(SimpleRobotsDistributed &simpleRobotsDistributed);
  void execute();
  void updateWithLocalData(const bool& upper);
  void updateWithTeamMatePoses();
  void updateWithGlobalData();

  void mergeSimpleRobots();
  void prune();
  void predict();
  void fillRobotMap();

  std::vector<SimpleRobot> simpleRobots;
  unsigned timeOfLastExeccution;
  RobotMap internalRobotMap;
};
