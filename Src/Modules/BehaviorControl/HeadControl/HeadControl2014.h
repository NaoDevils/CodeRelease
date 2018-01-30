/**
* @file HeadControl2014.h
*
* Declaration of class HeadControl2014, 
* calculates optimal head joint angle depending on head control request.
* @author <A href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*
*/
#pragma once

#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/Path.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/RobotModel.h"


MODULE(HeadControl2014,
{ ,
  REQUIRES(BallModel),
  REQUIRES(BallPercept),
  REQUIRES(CameraInfo),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FieldDimensions),
  REQUIRES(JointSensorData),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(HeadControlRequest),
  REQUIRES(JointCalibration),
  REQUIRES(TeamBallModel),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(RobotMap),
  REQUIRES(RobotModel),
  // behavior stuff
  REQUIRES(RemoteBallModel),
  REQUIRES(BehaviorData),
  PROVIDES(HeadAngleRequest),
  LOADS_PARAMETERS(
  {,
    (Angle) headSpeedOpt, /* standard head speed for most motions */
    (Angle) headSpeedMax, /* fastest head speed, usually used when moving to specific percept */
    (Angle) headSpeedSafe, /* slow head speed, if percepts are more important than time needed for movement */
    (float) criticalBallDistance, /* if robot is waiting (not in goto) and ball is nearer, keep in sight */
    (float) criticalBallDistanceGoalie, /* if ball is nearer, always look at it, since goalie cant get called for pushing that way */
    /*
    * Parameters for ball percept verification. Usually only the BallModels are used.
    */
    (int) timeToFalsifyPercept, /* after this time (in ms) percept is either verified or falsified */
    (int) timeToKeepBallPerceptPosition, /* after this time (in ms) a falsified field position is removed from list */
    (bool) goalieLookAtPercepts, /* enable percept verification for goalie */
    (bool) fielderLookAtPercepts, /* enable percept verification for field players */
    (float) minDistanceOfBallPerceptToVerify, /* min percept distance for field players to start percept verification */
    (float) maxBallModelValForPerceptCheck, /* max validity of BallModels for decision to start percept verification */
  }),
});

class HeadControl2014: public HeadControl2014Base
{
public:
  class PointOfInterest
  {
  public:
    PointOfInterest() {}
    PointOfInterest(const Vector2f &point, Angle speed, int waitTime, bool wait, 
      Angle angleRel, Angle overShoot = 25_deg)
    {
      pointOnField = point;
      maxSpeed = speed;
      timeAtTarget = waitTime;
      waitForReached = wait;
      relativeAngle = angleRel;
      maxOverShoot = overShoot;
    }
    Vector2f pointOnField;
    Angle maxSpeed;
    int timeAtTarget;
    bool waitForReached;
    Angle relativeAngle;
    Angle maxOverShoot;
  };
  HeadControl2014();

private:
  ENUM(HeadControlState,
  { ,
    ballTracking,
    ballLost,
    ballSweep,
    ballSweepWide,
    set,
    ready,
    localization,
    keeperSweepWide,
    keeperSweepBallFar,
    keeperSweepBallNear,
    verifyBall,
  });

  ENUM(RobotState,
  { ,
    gotoBall,
    gotoPoint,
    searchForBall,
    waiting,
  });

  void update(HeadAngleRequest &headAngleRequest);
  void resetQueue();
  void lookAtPointOnField(const Vector2f &point, Angle speed, bool waitForReached, int timeAtTarget);
  void calculatePointsOfInterest();
  void handleReadyState();
  void handlePlayingState();
  void lookAtBall(bool lookAtPercept = false);
  
  struct sortPoints
  {
    HeadControl2014* hc;
    sortPoints(HeadControl2014* p) : hc(p) {};
    bool operator()(const PointOfInterest &a, const PointOfInterest &b)
    {
      return Transformation::fieldToRobot(hc->theRobotPose,a.pointOnField).angle() <
        Transformation::fieldToRobot(hc->theRobotPose,b.pointOnField).angle();
    }
  };

  struct sortRobots
  {
    HeadControl2014* hc;
    sortRobots(HeadControl2014* p) : hc(p) {};
    bool operator()(const Pose2f &a, const Pose2f &b)
    {
      return (hc->theRobotPose.translation - a.translation).norm() <
             (hc->theRobotPose.translation - b.translation).norm();
    }
  };

  struct BallPerceptPosition
  {
    Vector2f position;
    unsigned timeStampCreated;
  };
  
  /**
  * Calculate head joint angles to look at first point in target queue.
  */
  void calcHeadAngles();
  void moveHead();
  void lookDown();
  void sweepField(const Angle leftAngle, 
    const Angle rightAngle,
    const float distance,
    const bool forceSweepType,
    const Angle speed,
    bool sweepNearestFirst = true,
    bool waitForReached = true);
  
  HeadControlState headControlState;
  HeadControlState lastHeadControlState;
  // queue for points of interest on field to look at, always check if empty
  // in field coordinates
  std::vector<PointOfInterest> targetQueue;
  Vector2f lastPOI;
  Vector2f lastPOIRel;
  HeadAngleRequest localHeadAngleRequest;
  Vector2f lastSpeed;
  Angle newPan;
  Angle newTilt;
  bool trackWithUpper;
  HeadControlRequest::ControlType lastControlType;
  unsigned int timeSinceTypeChange;
  bool targetReached;
  bool relativeAngleReached;
  //bool changedtargets;
  unsigned timeStampTarget;
  bool isLookingDown;
  RingBufferWithSum<float, 5> headPanDistances;
  unsigned timeStampLastHeadPanDiff;
  int timeForHeadMovement;
  
 std::vector<BallPerceptPosition> falsifiedBallPositions;
  Vector2f perceptToVerify;

  bool targetIsClose;
};