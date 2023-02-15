/**
* @file HeadControl.h
*
* Declaration of class HeadControl, 
* calculates optimal head joint angle depending on head control request.
* @author <A href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*
*/
#pragma once

#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/BehaviorControl/HeadControlRequest.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/PositioningSymbols.h"
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
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/Path.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"


MODULE(HeadControl,
  REQUIRES(BallModel),
  REQUIRES(BallPercept),
  REQUIRES(BehaviorConfiguration),
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
  REQUIRES(TorsoMatrix),
  REQUIRES(SpeedInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  // behavior stuff
  REQUIRES(BallSymbols),
  REQUIRES(RemoteBallModel),
  REQUIRES(BehaviorData),
  REQUIRES(GameSymbols),
  REQUIRES(PositioningSymbols),
  PROVIDES(HeadAngleRequest),
  LOADS_PARAMETERS(,
    (Angle) headSpeedOpt, /* standard head speed for most motions */
    (Angle) headSpeedMax, /* fastest head speed, usually used when moving to specific percept */
    (Angle) headSpeedSafe, /* slow head speed, if percepts are more important than time needed for movement */
    (Angle)(3_deg) minHeadMovementPan, /* do not change head pan if movement would be less than this */
    (float) criticalBallDistance, /* if robot is waiting (not in goto) and ball is nearer, keep in sight */
    (float) criticalBallDistanceGoalie, /* if ball is nearer, always look at it, since goalie cant get called for pushing that way */
    (float) safeBallDistanceGoalie, /* Goalie sweep field around ball full angle if ball further away than that */
    (Angle) goalieSweepFieldFullAngle, /* The full angle for goalie to sweep field if ball further away than safeBallDistanceGoalie.
                                       Angle increases linearly between criticalBallDistanceGoalie and safeBallDistanceGoalie */
    (Angle) goalieSweepFieldMinAngleWhenBallNotMoving, /* Min angle to sweep field when ball is not moving but closer than criticalBallDistanceGoalie */
    (Angle)(15_deg) minTiltAngle,      /* Min tilt angle of the head to not look into the sky */
    /*
    * Parameters for ball percept verification. Usually only the BallModels are used.
    */
    (int) timeToFalsifyPercept, /* after this time (in ms) percept is either verified or falsified */
    (int) timeToKeepBallPerceptPosition, /* after this time (in ms) a falsified field position is removed from list */
    (bool) goalieLookAtPercepts, /* enable percept verification for goalie */
    (bool) fielderLookAtPercepts, /* enable percept verification for field players */
    (float) minDistanceOfBallPerceptToVerify, /* min percept distance for field players to start percept verification */
    (float) maxBallModelValForPerceptCheck, /* max validity of BallModels for decision to start percept verification */
    /*
     * Parameters for BallSeach using FieldCoverage
     */
    (float) viewDistance, /* max distance at wich the ball can be detected */
    (Angle) viewAngle, /* max angle (robot to point) at wich the point can be seen */
    (float) ballLostSweepFieldDistance,
    (float)(300.f) gotoStateMinDistance /** Minimum walking distance for goto state */
  )
);

class HeadControl : public HeadControlBase
{
public:
  class PointOfInterest
  {
  public:
    PointOfInterest() {}
    PointOfInterest(const Vector2f& point, Angle speed, int waitTime, bool wait, Angle angleRel, Angle overShoot = 25_deg)
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
  HeadControl();

private:
  ENUM(HeadControlState,
    ballTracking,
    ballLost,
    ballSweep,
    ballSweepWide,
    set,
    ready,
    localization,
    localizationOfBall,
    keeperSweepWide,
    keeperSweepBall,
    verifyBall
  );

  ENUM(RobotState,
    gotoBall,
    gotoPoint,
    searchForBall,
    waiting
  );

  void update(HeadAngleRequest& headAngleRequest);
  void resetQueue();
  void lookAtPointOnField(const Vector2f& point, Angle speed, bool waitForReached, int timeAtTarget);
  /** 
  * Scan path way and add interesting points
  */
  void addPointsOnPath(std::vector<Vector2f>& points);
  void calculatePointsOfInterest();
  void handleReadyState();
  void handlePlayingState();
  void handleLocalizeBall();
  void fillTargetQueue(const bool waitForReached, std::vector<Vector2f> pointsOfInterest);
  void lookAtPercept();
  void lookAtBall(bool lookAtPercept = false);
  void handleBallLost();
  void handleGoalie();
  Vector2f getBestFieldCrossingPoint();

  struct sortPoints
  {
    HeadControl* hc;
    sortPoints(HeadControl* p) : hc(p){};
    bool operator()(const PointOfInterest& a, const PointOfInterest& b)
    {
      return Transformation::fieldToRobot(hc->theRobotPose, a.pointOnField).angle() < Transformation::fieldToRobot(hc->theRobotPose, b.pointOnField).angle();
    }
  };

  struct sortRobots
  {
    HeadControl* hc;
    sortRobots(HeadControl* p) : hc(p){};
    bool operator()(const Pose2f& a, const Pose2f& b) { return (hc->theRobotPose.translation - a.translation).norm() < (hc->theRobotPose.translation - b.translation).norm(); }
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
  void lookAtRobots();
  void lookDown();
  void sweepField(const Angle leftAngle, const Angle rightAngle, const float distance, const bool forceSweepType, const Angle speed, bool sweepNearestFirst = true, bool waitForReached = true);
  bool isPerceptOnNoBallPosition();

  HeadControlState headControlState;
  HeadControlState lastHeadControlState;
  unsigned timeStampSwitchedToNewState;
  RobotState robotState;
  RobotState lastRobotState;
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
  bool calcAngles;
  //bool changedtargets;
  unsigned timeStampTarget;
  bool isLookingDown;
  RingBufferWithSum<Angle, 5> headPanDistances;
  unsigned timeStampLastHeadPanDiff;
  int timeForHeadMovement;

  bool wasBallLost;
  unsigned timeStampBallLost;
  bool ballDistanceWasCritical;
  bool ballWasLeft;
  std::vector<BallPerceptPosition> falsifiedBallPositions;
  Vector2f perceptToVerify;
  unsigned timeVerifyStarted;

  bool targetIsClose;
};
