#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Path.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"

MODULE(SimplePathProvider,
  REQUIRES(BallSymbols),
  REQUIRES(BallModelAfterPreview),
  REQUIRES(BehaviorData),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(Ballchaser),
  REQUIRES(MotionRequest),
  REQUIRES(RobotMap),
  REQUIRES(RobotPose),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(GameSymbols),
  PROVIDES(Path),
  LOADS_PARAMETERS(,
    (float)(200.f) ballInfluenceRadius,
    (float)(350.f) centerCircleInfluenceRadius,
    (float)(250.f) goalPostInfluenceRadius,
    (float)(400.f) robotInfluenceRadius,
    (float)(500.f) teamRobotInfluenceRadius,
    (float)(600.f) ballchaserInfluenceRadius,
    (float)(400.f) targetStateSwitchDistance,
    (float)(100.f) targetStateSwitchDistanceHysteresis,
    (float)(300.f) omniStateSwitchDistance,
    (float)(100.f) omniStateSwitchDistanceHysteresis,
    (bool)(true) keeperInGoalAreaOmniOnly,
    (bool)(false) stablizePath,
    (bool)(false) mergeObstacles,
    (bool)(false) avoidRobotCrashes,
    (Angle)(90_deg) ballApproachCone,
    (float)(800.f) setPlayInfluenceRadius
  )
);

class SimplePathProvider : public SimplePathProviderBase
{
public:
  ENUM(PathFollowState,
    omni, // near obstacles, omnidirectional avoidance
    target, // near target: keep in sight, turn to target rotation
    far // default: high speed
  );

  SimplePathProvider();

private:
  struct Obstacle
  {
    Obstacle(float x, float y, float rad, Path::ObstacleType t, bool mate = false)
    {
      position.x() = x;
      position.y() = y;
      radius = rad;
      type = t;
      isTeamMate = mate;
    }
    Vector2f position;
    float radius;
    Path::ObstacleType type;
    bool isTeamMate;
  };

  struct sortObstacles
  {
    SimplePathProvider* spp;
    sortObstacles(SimplePathProvider* p) : spp(p){};
    bool operator()(const Obstacle& a, const Obstacle& b)
    {
      return (spp->theRobotPoseAfterPreview.translation - a.position).norm() - a.radius < (spp->theRobotPoseAfterPreview.translation - b.position).norm() - b.radius;
    }
  };

  struct sortPoses
  {
    SimplePathProvider* spp;
    sortPoses(SimplePathProvider* p) : spp(p){};
    bool operator()(const Pose2f& a, const Pose2f& b)
    {
      return (spp->theRobotPoseAfterPreview.translation - a.translation).norm() < (spp->theRobotPoseAfterPreview.translation - b.translation).norm();
    }
  };
  bool initialized = false;
  PathFollowState state = PathFollowState::far;
  bool wasBlockingWayToGoal = false;

  float distanceToClosestObstacle;
  Vector2f nearestObstaclePosition;
  Path::ObstacleType nearestObstacleType;
  std::vector<Pose2f> wayPoints;
  std::vector<Obstacle> obstacles;
  bool ballIsObstacle = false;
  bool avoidBallLeft = false;
  bool inBallApproach = false;
  Vector2f goalAreaDownLeft;
  Vector2f goalAreaDownRight;
  Vector2f goalAreaUpRight;
  Vector2f goalAreaUpLeft;

  void initialize();
  void update(Path& path);
  void handleRobots();
  void handleBall();
  void handleStaticObstacles();
  void buildPath();
  /** used for goal/penalty area avoidance **/
  void avoidRectangularArea(const Vector2f& bottomLeft, const Vector2f& topLeft, const Vector2f& topRight, const Vector2f& bottomRight);
  void avoidObstacles(bool notAllowedInPenaltyArea);
  bool isOldPathFine(Path& path);

  Pose2f destWorldCoordinates;

  inline float distancePointToVector(const Vector2f& point, const Vector2f& vector, const Vector2f& base)
  {
    Vector2f normal = vector;
    normal.rotateRight();
    normal.normalize();
    float distance = (point - base).dot(normal);
    return std::abs(distance);
  }
};
