#include "SimplePathProvider.h"
#include <algorithm>
#include <functional>
//#include "Representations/Infrastructure/TeamMateData.h" // TODO

SimplePathProvider::SimplePathProvider()
{
  wayPoints.clear();
  wayPoints.reserve(20);
  obstacles.clear();
  obstacles.reserve(20);
  state = far;
  wasBlockingWayToGoal = false;

  goalAreaDownLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftPenaltyArea+150);
  goalAreaDownRight = Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightPenaltyArea-150);
  goalAreaUpRight = Vector2f(theFieldDimensions.xPosOwnPenaltyArea+150,theFieldDimensions.yPosRightPenaltyArea-150);
  goalAreaUpLeft = Vector2f(theFieldDimensions.xPosOwnPenaltyArea+150,theFieldDimensions.yPosLeftPenaltyArea+150);
}

void SimplePathProvider::update(Path& path)
{
  // TODO: ball next to goal post
  DECLARE_DEBUG_DRAWING("module:SimplePathProvider:obstacles","drawingOnField");
 
  // clear old wayPoints/obstacles
  // TODO : remember the old way
  if (theMotionRequest.motion == MotionRequest::walk && theMotionRequest.walkRequest.requestType == WalkRequest::destination)
  {
    wayPoints.clear();
    obstacles.clear();

    destWorldCoordinates = Pose2f(theRobotPoseAfterPreview + theMotionRequest.walkRequest.request);

    /*** always build new path, if only to compare with old one ***/
    // add wayPoints from robots, but only if situation is not critical defensively
    if (theBallSymbols.ballPositionFieldPredicted.x() > 2 * theFieldDimensions.xPosOwnGroundline / 3
      || (theRobotPose.translation - theBallSymbols.ballPositionFieldPredicted).norm() < 2500)
      handleRobots();
    // add wayPoint from ball
    handleBall();
    // add wayPoints from static obstacles
    handleStaticObstacles();
    // build path avoiding obstacles in way
    buildPath();

    // check, if old path (with updated destination) is still fine
    path.wayPoints.front() = theRobotPoseAfterPreview;
    path.wayPoints.back() = destWorldCoordinates;
    if (!isOldPathFine(path))
    {
      path.reset();
      path.wayPoints = wayPoints;
      
    }
    // TODO: remember obstacle id and update distance, if obstacle is not behind..
    path.nearestObstacle = distanceToClosestObstacle;
  }
  else
  {
    path.reset();
    path.nearestObstacle = distanceToClosestObstacle;
    path.wayPoints.push_back(theRobotPoseAfterPreview);
    path.wayPoints.push_back(theRobotPoseAfterPreview);
  }
}

void SimplePathProvider::handleRobots()
{
  std::vector<RobotMapEntry>::const_iterator robot = theRobotMap.robots.begin();
  std::vector<RobotMapEntry>::const_iterator end = theRobotMap.robots.end();
  for (; robot != end; ++robot)
  {
    const Vector2f &trans = robot->pose.translation;
    if ((trans-theRobotPose.translation).norm() < 2500)
    {
      obstacles.push_back(
        Obstacle(robot->pose.translation.x(),robot->pose.translation.y(),
        ((robot->robotType == RobotEstimate::teammateRobot) ? teamRobotInfluenceRadius : robotInfluenceRadius), 
        Obstacle::robot,
        (robot->robotType == RobotEstimate::teammateRobot))
        );
    }
  }
}

void SimplePathProvider::handleBall()
{
  if ((theBallSymbols.timeSinceLastSeen < 2000 || 
      (theBallSymbols.timeSinceLastSeenByTeam < 2000 && Transformation::fieldToRobot(theRobotPoseAfterPreview,theBallSymbols.ballPositionField).x() < 0))
    && theGameInfo.state == STATE_PLAYING)
  {
    const Vector2f &trans = Transformation::robotToField(theRobotPoseAfterPreview, theBallModelAfterPreview.estimate.position);
    float alpha = std::abs(Angle::normalize(destWorldCoordinates.translation.angle() - (destWorldCoordinates.translation-theBallSymbols.ballPositionField).angle()))
      / pi;
    obstacles.push_back(Obstacle(trans.x(), trans.y(), ballInfluenceRadius+ballInfluenceRadius*alpha, Obstacle::ball));
  }
}

void SimplePathProvider::handleStaticObstacles()
{
  Vector2f oppGoalPostLeft(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosLeftGoal);
  Vector2f oppGoalPostRight(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightGoal);
  Vector2f ownGoalPostLeft(theFieldDimensions.xPosOwnGoalPost,theFieldDimensions.yPosLeftGoal);
  Vector2f ownGoalPostRight(theFieldDimensions.xPosOwnGoalPost,theFieldDimensions.yPosRightGoal);

  obstacles.push_back(Obstacle(oppGoalPostLeft.x(), oppGoalPostLeft.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(oppGoalPostRight.x(), oppGoalPostRight.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostLeft.x(), ownGoalPostLeft.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostRight.x(), ownGoalPostRight.y(), goalPostInfluenceRadius, Obstacle::goalPost));
}

void SimplePathProvider::buildPath()
{
  if (theGameSymbols.avoidCenterCircle)
  {
    float circleSafetyRadius = theFieldDimensions.centerCircleRadius + centerCircleInfluenceRadius;
    if (destWorldCoordinates.translation.norm() < circleSafetyRadius)
    {
      float rotation = Angle::normalize(theRobotPoseAfterPreview.translation.angle() + pi);
      destWorldCoordinates = Pose2f(rotation,
        Vector2f(theRobotPoseAfterPreview.translation).normalize(circleSafetyRadius));
    }
    else 
    {
      obstacles.push_back(Obstacle(0,0,centerCircleInfluenceRadius+theFieldDimensions.centerCircleRadius,Obstacle::centerCircle));
    }
  }

  wayPoints.push_back(theRobotPoseAfterPreview);
    
  bool notAllowedInGoalArea = false;
  // handle own goal area
  // cases ignored when ball on goal line and inside left/right penalty area lines 
  // -> one robot is allowed into penalty area anyway, behavior should not send other robots there
  // does not add all necessary wayPoints for avoidance at once, they are added on the way if going around goal area
  if (theBehaviorData.role != BehaviorData::keeper && theGameSymbols.allowedInPenaltyArea == false)
  {
    notAllowedInGoalArea = true;
    static const Geometry::Line lineFront(goalAreaUpLeft,goalAreaUpRight-goalAreaUpLeft);
    const Geometry::Line lineToTarget(theRobotPoseAfterPreview.translation,destWorldCoordinates.translation-theRobotPoseAfterPreview.translation);
    static const Geometry::Line lineLeft(goalAreaUpLeft,goalAreaDownLeft-goalAreaUpLeft);
    static const Geometry::Line lineRight(goalAreaUpRight,goalAreaDownRight-goalAreaUpRight);
    static const Geometry::Line lineBack(goalAreaDownLeft,goalAreaDownRight-goalAreaDownLeft);

    Vector2f intersectionPoint(0,0);

    // below penalty area line? 
    if (destWorldCoordinates.translation.x() < goalAreaUpLeft.x())
    {
      // if the robot is below the penalty area line and has to cross sides
      if (theRobotPoseAfterPreview.translation.x() < goalAreaUpLeft.x() - 100)
      {
        // robot is on right side
        if (theRobotPoseAfterPreview.translation.y() < goalAreaUpRight.y() - 100 && destWorldCoordinates.translation.y() > goalAreaUpLeft.y() - 100)
        {
          wayPoints.push_back(Pose2f(pi_2, goalAreaUpRight));
          wayPoints.push_back(Pose2f((destWorldCoordinates.translation - goalAreaUpLeft).angle(), goalAreaUpLeft));
        }
        // robot is on left side
        else if (theRobotPoseAfterPreview.translation.y() > goalAreaUpLeft.y() + 100 && destWorldCoordinates.translation.y() < goalAreaUpRight.y() + 100)
        {
          wayPoints.push_back(Pose2f(-pi_2, goalAreaUpLeft));
          wayPoints.push_back(Pose2f((destWorldCoordinates.translation - goalAreaUpRight).angle(), goalAreaUpRight));
        }
      }
      // robot above penalty area line? Threshold to make sure robot reaches position
      else if (theRobotPoseAfterPreview.translation.x() > goalAreaUpLeft.x() + 100)
      {
        // would cross penalty area on the way to target (left)
        if (destWorldCoordinates.translation.y() > 0 && theRobotPoseAfterPreview.translation.y() < goalAreaUpLeft.y() + 100
          && Geometry::getIntersectionOfLines(lineToTarget,lineFront,intersectionPoint) && intersectionPoint.y() < goalAreaUpLeft.y())
          wayPoints.push_back(Pose2f(pi3_4, goalAreaUpLeft));
        // would cross penalty area on the way to target (right)
        else if (destWorldCoordinates.translation.y() < 0 && theRobotPoseAfterPreview.translation.y() > goalAreaUpRight.y() - 100
          && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint) && intersectionPoint.y() > goalAreaUpRight.y())
          wayPoints.push_back(Pose2f(-pi3_4, goalAreaUpRight));
      }
    }
    // if the robot is below the penalty area line
    else if (theRobotPoseAfterPreview.translation.x() < goalAreaUpLeft.x() + 100)
    {
      // robot is on right side and would cross penalty area
      if (theRobotPoseAfterPreview.translation.y() < goalAreaUpRight.y() + 100
        && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint) && intersectionPoint.y() > goalAreaUpRight.y())
      {
        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - goalAreaUpRight).angle(), goalAreaUpRight));
      }
      // robot is on left side and would cross penalty area
      else if (theRobotPoseAfterPreview.translation.y() > goalAreaUpLeft.y() - 100
        && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint) && intersectionPoint.y() < goalAreaUpLeft.y())
      {
        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - goalAreaUpLeft).angle(), goalAreaUpLeft));
      }
    }
    
  }

  if (wayPoints.size() > 1 && 
    ((wayPoints[1].translation - theRobotPoseAfterPreview.translation).norm() < 150 
      || (wayPoints[1].translation - destWorldCoordinates.translation).norm() < (destWorldCoordinates.translation - theRobotPoseAfterPreview.translation).norm()))
    wayPoints.erase(wayPoints.begin() + 1);

  avoidObstacles(notAllowedInGoalArea);

  // needed bc of own goal area waypoints (not anymore?)
  //   std::sort(wayPoints.begin(),wayPoints.end(),sortPoses(this));

  // if block way to goal is requested (while going to ball), move between ball and goal before actually going to ball
  // useful for defence
  if (theKickSymbols.blockWayToGoalOnApproach && wayPoints.size() == 1 && theMotionRequest.walkRequest.request.translation.norm() > 750.f - (wasBlockingWayToGoal ? 250.f : 0))
  {
    Vector2f destRel = destWorldCoordinates.translation - theRobotPoseAfterPreview.translation;
    Vector2f goalToBall = theBallSymbols.ballPositionFieldPredicted - Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f);
    Angle angleGoalToBall = goalToBall.angle();
    Angle angleDifference = std::abs(Angle::normalize(destRel.angle() - angleGoalToBall));
    Angle angleHysteresis = (wasBlockingWayToGoal ? 10_deg : 0_deg);
    if (std::abs(angleGoalToBall) < 60_deg + angleHysteresis
      && std::abs(angleDifference) > 20_deg - angleHysteresis
      && std::abs(angleDifference) < 30_deg + angleHysteresis
      && std::abs(Angle::normalize(angleGoalToBall - theRobotPoseAfterPreview.rotation)) < 30_deg + angleHysteresis)
    {
      goalToBall.normalize(destRel.norm()*std::cos(angleDifference));
      Pose2f newWP(angleGoalToBall, destWorldCoordinates.translation - goalToBall);
      if ((newWP.translation - theRobotPoseAfterPreview.translation).norm() < 600.f + (wasBlockingWayToGoal ? 150.f : 0.f))
      {
        wayPoints.push_back(newWP);
        distanceToClosestObstacle = robotInfluenceRadius - 50;
        wasBlockingWayToGoal = true;
      }
    }

  }
  else
    wasBlockingWayToGoal = false;
  
  wayPoints.push_back(destWorldCoordinates);
  
}

void SimplePathProvider::avoidObstacles(bool notAllowedInGoalArea)
{
  // TODO: create wall were player may shoot ball and avoid that..??!!
  // sort obstacles by distance
  std::sort(obstacles.begin(),obstacles.end(),sortObstacles(this));
  int i = 0;
  distanceToClosestObstacle = 10000.f;

  // merge too close obstacles
  // TODO: memory!
  // TODO: should do this while building path..
  // TODO: check missing if obstacle is actually relevant
  if (mergeObstacles)
  {
    const float maxObstacleDistanceForMerging = 500.f;
    for (size_t first = 0; first < obstacles.size(); first++)
    {
      for (size_t second = first + 1; second < obstacles.size(); second++)
      {
        float obstacleDistance = (obstacles[first].position - obstacles[second].position).norm();
        if (obstacleDistance < std::min(maxObstacleDistanceForMerging, obstacles[first].radius + obstacles[second].radius) &&
          !(obstacles[first].type == Obstacle::ball || obstacles[second].type == Obstacle::ball) &&
          !((obstacles[first].type == Obstacle::goalPost && obstacles[second].type == Obstacle::robot) ||
            (obstacles[second].type == Obstacle::goalPost && obstacles[first].type == Obstacle::robot)))
        {
          obstacles[first].position = (obstacles[first].position + obstacles[second].position) / 2;
          obstacles[first].radius = obstacleDistance + (obstacles[first].radius + obstacles[second].radius) / 2;
          obstacles.erase(obstacles.begin() + second);
          second = obstacles.size();
        }
      }
    }
  }

  ballIsObstacle = false;

  for (std::vector<Obstacle>::iterator obstacle = obstacles.begin(); obstacle != obstacles.end(); ++obstacle)
  {
    if (obstacle->type == Obstacle::ball)
      ballIsObstacle = true;
    i++;
    CIRCLE("module:SimplePathProvider:obstacles",
      obstacle->position.x(),
      obstacle->position.y(),
      obstacle->radius,
      5,Drawings::solidPen,ColorRGBA::yellow,
      Drawings::noBrush, ColorRGBA::yellow);
    DRAWTEXT("module:SimplePathProvider:obstacles",
      obstacle->position.x(),
      obstacle->position.y(),
      30, ColorRGBA::yellow,
      i);
    const Pose2f &currentWayPoint = ((wayPoints.size() < 2) ? (Pose2f&)theRobotPose : wayPoints.back());
    const Vector2f vectorToTarget = (destWorldCoordinates.translation - currentWayPoint.translation);
    const float distance = vectorToTarget.norm();
    const Vector2f vectorToObstacle = (obstacle->position - currentWayPoint.translation);
    const float distanceToObstacle = vectorToObstacle.norm();

    const float angleToTargetWC = vectorToTarget.angle();
    const float angleToObstacleWC = vectorToObstacle.angle();
    float angleDiff = Angle::normalize(angleToTargetWC-angleToObstacleWC);
    float maxAngleDiff = pi_2*0.8f;
    if (obstacle->type == Obstacle::robot && obstacle->isTeamMate)
      maxAngleDiff = pi_2*1.5f;

    if ((std::abs(angleDiff) < maxAngleDiff && 
      distanceToObstacle < distance) || 
      (obstacle->type == Obstacle::ball 
      && std::abs(Angle::normalize(angleToObstacleWC-destWorldCoordinates.rotation)) > pi_2*0.7f 
      && distance > 150
      && distanceToObstacle -50 < distance))
    {
      // might be dangerous
      if (std::abs(Angle::normalize(angleToObstacleWC-destWorldCoordinates.rotation)) < pi_2 && obstacle->type == Obstacle::ball)
      {
        obstacle->radius = ballInfluenceRadius;
        obstacle->position = Transformation::robotToField(theRobotPoseAfterPreview, theBallModelAfterPreview.estimate.position);
      }
      CIRCLE("module:SimplePathProvider:obstacles",
        obstacle->position.x(),
        obstacle->position.y(),
        obstacle->radius,
        5,Drawings::solidPen, ColorRGBA::orange,
        Drawings::noBrush, ColorRGBA::orange);
      Vector2f normalToObstacle = vectorToObstacle;
      if (std::abs(angleDiff) > pi_4)
        angleDiff = pi_4*sgn(angleDiff);
      if (angleDiff > 0)
        normalToObstacle.rotate(pi_2+std::abs(angleDiff));
      else
        normalToObstacle.rotate(-pi_2-std::abs(angleDiff));
      
      normalToObstacle.normalize(obstacle->radius);

      Vector2f avoidancePoint = obstacle->position + normalToObstacle;

      const Vector2f goalCenter(theFieldDimensions.xPosOwnGroundline, 0.f);
      if ((notAllowedInGoalArea && 
          Geometry::isPointInsideRectangle(goalAreaDownLeft, goalAreaUpRight, avoidancePoint) && 
          (goalCenter - (obstacle->position + normalToObstacle)).norm() < (goalCenter - (obstacle->position - normalToObstacle)).norm())
        || (obstacle->type == Obstacle::goalPost && 
          std::abs(obstacle->position.x()+normalToObstacle.x()) > theFieldDimensions.xPosOpponentGroundline))
      {
        normalToObstacle.rotate(pi);
      }

      avoidancePoint = obstacle->position + normalToObstacle;

      float distObstacleToTargetLine = std::asin(std::min<float>(std::abs(angleDiff),1.f))*distanceToObstacle;

      if (normalToObstacle.norm() > distObstacleToTargetLine)
      {
        if (obstacle->type != Obstacle::ball)
          distanceToClosestObstacle = std::min<float>(distanceToClosestObstacle,distanceToObstacle);
        // okay, that thing is really blocking my path
        CIRCLE("module:SimplePathProvider:obstacles",
        obstacle->position.x(),
        obstacle->position.y(),
        obstacle->radius,
        5,Drawings::solidPen, ColorRGBA::red,
        Drawings::noBrush, ColorRGBA::red);

        // if inside obstacle influence radius, try to get out roughly in direction to target
        if (avoidRobotCrashes && (theRobotPose.translation - obstacle->position).norm() <= obstacle->radius)
        {
          // back out vector
          Vector2f meToObstacle = (theRobotPose.translation - obstacle->position);
          // rotate back out vector towards target to at least move a little close to target
          Angle backOutVectorToTargetVector = Angle::normalize((destWorldCoordinates.translation - obstacle->position).angle() - meToObstacle.angle());
          meToObstacle.rotate(-sgn(backOutVectorToTargetVector) * std::min<Angle>(std::abs(backOutVectorToTargetVector), 30_deg));
          avoidancePoint = obstacle->position + meToObstacle.normalize(obstacle->radius*1.2f);
        }
        
        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - avoidancePoint).angle(), avoidancePoint));
      }

      LINE("module:SimplePathProvider:obstacles",
        obstacle->position.x(),
        obstacle->position.y(),
        avoidancePoint.x(),
        avoidancePoint.y(),
        4,Drawings::solidPen, ColorRGBA::blue);
    }
  }
}

bool SimplePathProvider::isOldPathFine(Path& path)
{
  if (!stablizePath)
    return false;
  /*** check for length and if (new) obstacle is in way ***/
  float oldLength = 0.f;
  float newLength = 0.f;
  if ((path.wayPoints[1].translation - path.wayPoints[0].translation).norm() < 250)
    return false;
  for (unsigned int i = 1; i < path.wayPoints.size(); i++)
  {
    Vector2f vToNextWayPoint = path.wayPoints[i].translation - path.wayPoints[i - 1].translation;
    for (auto &robot : theRobotMap.robots)
    {
      if (distancePointToVector(robot.pose.translation, vToNextWayPoint, path.wayPoints[i-1].translation) < 150)
        return false;
    }
    for (auto const& obstacle : obstacles)
    {
      if (obstacle.type != Obstacle::robot)
      {
        if (distancePointToVector(obstacle.position, vToNextWayPoint, path.wayPoints[i - 1].translation) < 150)
          return false;
      }
    }
    oldLength += vToNextWayPoint.norm();
  }
  for (unsigned int i = 1; i < wayPoints.size(); i++)
    newLength += (wayPoints[i].translation - wayPoints[i - 1].translation).norm();
  return (oldLength - 500 < newLength);
}

MAKE_MODULE(SimplePathProvider, pathPlanning)