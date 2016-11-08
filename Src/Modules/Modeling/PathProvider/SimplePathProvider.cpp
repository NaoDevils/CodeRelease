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
    // add wayPoints from robots
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
  if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 3000
    && theGameInfo.state == STATE_PLAYING)
  {
    const Vector2f &trans = Transformation::robotToField(theRobotPoseAfterPreview, theBallModelAfterPreview.estimate.position);
    float alpha = std::abs(Angle::normalize(destWorldCoordinates.translation.angle() - (destWorldCoordinates.translation - Transformation::robotToField(theRobotPoseAfterPreview, theBallModel.estimate.position)).angle()))
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
  wayPoints.push_back(theRobotPoseAfterPreview);
    
  bool notAllowedInGoalArea = false;
  // handle own goal area
  // cases ignored when ball on goal line and inside left/right penalty area lines 
  // -> one robot is allowed into penalty area anyway, behavior should not send other robots there
  // does not add all necessary wayPoints for avoidance at once, they are added on the way if going around goal area
  if (theBehaviorData.role != BehaviorData::keeper)
  {
    notAllowedInGoalArea = true;
    static const Geometry::Line lineFront(goalAreaUpLeft,goalAreaUpRight-goalAreaUpLeft);
    static const Geometry::Line lineToTarget(theRobotPoseAfterPreview.translation,destWorldCoordinates.translation-theRobotPoseAfterPreview.translation);
    static const Geometry::Line lineLeft(goalAreaUpLeft,goalAreaDownLeft-goalAreaUpLeft);
    static const Geometry::Line lineRight(goalAreaUpRight,goalAreaDownRight-goalAreaUpRight);
    static const Geometry::Line lineBack(goalAreaDownLeft,goalAreaDownRight-goalAreaDownLeft);

    Vector2f intersectionPoint(0,0);

    if (destWorldCoordinates.translation.x() < goalAreaUpLeft.x() && 
      theRobotPoseAfterPreview.translation.x() > goalAreaUpLeft.x() &&
      Geometry::getIntersectionOfLines(lineToTarget,lineFront,intersectionPoint))
    {
      if (std::abs(intersectionPoint.x() - goalAreaUpLeft.x()) < 10 && std::abs(intersectionPoint.y()) < goalAreaUpLeft.y())
      {
        if (destWorldCoordinates.translation.y() > 0)
          wayPoints.push_back(Pose2f(pi3_4,goalAreaUpLeft));
        else
          wayPoints.push_back(Pose2f(-pi3_4,goalAreaUpRight));
      }
    }
    if (theRobotPoseAfterPreview.translation.x() < goalAreaUpLeft.x())
    {
      if (Geometry::getIntersectionOfLines(lineToTarget,lineLeft,intersectionPoint))
      {
        if ((intersectionPoint.x() > theRobotPoseAfterPreview.translation.x()-50 || destWorldCoordinates.translation.y() < 0)
          && theRobotPoseAfterPreview.translation.y() > goalAreaUpLeft.y()-100)
        {
          wayPoints.push_back(Pose2f(-pi_4,goalAreaUpLeft));
        }
      }
    }
    if (theRobotPoseAfterPreview.translation.x() < goalAreaUpRight.x())
    {
      if (Geometry::getIntersectionOfLines(lineToTarget,lineRight,intersectionPoint))
      {
        if ((intersectionPoint.x() > theRobotPoseAfterPreview.translation.x()-50 || destWorldCoordinates.translation.y() > 0)
          && theRobotPoseAfterPreview.translation.y() < goalAreaUpRight.y()+100)
        {
          wayPoints.push_back(Pose2f(pi_4,goalAreaUpRight));
        }
      }
    }

    if (destWorldCoordinates.translation.x() < goalAreaUpLeft.x() &&
      theRobotPoseAfterPreview.translation.y() < goalAreaDownLeft.y() &&
      Geometry::getIntersectionOfLines(lineToTarget,lineLeft,intersectionPoint))
    {
      if (std::abs(intersectionPoint.y() - goalAreaDownLeft.y()) < 10 && intersectionPoint.x() < goalAreaUpLeft.x() && intersectionPoint.x() > goalAreaDownLeft.x())
      {
        wayPoints.push_back(Pose2f(-pi3_4, (float)goalAreaDownLeft.x() - 100, (float)goalAreaDownLeft.y()));
      }
    }
    if (destWorldCoordinates.translation.x() < goalAreaUpRight.x() &&
      theRobotPoseAfterPreview.translation.y() > goalAreaDownRight.y() &&
      Geometry::getIntersectionOfLines(lineToTarget,lineRight,intersectionPoint))
    {
      if (std::abs(intersectionPoint.y() - goalAreaDownRight.y()) < 10 && intersectionPoint.x() < goalAreaUpRight.x() && intersectionPoint.x() > goalAreaDownRight.x())
      {
        wayPoints.push_back(Pose2f(pi3_4, (float)goalAreaDownRight.x() - 100, (float)goalAreaDownRight.y()));
      }
    }
  }

  avoidObstacles(notAllowedInGoalArea);

  // needed bc of own goal area waypoints
  std::sort(wayPoints.begin(),wayPoints.end(),sortPoses(this));

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
  std::vector<Obstacle>::iterator first = obstacles.begin();
  for (; first != obstacles.end(); ++first)
  {
    std::vector<Obstacle>::iterator second = first;
    second++;
    for (; second != obstacles.end(); ++second)
    {
      float obstacleDistance = (first->position - second->position).norm();
      if (obstacleDistance < first->radius+second->radius && 
        !((first->type == Obstacle::ball && second->type == Obstacle::robot)
        || (second->type == Obstacle::ball && first->type == Obstacle::robot)))
      {
        first->position = (first->position+second->position)/2;
        first->radius = obstacleDistance+(first->radius+second->radius)/2;
        obstacles.erase(second);
        break;
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

      if ((notAllowedInGoalArea && Geometry::isPointInsideRectangle(goalAreaDownRight,goalAreaUpLeft,
        Vector2f((obstacle->position.x()+normalToObstacle.x()),(obstacle->position.y()+normalToObstacle.y()))))
        || (obstacle->type == Obstacle::goalPost && 
          std::abs(obstacle->position.x()+normalToObstacle.x()) > theFieldDimensions.xPosOpponentGroundline))
      {
        normalToObstacle.rotate(pi);
      }

      const Vector2f avoidancePoint = obstacle->position + normalToObstacle;

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