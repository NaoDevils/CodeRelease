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
}

void SimplePathProvider::initialize()
{
  penaltyAreaDownLeft = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea + 150);
  penaltyAreaDownRight = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea - 150);
  penaltyAreaUpRight = Vector2f(theFieldDimensions.xPosOwnPenaltyArea + 150, theFieldDimensions.yPosRightPenaltyArea - 150);
  penaltyAreaUpLeft = Vector2f(theFieldDimensions.xPosOwnPenaltyArea + 150, theFieldDimensions.yPosLeftPenaltyArea + 150);
  initialized = true;
}

void SimplePathProvider::update(Path& path)
{
  // TODO: ball next to goal post
  DECLARE_DEBUG_DRAWING("module:SimplePathProvider:obstacles", "drawingOnField");

  if (!initialized)
    initialize();
  // clear old wayPoints/obstacles
  // TODO : remember the old way
  if (theMotionRequest.motion == MotionRequest::walk && theMotionRequest.walkRequest.requestType == WalkRequest::destination && !path.wayPoints.empty())
  {
    wayPoints.clear();
    obstacles.clear();
    distanceToClosestObstacle = std::numeric_limits<float>::max();

    destWorldCoordinates = Pose2f(theRobotPoseAfterPreview + theMotionRequest.walkRequest.request);

    /*** always build new path, if only to compare with old one ***/
    // add wayPoints from robots, but only if situation is not critical defensively
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

    path.nearestObstacle = distanceToClosestObstacle;
    path.nearestObstaclePosition = nearestObstaclePosition;
    path.nearestObstacleType = nearestObstacleType;
    path.length = 0.f;
    for (size_t i = 0; i < path.wayPoints.size() - 1; i++)
    {
      path.length += (path.wayPoints[i + 1].translation - path.wayPoints[i].translation).norm();
    }
  }
  else
  {
    path.reset();
    path.nearestObstacle = distanceToClosestObstacle;
    path.nearestObstaclePosition = nearestObstaclePosition;
    path.nearestObstacleType = nearestObstacleType;
    path.wayPoints.push_back(theRobotPoseAfterPreview);
    path.wayPoints.push_back(theRobotPoseAfterPreview);
    path.length = 0.f;
  }
}

void SimplePathProvider::handleRobots()
{
  std::vector<RobotMapEntry>::const_iterator robot = theRobotMap.robots.begin();
  std::vector<RobotMapEntry>::const_iterator end = theRobotMap.robots.end();
  for (; robot != end; ++robot)
  {
    const Vector2f& trans = robot->pose.translation;
    float distance = (trans - theRobotPose.translation).norm() - (robot->robotType == RobotEstimate::teammateRobot ? teamRobotInfluenceRadius : robotInfluenceRadius);
    if (distance < distanceToClosestObstacle)
    {
      distanceToClosestObstacle = distance;
      nearestObstaclePosition = Vector2f(trans);
      nearestObstacleType = Path::robot;
    }
    if (distance < 2500)
    {
      obstacles.push_back(Obstacle(robot->pose.translation.x(),
          robot->pose.translation.y(),
          ((robot->robotType == RobotEstimate::teammateRobot) ? teamRobotInfluenceRadius : robotInfluenceRadius),
          Path::robot,
          (robot->robotType == RobotEstimate::teammateRobot)));
    }
  }
}

void SimplePathProvider::handleBall()
{
  if ((theBallSymbols.timeSinceLastSeen < 2000
          || (theBallSymbols.timeSinceLastSeenByTeam < 2000 && Transformation::fieldToRobot(theRobotPoseAfterPreview, theBallSymbols.ballPositionField).x() < 0))
      && (theGameInfo.state == STATE_PLAYING || theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
      && theBallchaser.kickType != MotionRequest::dribble && theBallSymbols.avoidBall == true)
  {
    const Vector2f& trans = Transformation::robotToField(theRobotPoseAfterPreview, theBallModelAfterPreview.estimate.position);
    float distance = theBallModelAfterPreview.estimate.position.norm();
    if (distance < distanceToClosestObstacle)
    {
      distanceToClosestObstacle = distance;
      nearestObstaclePosition = Vector2f(trans);
      nearestObstacleType = Path::ball;
    }
    float alpha = std::abs(Angle::normalize(destWorldCoordinates.translation.angle() - (destWorldCoordinates.translation - theBallSymbols.ballPositionField).angle())) / pi;
    float influenceRadius = ballInfluenceRadius + ballInfluenceRadius * alpha;
    // if free kick, use wide circle around ball
    isGoalieInOwnPenaltyArea = theBehaviorData.role == BehaviorData::keeper
        && std::abs(theRobotPoseAfterPreview.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea + (isGoalieInOwnPenaltyArea ? 100.f : 0.f)
        && theRobotPoseAfterPreview.translation.x() < theFieldDimensions.xPosOwnPenaltyArea + (isGoalieInOwnPenaltyArea ? 100.f : 0.f);
    if (theGameInfo.setPlay != SET_PLAY_NONE && !theGameSymbols.ownKickOff && !isGoalieInOwnPenaltyArea)
      influenceRadius = 150.f;
    obstacles.push_back(Obstacle(trans.x(), trans.y(), influenceRadius, Path::ball));
  }
}

void SimplePathProvider::handleStaticObstacles()
{
  Vector2f oppGoalPostLeft(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
  Vector2f oppGoalPostRight(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
  Vector2f ownGoalPostLeft(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  Vector2f ownGoalPostRight(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  /*Vector2f oppGoalPostLeftBack(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosLeftGoal);
  Vector2f oppGoalPostRightBack(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosRightGoal);
  Vector2f ownGoalPostLeftBack(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosLeftGoal);
  Vector2f ownGoalPostRightBack(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosRightGoal);*/

  obstacles.push_back(Obstacle(oppGoalPostLeft.x(), oppGoalPostLeft.y(), goalPostInfluenceRadius, Path::goalPost));
  obstacles.push_back(Obstacle(oppGoalPostRight.x(), oppGoalPostRight.y(), goalPostInfluenceRadius, Path::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostLeft.x(), ownGoalPostLeft.y(), goalPostInfluenceRadius, Path::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostRight.x(), ownGoalPostRight.y(), goalPostInfluenceRadius, Path::goalPost));
  /*obstacles.push_back(Obstacle(oppGoalPostLeftBack.x(), oppGoalPostLeftBack.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(oppGoalPostRightBack.x(), oppGoalPostRightBack.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostLeftBack.x(), ownGoalPostLeftBack.y(), goalPostInfluenceRadius, Obstacle::goalPost));
  obstacles.push_back(Obstacle(ownGoalPostRightBack.x(), ownGoalPostRightBack.y(), goalPostInfluenceRadius, Obstacle::goalPost));*/
}

void SimplePathProvider::buildPath()
{
  if (theGameSymbols.avoidCenterCircle)
  {
    float circleSafetyRadius = theFieldDimensions.centerCircleRadius + centerCircleInfluenceRadius;
    if (destWorldCoordinates.translation.norm() < circleSafetyRadius)
    {
      float rotation = Angle::normalize(theRobotPoseAfterPreview.translation.angle() + pi);
      destWorldCoordinates = Pose2f(rotation, Vector2f(theRobotPoseAfterPreview.translation).normalize(circleSafetyRadius));
    }
    else
    {
      obstacles.push_back(Obstacle(0, 0, centerCircleInfluenceRadius + theFieldDimensions.centerCircleRadius, Path::centerCircle));
    }
  }

  if (theGameInfo.setPlay != SET_PLAY_NONE && !theGameSymbols.ownKickOff)
  {
    obstacles.push_back(Obstacle(theBallSymbols.ballPositionField.x(), theBallSymbols.ballPositionField.y(), setPlayInfluenceRadius, Path::setPlayCircle));
  }

  // all obstacles should be added by now, update distanceToClosestObstacle.
  for (const auto& obstacle : obstacles)
  {
    float distance = std::abs((obstacle.position - theRobotPoseAfterPreview.translation).norm() - obstacle.radius);
    if (distance < distanceToClosestObstacle)
    {
      distanceToClosestObstacle = distance;
      nearestObstaclePosition = obstacle.position;
      nearestObstacleType = obstacle.type;
    }
  }

  wayPoints.push_back(theRobotPoseAfterPreview);

  bool notAllowedInPenaltyArea = false;
  // handle penalty area
  // only defender roles or ball chaser are allowed in penalty area
  //bool ballChaserInPenaltyArea = theBallchaser.optPosition.translation.x() < theFieldDimensions.xPosOwnPenaltyArea + 200.f;
  if (!theGameSymbols.allowedInPenaltyArea)
  {
    notAllowedInPenaltyArea = true;
    avoidRectangularArea(penaltyAreaDownLeft, penaltyAreaUpLeft, penaltyAreaUpRight, penaltyAreaDownRight);
  }

  if (wayPoints.size() > 1
      && ((wayPoints[1].translation - theRobotPoseAfterPreview.translation).norm() < 150
          || (wayPoints[1].translation - destWorldCoordinates.translation).norm() > (destWorldCoordinates.translation - theRobotPoseAfterPreview.translation).norm()))
    wayPoints.erase(wayPoints.begin() + 1);

  // do not cross through goal sides
  // if robot is left of goal side and target is right of goal side
  // or robot is inside goal side and target is left/right of goal side
  // or robot is right of goal side and target is left of goal side
  const float robotY = theRobotPoseAfterPreview.translation.y();
  const float goalLeftY = theFieldDimensions.yPosLeftGoal;
  const float targetY = destWorldCoordinates.translation.y();
  if (theRobotPoseAfterPreview.translation.x() < theFieldDimensions.xPosOwnGroundline)
  {
    bool insertLeftGoalPost = false, insertRightGoalPost = false;

    // left goal side
    if ((robotY > goalLeftY && targetY < goalLeftY) || (robotY < goalLeftY && targetY > goalLeftY))
    {
      Geometry::Line lineToTarget(theRobotPoseAfterPreview.translation, destWorldCoordinates.translation - theRobotPoseAfterPreview.translation);
      Geometry::Line leftGoalNet(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal), Vector2f(-1.f, 0.f));
      Vector2f intersectionPoint;

      if (Geometry::getIntersectionOfLines(lineToTarget, leftGoalNet, intersectionPoint) && intersectionPoint.x() < theFieldDimensions.xPosOwnGroundline)
        insertLeftGoalPost = true;
    }
    // right goal side
    if ((robotY < -goalLeftY && targetY > -goalLeftY) || (robotY > -goalLeftY && targetY < -goalLeftY))
    {
      Geometry::Line lineToTarget(theRobotPoseAfterPreview.translation, destWorldCoordinates.translation - theRobotPoseAfterPreview.translation);
      Geometry::Line rightGoalNet(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal), Vector2f(-1.f, 0.f));
      Vector2f intersectionPoint;
      if (Geometry::getIntersectionOfLines(lineToTarget, rightGoalNet, intersectionPoint) && intersectionPoint.x() < theFieldDimensions.xPosOwnGroundline)
        insertRightGoalPost = true;
    }

    // insert right goal post first if on right side
    // insert left goal post first if on left side
    if (theRobotPoseAfterPreview.translation.y() > 0.f)
    {
      if (insertLeftGoalPost)
        wayPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost + 200.f, theFieldDimensions.yPosLeftGoal + 200.f));
      if (insertRightGoalPost)
        wayPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost + 200.f, theFieldDimensions.yPosRightGoal - 200.f));
    }
    else
    {
      if (insertRightGoalPost)
        wayPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost + 200.f, theFieldDimensions.yPosRightGoal - 200.f));
      if (insertLeftGoalPost)
        wayPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGoalPost + 200.f, theFieldDimensions.yPosLeftGoal + 200.f));
    }
  }

  avoidObstacles(notAllowedInPenaltyArea);

  // needed bc of own goal area waypoints (not anymore?)
  //   std::sort(wayPoints.begin(),wayPoints.end(),sortPoses(this));

  // if block way to goal is requested (while going to ball), move between ball and goal before actually going to ball
  // useful for defence
  if (theBallchaser.blockWayToGoalOnApproach && wayPoints.size() <= 2 && theMotionRequest.walkRequest.request.translation.norm() > 0.f - (wasBlockingWayToGoal ? 250.f : 0)) //750.f
  {
    Vector2f destRel = destWorldCoordinates.translation - theRobotPoseAfterPreview.translation;
    Vector2f goalToBall = theBallSymbols.ballPositionFieldPredicted - Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f vectorToBall = theBallSymbols.ballPositionField - theRobotPose.translation;
    Angle angleGoalToBall = goalToBall.angle();
    Angle angleDifference = std::abs(Angle::normalize(destRel.angle() - angleGoalToBall));
    Angle angleHysteresis = (wasBlockingWayToGoal ? 10_deg : 0_deg);
    float distanceHysteresis = (wasBlockingWayToGoal ? 100.f : 0.f);
    if ((std::abs(angleGoalToBall) < 60_deg + angleHysteresis //60_deg
            && std::abs(angleDifference) > 20_deg - angleHysteresis //20_deg
            && std::abs(angleDifference) < 45_deg + angleHysteresis //30_deg
            && std::abs(vectorToBall.y()) > 300 - distanceHysteresis)
        //TODO: check condition
        /*&& std::abs(Angle::normalize(angleGoalToBall - theRobotPoseAfterPreview.rotation)) < 30_deg + angleHysteresis*/)
    {
      goalToBall.normalize(destRel.norm() * std::cos(angleDifference));
      Pose2f newWP(angleGoalToBall, destWorldCoordinates.translation - goalToBall);
      float distanceToNewWP = (newWP.translation - theRobotPoseAfterPreview.translation).norm();
      if ((distanceToNewWP < 700.f + (wasBlockingWayToGoal ? 150.f : 0.f)) && (distanceToNewWP > 350.f - (wasBlockingWayToGoal ? 150.f : 0.f)))
      {
        //if robot approaches ball from the side remove last waypoint since robot is moving to the center anyway
        if (inBallApproach)
        {
          wayPoints.pop_back();
        }
        wayPoints.push_back(newWP);
        wasBlockingWayToGoal = true;
      }
    }
  }
  else
    wasBlockingWayToGoal = false;


  wayPoints.push_back(destWorldCoordinates);
}

void SimplePathProvider::avoidRectangularArea(const Vector2f& bottomLeft, const Vector2f& topLeft, const Vector2f& topRight, const Vector2f& bottomRight)
{
  // used for goal/penalty area avoidance
  const Geometry::Line lineToTarget(theRobotPoseAfterPreview.translation, destWorldCoordinates.translation - theRobotPoseAfterPreview.translation);
  const Geometry::Line lineFront(topLeft, topRight - topLeft);
  const Geometry::Line lineLeft(topLeft, bottomLeft - topLeft);
  const Geometry::Line lineRight(topRight, bottomRight - topRight);
  const Geometry::Line lineBack(bottomLeft, bottomRight - bottomLeft);

  Vector2f intersectionPoint(0, 0);

  // below penalty area line?
  if (destWorldCoordinates.translation.x() < topLeft.x())
  {
    // if the robot is below the penalty area line and has to cross sides
    if (theRobotPoseAfterPreview.translation.x() < topLeft.x() - 100)
    {
      // robot is on right side
      if (theRobotPoseAfterPreview.translation.y() < topRight.y() - 100 && destWorldCoordinates.translation.y() > topLeft.y() - 100)
      {
        wayPoints.push_back(Pose2f(pi_2, topRight));
        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - topLeft).angle(), topLeft));
      }
      // robot is on left side
      else if (theRobotPoseAfterPreview.translation.y() > topLeft.y() + 100 && destWorldCoordinates.translation.y() < topRight.y() + 100)
      {
        wayPoints.push_back(Pose2f(-pi_2, topLeft));
        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - topRight).angle(), topRight));
      }
    }
    // robot above penalty area line? Threshold to make sure robot reaches position
    else if (theRobotPoseAfterPreview.translation.x() > topLeft.x() + 100)
    {
      // would cross penalty area on the way to target (left)
      if (destWorldCoordinates.translation.y() > 0 && theRobotPoseAfterPreview.translation.y() < topLeft.y() + 100
          && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint) && intersectionPoint.y() < topLeft.y())
        wayPoints.push_back(Pose2f(pi3_4, topLeft));
      // would cross penalty area on the way to target (right)
      else if (destWorldCoordinates.translation.y() < 0 && theRobotPoseAfterPreview.translation.y() > topRight.y() - 100
          && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint) && intersectionPoint.y() > topRight.y())
        wayPoints.push_back(Pose2f(-pi3_4, topRight));
    }
  }
  // if the robot is below the penalty area line
  else if (theRobotPoseAfterPreview.translation.x() < topLeft.x() + 100)
  {
    // robot is on right side and would cross penalty area
    if (theRobotPoseAfterPreview.translation.y() < topRight.y() + 100 && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint)
        && intersectionPoint.y() > topRight.y())
    {
      wayPoints.push_back(Pose2f((destWorldCoordinates.translation - topRight).angle(), topRight));
    }
    // robot is on left side and would cross penalty area
    else if (theRobotPoseAfterPreview.translation.y() > topLeft.y() - 100 && Geometry::getIntersectionOfLines(lineToTarget, lineFront, intersectionPoint)
        && intersectionPoint.y() < topLeft.y())
    {
      wayPoints.push_back(Pose2f((destWorldCoordinates.translation - topLeft).angle(), topLeft));
    }
  }
}

void SimplePathProvider::avoidObstacles(bool notAllowedInPenaltyArea)
{
  // TODO: create wall were player may shoot ball and avoid that..??!!
  std::sort(obstacles.begin(), obstacles.end(), sortObstacles(this));
  int i = 0;

  // TODO: memory!
  // TODO: should do this while building path..
  // TODO: check missing if obstacle is actually relevant
  if (mergeObstacles)
  {
    // TODO: move to params
    const float maxObstacleDistanceForMerging = 500.f;
    for (size_t first = 0; first < obstacles.size(); first++)
    {
      for (size_t second = first + 1; second < obstacles.size(); second++)
      {
        float obstacleDistance = (obstacles[first].position - obstacles[second].position).norm();
        if (obstacleDistance < std::min(maxObstacleDistanceForMerging, obstacles[first].radius + obstacles[second].radius)
            && !(obstacles[first].type == Path::ball || obstacles[second].type == Path::ball)
            && !((obstacles[first].type == Path::goalPost && obstacles[second].type == Path::robot) || (obstacles[second].type == Path::goalPost && obstacles[first].type == Path::robot)))
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
    if (obstacle->type == Path::ball)
      ballIsObstacle = true;
    i++;
    CIRCLE("module:SimplePathProvider:obstacles", obstacle->position.x(), obstacle->position.y(), obstacle->radius, 5, Drawings::solidPen, ColorRGBA::yellow, Drawings::noBrush, ColorRGBA::yellow);
    DRAWTEXT("module:SimplePathProvider:obstacles", obstacle->position.x(), obstacle->position.y(), 30, ColorRGBA::yellow, i);
    const Pose2f& currentWayPoint = ((wayPoints.size() < 2) ? (Pose2f&)theRobotPose : wayPoints.back());
    const Vector2f vectorToTarget = (destWorldCoordinates.translation - currentWayPoint.translation);
    const float distance = vectorToTarget.norm();
    const Vector2f vectorToObstacle = (obstacle->position - currentWayPoint.translation);
    const float distanceToObstacle = vectorToObstacle.norm();

    const float angleToTargetWC = vectorToTarget.angle();
    const float angleToObstacleWC = vectorToObstacle.angle();
    float angleDiff = Angle::normalize(angleToTargetWC - angleToObstacleWC);
    float maxAngleDiff = pi_2 * 0.8f;
    if ((obstacle->type == Path::robot && obstacle->isTeamMate) || obstacle->type == Path::centerCircle || obstacle->type == Path::setPlayCircle)
      maxAngleDiff = pi_2 * 1.5f;

    // second clause?
    if ((std::abs(angleDiff) < maxAngleDiff && distanceToObstacle < distance)
        || (obstacle->type == Path::ball && std::abs(Angle::normalize(angleToObstacleWC - destWorldCoordinates.rotation)) > pi_2 * 0.7f && distance > 150 && distanceToObstacle - 50 < distance))
    {
      // might be dangerous
      /*
      if (std::abs(Angle::normalize(angleToObstacleWC-destWorldCoordinates.rotation)) < pi_2 && obstacle->type == Obstacle::ball)
      {
        obstacle->radius = ballInfluenceRadius;
        obstacle->position = Transformation::robotToField(theRobotPoseAfterPreview, theBallModelAfterPreview.estimate.position);
      }
      */
      CIRCLE("module:SimplePathProvider:obstacles", obstacle->position.x(), obstacle->position.y(), obstacle->radius, 5, Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);
      Vector2f normalToObstacle = vectorToObstacle;

      if (obstacle->type == Path::ball)
      {
        Angle approachAngleWC = Angle::normalize(vectorToTarget.angle() - destWorldCoordinates.rotation);
        if (avoidBallLeft && approachAngleWC > -160 && approachAngleWC < 0)
          avoidBallLeft = false;
        else if (!avoidBallLeft && approachAngleWC < 160_deg && approachAngleWC > 0)
          avoidBallLeft = true;
        if (avoidBallLeft)
          normalToObstacle.rotate(110_deg);
        else
          normalToObstacle.rotate(-110_deg);
      }
      else
      {
        if (std::abs(angleDiff) > pi_4)
          angleDiff = pi_4 * sgn(angleDiff);
        if (angleDiff > 0)
          normalToObstacle.rotate(pi_2 + std::abs(angleDiff));
        else
          normalToObstacle.rotate(-pi_2 - std::abs(angleDiff));
      }

      normalToObstacle.normalize(obstacle->radius);

      Vector2f avoidancePoint = obstacle->position + normalToObstacle;

      const Vector2f goalCenter(theFieldDimensions.xPosOwnGroundline, 0.f);
      if ((notAllowedInPenaltyArea && Geometry::isPointInsideRectangle(penaltyAreaDownLeft, penaltyAreaUpRight, avoidancePoint)
              && (goalCenter - (obstacle->position + normalToObstacle)).norm() < (goalCenter - (obstacle->position - normalToObstacle)).norm())
          || (obstacle->type == Path::goalPost && std::abs(obstacle->position.x() + normalToObstacle.x()) > theFieldDimensions.xPosOpponentGroundline))
      {
        normalToObstacle.rotate(pi);
      }

      avoidancePoint = obstacle->position + normalToObstacle;

      float distObstacleToTargetLine = std::asin(std::min<float>(std::abs(angleDiff), 1.f)) * distanceToObstacle;

      if (normalToObstacle.norm() > distObstacleToTargetLine)
      {
        // okay, that thing is really blocking my path
        CIRCLE("module:SimplePathProvider:obstacles", obstacle->position.x(), obstacle->position.y(), obstacle->radius, 5, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);

        // if inside obstacle influence radius, try to get out roughly in direction to target
        if (avoidRobotCrashes && (theRobotPose.translation - obstacle->position).norm() <= obstacle->radius)
        {
          // back out vector
          Vector2f meToObstacle = (theRobotPose.translation - obstacle->position);
          // rotate back out vector towards target to at least move a little close to target
          Angle backOutVectorToTargetVector = Angle::normalize((destWorldCoordinates.translation - obstacle->position).angle() - meToObstacle.angle());
          meToObstacle.rotate(-sgn(backOutVectorToTargetVector) * std::min<Angle>(std::abs(backOutVectorToTargetVector), 30_deg));
          avoidancePoint = obstacle->position + meToObstacle.normalize(obstacle->radius * 1.2f);
        }


        wayPoints.push_back(Pose2f((destWorldCoordinates.translation - avoidancePoint).angle(), avoidancePoint));
      }

      LINE("module:SimplePathProvider:obstacles", obstacle->position.x(), obstacle->position.y(), avoidancePoint.x(), avoidancePoint.y(), 4, Drawings::solidPen, ColorRGBA::blue);
    }

    /*if (obstacle->type == Obstacle::ball)
    {
      Pose2f posBeforeTarget = destWorldCoordinates;
      posBeforeTarget.translate(300.f, 0.f);
      Angle angleToBeforeTarget = Geometry::angleTo(posBeforeTarget, currentWayPoint.translation);
      Angle angleToObstacle = Geometry::angleTo(Pose2f(destWorldCoordinates.rotation, obstacle->position), currentWayPoint.translation);

      if (std::abs(angleToBeforeTarget) < 180_deg - ballApproachCone/2 || std::abs(angleToObstacle) < 90_deg)
      {
        Pose2f posBehindTarget = destWorldCoordinates;
        posBehindTarget.translate(-120.f, 0.f);
        wayPoints.push_back(posBehindTarget);
        inBallApproach = true;
      }
      else
        inBallApproach = false;
    }*/
  }
}

bool SimplePathProvider::isOldPathFine(Path& path)
{
  if (!stablizePath || path.wayPoints.size() != wayPoints.size())
    return false;
  /*** check for length and if (new) obstacle is in way ***/
  float oldLength = 0.f;
  float newLength = 0.f;
  if ((path.wayPoints[1].translation - path.wayPoints[0].translation).norm() < 250)
    return false;
  for (unsigned int i = 1; i < path.wayPoints.size(); i++)
  {
    Vector2f vToNextWayPoint = path.wayPoints[i].translation - path.wayPoints[i - 1].translation;
    for (auto& robot : theRobotMap.robots)
    {
      if ((robot.pose.translation - path.wayPoints[i].translation).norm() < 400)
        return false;
    }
    /*for (auto const& obstacle : obstacles)
    {
      if (obstacle.type != Obstacle::robot)
      {
        if (distancePointToVector(obstacle.position, vToNextWayPoint, path.wayPoints[i - 1].translation) < 150)
          return false;
      }
    }*/
    for (unsigned int i = 1; i < path.wayPoints.size() - 1; i++)
    {
      if ((path.wayPoints[i].translation - wayPoints[i].translation).norm() < 100)
        path.wayPoints[i] = wayPoints[i];
    }
    oldLength += vToNextWayPoint.norm();
  }
  for (unsigned int i = 1; i < wayPoints.size(); i++)
    newLength += (wayPoints[i].translation - wayPoints[i - 1].translation).norm();
  return (oldLength - 500 < newLength);
}

MAKE_MODULE(SimplePathProvider, pathPlanning)
