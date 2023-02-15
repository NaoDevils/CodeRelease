#include "BallchaserProvider.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Ballchaser/NoDeadlockObjective.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackVeryLong.h>

BallchaserProvider::BallchaserProvider() : regularPlayObjectivesManager(logger)
{
  //regularPlayObjectivesManager.add(std::make_unique<TestObjective>(this, logger));
  regularPlayObjectivesManager.add(std::make_unique<GoalObjective>(this, logger));
  regularPlayObjectivesManager.add(std::make_unique<OneVsOneObjective>(this, logger));
  regularPlayObjectivesManager.add(std::make_unique<MoveObjective>(this, logger));
  regularPlayObjectivesManager.add(std::make_unique<NoDeadlockObjective>(this, logger));

  kickOffKicks.push_back(std::make_unique<KickHack>());
  kickOffKicks.push_back(std::make_unique<KickHackLong>());
  kickOffKicks.push_back(std::make_unique<KickHackVeryLong>());

  kickInKicks.push_back(std::make_unique<KickHack>());
  kickInKicks.push_back(std::make_unique<KickHackLong>());
  kickInKicks.push_back(std::make_unique<KickHackVeryLong>());

  cornerKicks.push_back(std::make_unique<KickHack>());
  cornerKicks.push_back(std::make_unique<KickHackLong>());
  cornerKicks.push_back(std::make_unique<KickHackVeryLong>());

  goalKick_Kicks.push_back(std::make_unique<KickHack>());
  goalKick_Kicks.push_back(std::make_unique<KickHackLong>());
  goalKick_Kicks.push_back(std::make_unique<KickHackVeryLong>());

  penaltyKicks.push_back(std::make_unique<SlowLongKick>());
}

void BallchaserProvider::update(Ballchaser& ballchaser)
{
  DECLARE_DEBUG_DRAWING(DRAW_KICK_MANAGER_ACTIVATE_RANGE_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_RANGE_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_DANGER_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:BallchaserProvider:KickManager:Blocked", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:BallchaserProvider:KickManager:TargetFree", "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICK_TARGET_AREA, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_MIN_WIDTH, "drawingOnField");

  // Reset
  ballchaser.kickType = MotionRequest::KickType::walkKick;
  ballchaser.walkKickType = WalkRequest::StepRequest::none;
  danger = Danger::IMPOSSIBLE;
  // Reset Logs
  ballchaser.log_currState = "None";
  ballchaser.log_currObj = "None";
  ballchaser.log_danger = "None";
  ballchaser.log_kickName = "None";
  logger.clear();

  // Update
  updateVariables();
  setPlayKickManager.update(ballPosition, theRobotPoseAfterPreview, theWalkingEngineParams);

  // Set
  ballchaser.stopAtTarget = false;
  decide(ballchaser, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
  ballchaser.previewArrival = previewArrival;
  // Set Logs
  ballchaser.log_toBallDistance = std::to_string(Geometry::distance(theRobotPoseAfterPreview.translation, ballPosition));
  ballchaser.log_obj1 = logger.get(0);
  ballchaser.log_obj2 = logger.get(1);
  ballchaser.log_obj3 = logger.get(2);
  ballchaser.log_obj4 = logger.get(3);
  ballchaser.log_obj5 = logger.get(4);
}

void BallchaserProvider::stateReady_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  PositionUtils::setPosition(positioningSymbols, -500.f, 0);
  PositionUtils::turnToPosition(positioningSymbols, kickOffPosition);
  ThresholdUtils::setThreshholdsLow(positioningSymbols);
}

void BallchaserProvider::statePlaying_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  ExecutableKicks executableKicks =
      setPlayKickManager.getExecutableKicks(theRobotPoseAfterPreview, ballPosition, KickUtils::unpack(kickOffKicks), theFieldDimensions, theHeatMapCollection, theRobotMap)
          .filterTooFarBack(-500.f)
          .filterOutside(theFieldDimensions)
          .filterBlocked(theFieldDimensions, theRobotMap)
          .reduceToBest(0.f, 0.f, 1.f, 0.f, 0.5f, 0.f, 0.f, 1.f, 0.f, -0.5f, -0.5f, kickWithLeft, theFieldDimensions, theHeatMapCollection, theRobotMap);
  if (executableKicks.hasBest())
  {
    setPlayKickManager.kickTo(positioningSymbols, executableKicks);
  }
  else
  {
    // TODO Warning only if ballchaser if (theRoleSymbols.role == Ballchaser)
    //OUTPUT_WARNING("KickOff not possible!");
  }
}

void BallchaserProvider::stateReady_kickOff_opponent(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  positioningSymbols.optPosition.translation.x() = -theFieldDimensions.centerCircleRadius - theBehaviorConfiguration.behaviorParameters.kickOffLineDistance;
  positioningSymbols.optPosition.translation.y() = 0;
  positioningSymbols.optPosition.rotation = 0;
  positioningSymbols.thresholdXFront = 50;
  positioningSymbols.thresholdXBack = 50;
  positioningSymbols.thresholdY = 25;
}

float BallchaserProvider::goalKick_own(Ballchaser& positioningSymbols, bool left)
{
  const Vector2f waitPos = Vector2f(theFieldDimensions.xPosOwnGoalArea - 300.f, left ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
  const float minX = theFieldDimensions.xPosOwnGoalArea + 300.f;
  freeKick(positioningSymbols, waitPos, minX, 0.5f);
  return 0.f;
}

float BallchaserProvider::goalKick_opponent(Ballchaser& positioningSymbols, bool left)
{
  const Vector2f targetToCover{theFieldDimensions.xPosOwnGroundline,
      left ? (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosRightGoal) / 2.f : (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosLeftGoal) / 2.f};
  const Vector2f ballToOwnGoal{targetToCover - ballPosition};
  Vector2f position{ballPosition + ballToOwnGoal.normalized(oppSetPlayBallDistance)};
  position.x() = std::max(position.x(), theFieldDimensions.xPosOwnGroundline + 300.f);

  positioningSymbols.optPosition.translation = position;
  positioningSymbols.optPosition.rotation = (ballPosition - position).angle();
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_own(Ballchaser& positioningSymbols)
{
  regularPlayObjectivesManager.executeObjective(positioningSymbols);
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_opponent(Ballchaser& positioningSymbols)
{
  goalKick_opponent(positioningSymbols, ballOnLeftSide); // TODO
  return 0.f;
}

float BallchaserProvider::cornerKick_own(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  Vector2f waitPos = Vector2f(theFieldDimensions.xPosOpponentGroundline + 200.f, left ? theFieldDimensions.yPosLeftSideline + 200.f : theFieldDimensions.yPosRightSideline - 200.f);
  const float minX = theFieldDimensions.xPosOpponentPenaltyArea - 1000.f;
  freeKick(positioningSymbols, waitPos, minX, 0.2f);
  return 0.f;
}

float BallchaserProvider::cornerKick_opponent(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  Vector2f kickerPose;
  int nmbrDetected = 0;
  Vector2f ballPosition = theBallSymbols.ballPositionField;
  for (RobotMapEntry robot : theRobotMap.robots)
  {
    Vector2f robotPose = robot.pose.translation;
    if ((robotPose - ballPosition).norm() < 400.f && (abs(robotPose.y()) >= abs(theFieldDimensions.yPosLeftSideline) + 60.f && robotPose.x() < theFieldDimensions.xPosOwnGroundline + 20.f))
    {
      kickerPose = robotPose;
      nmbrDetected++;
    }
  }
  if (nmbrDetected == 1) //TODO: time hysteresis
  {
    Vector2f kickerToBall = ballPosition - kickerPose;
    kickerToBall.angle();
    float distanceFromKickerToBall = (kickerToBall).norm();
    Vector2f targetPos = ballPosition + kickerToBall * oppSetPlayBallDistance / distanceFromKickerToBall;
    positioningSymbols.optPosition.translation = targetPos;
    PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  }
  else
  {
    goalKick_opponent(positioningSymbols, left);
  }

  return 0.f;
}

float BallchaserProvider::kickIn_own(Ballchaser& positioningSymbols, bool left) //TODO: Wait for 10 seconds if time permits before shooting. Longer if we are winning. RoleProvider. 1vs1.
{
  Vector2f waitPos = Vector2f(theBallSymbols.ballPositionField.x(), left ? theFieldDimensions.yPosLeftSideline + 300.f : theFieldDimensions.yPosRightSideline - 300.f);
  const float minX = MathUtils::clamp_f(ballPosition.x() - 500.f, theFieldDimensions.xPosOwnPenaltyArea + 200.f, theFieldDimensions.xPosOpponentGoalArea - 200.f);
  freeKick(positioningSymbols, waitPos, minX, 0.3f);
  return 0.f;
}

float BallchaserProvider::kickIn_opponent(Ballchaser& positioningSymbols, bool left)
{
  Vector2f kickerPose;
  int nmbrDetected = 0;
  Vector2f ballPosition = theBallSymbols.ballPositionField;
  for (RobotMapEntry robot : theRobotMap.robots)
  {
    Vector2f robotPose = robot.pose.translation;
    if ((robotPose - ballPosition).norm() < 400.f && abs(robotPose.y()) >= abs(theFieldDimensions.yPosLeftSideline) - 20.f)
    {
      kickerPose = robotPose;
      nmbrDetected++;
    }
  }
  if (nmbrDetected == 1) //TODO: time hysteresis
  {
    Vector2f kickerToBall = ballPosition - kickerPose;
    kickerToBall.angle();
    float distanceFromKickerToBall = (kickerToBall).norm();
    Vector2f targetPos = ballPosition + kickerToBall * oppSetPlayBallDistance / distanceFromKickerToBall;
    positioningSymbols.optPosition.translation = targetPos;
    PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  }
  else
  {
    goalKick_opponent(positioningSymbols, left);
  }

  return 0.f;
}

float BallchaserProvider::stateReady_penaltyKick_own(Ballchaser& ballchaser)
{
  const Vector2f penaltyMarkPosition = {theFieldDimensions.xPosOpponentPenaltyMark, 0.f}; // TODo put Vector into theFieldDimensions
  const Vector2f position = {penaltyMarkPosition.x() - 200.f, penaltyMarkPosition.y()};
  ThresholdUtils::setThreshholdsMedium(ballchaser);
  PositionUtils::setPosition(ballchaser, position);
  PositionUtils::turnToPosition(ballchaser, position);
  return 0.f;
}

float BallchaserProvider::statePlaying_penaltyKick_own(Ballchaser& ballchaser)
{
  const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};
  ExecutableKicks executableKicks =
      setPlayKickManager
          .getExecutableKicks(theRobotPoseAfterPreview, {ballPosition, leftGoalPostPosition, rightGoalPostPosition, DistanceRequirement::mustFurther}, 10, KickUtils::unpack(penaltyKicks), theFieldDimensions, theHeatMapCollection, theRobotMap)
          .filterBlocked(theFieldDimensions, theRobotMap)
          .reduceToBest(0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, kickWithLeft, theFieldDimensions, theHeatMapCollection, theRobotMap);
  if (executableKicks.hasBest())
  {
    setPlayKickManager.kickTo(ballchaser, executableKicks);
  }
  else
  {
    // TODO Warning only if Ballchaser
  }
  return 0.f;
}

float BallchaserProvider::stateReady_penaltyKick_opponent(Ballchaser& positioningSymbols)
{
  positioningSymbols.optPosition.translation.x() = theFieldDimensions.xPosOwnPenaltyArea + penaltyKickOpponentPositionX;
  positioningSymbols.optPosition.translation.y() = theFieldDimensions.yPosKickOffPoint + penaltyKickOpponentPositionY;
  const Vector2f penaltyCross{theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosKickOffPoint};
  positioningSymbols.optPosition.rotation = (penaltyCross - positioningSymbols.optPosition.translation).angle();
  return 0.f;
}

void BallchaserProvider::regularPlay(Ballchaser& positioningSymbols)
{
  /* TODO
  if (goalKeepegehtZumBall und verbündeter Roboter ist nahe Ball)
  {
    stelle Tor zu; mit Kopf zum ball;
    return;
  }
  */
  updateDanger(positioningSymbols);
  regularPlayObjectivesManager.executeObjective(positioningSymbols);
}

bool BallchaserProvider::waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition)
{
  const int MAX_WAIT_TIME = 8; // TODO Constant
  const int MAX_LOOSING_WAIT_TIME = 5;
  const float WAIT_POSE_THRESHOLD = 100.f;
  bool winning = theOwnTeamInfo.score > theOpponentTeamInfo.score;
  int timeRemaining = 30 /* TODO Constant */ - theGameSymbols.timeSinceSetPlayStarted;

  if (timeRemaining >= 29)
  {
    timeArrivedInSetPlayWaiting = 0;
  }

  if (timeRemaining > MAX_WAIT_TIME && (winning || timeArrivedInSetPlayWaiting - timeRemaining < MAX_LOOSING_WAIT_TIME))
  {
    // TODO Head Control
    positioningSymbols.stopAtTarget = true;

    PositionUtils::setPosition(positioningSymbols, waitingPosition);
    PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
    if (timeArrivedInSetPlayWaiting == 0 && (theRobotPose.translation - waitingPosition).norm() < WAIT_POSE_THRESHOLD)
    {
      timeArrivedInSetPlayWaiting = timeRemaining;
    }
    return true;
  }
  positioningSymbols.stopAtTarget = false;
  return false;
}

void BallchaserProvider::freeKick(Ballchaser& ballchaser, const Vector2f& waitPosition, const float minX, const float opponentGoalHeatFactor)
{
  if (waitInOwnSetPlay(ballchaser, waitPosition))
  {
    return;
  }
  ExecutableKicks executableKicks =
      setPlayKickManager.getExecutableKicks(theRobotPoseAfterPreview, ballPosition, KickUtils::unpack(kickInKicks), theFieldDimensions, theHeatMapCollection, theRobotMap)
          .filterTooFarBack(minX)
          .filterOutside(theFieldDimensions) // TODO Also filter opponent Goal!!!
          .filterBlocked(theFieldDimensions, theRobotMap)
          .reduceToBest(0.f, 0.f, 0.5f, 0.f, opponentGoalHeatFactor, 0.f, 0.1f, 1.f, 0.f, -0.5f, -0.5f, kickWithLeft, theFieldDimensions, theHeatMapCollection, theRobotMap);
  if (executableKicks.hasBest())
  {
    setPlayKickManager.kickTo(ballchaser, executableKicks);
  }
  else
  {
    // TODO Nur ausgeben wenn ballchaser
    //OUTPUT_WARNING("Free kick not possible!");
  }
}

// update to avoid hysteresis =============================================================================================================================================

void BallchaserProvider::updateVariables()
{
  updateBallPosition(nullptr);

  if (ballOnLeftSide && ballPosition.y() < -250.f)
    ballOnLeftSide = false;
  else if (!ballOnLeftSide && ballPosition.y() > 250.f)
    ballOnLeftSide = true;

  updateKickWithLeft();
}

void BallchaserProvider::updateKickWithLeft()
{
  const Pose2f leftFoot = Pose2f(theRobotPoseAfterPreview).translate(0.f, theWalkingEngineParams.footMovement.footYDistance);
  const Pose2f rightFoot = Pose2f(theRobotPoseAfterPreview).translate(0.f, -theWalkingEngineParams.footMovement.footYDistance);

  const float leftDistance = (ballPosition - leftFoot.translation).norm();
  const float rightDistance = (ballPosition - rightFoot.translation).norm();

  if (kickWithLeft && rightDistance < leftDistance - footDecisionHysteresis)
    kickWithLeft = false;
  else if (!kickWithLeft && leftDistance < rightDistance - footDecisionHysteresis)
    kickWithLeft = true;
}

void BallchaserProvider::updateBallPosition(const Vector2f* useBallPosition)
{
  if (useBallPosition == nullptr)
  {
    ballPosition = BallUtils::getBallPosition(usePredictedBallPosition, theBallSymbols, theFrameInfo);
  }
  else
  {
    ballPosition = *useBallPosition;
  }
}

void BallchaserProvider::updateDanger(Ballchaser& ballchaser)
{
  if (DangerUtils::isDanger(theRobotPoseAfterPreview.translation, ballPosition, 1.f, danger == Danger::HIGH, theDangerMap, theFieldDimensions, theRobotMap))
  {
    ballchaser.log_danger = "High";
    danger = Danger::HIGH;
    return;
  }

  if (DangerUtils::isDanger(theRobotPoseAfterPreview.translation, ballPosition, 1.5f, danger == Danger::MEDIUM, theDangerMap, theFieldDimensions, theRobotMap))
  {
    ballchaser.log_danger = "Medium";
    danger = Danger::MEDIUM;
    return;
  }

  if (DangerUtils::isDanger(theRobotPoseAfterPreview.translation, ballPosition, 2.0f, danger == Danger::LOW, theDangerMap, theFieldDimensions, theRobotMap))
  {
    ballchaser.log_danger = "Low";
    danger = Danger::LOW;
    return;
  }

  ballchaser.log_danger = "None";
  danger = Danger::NONE;
}

MAKE_MODULE(BallchaserProvider, behaviorControl)
