#include "BallchaserProvider.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickInPositionTestObjective.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/KickLine.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h>

BallchaserProvider::BallchaserProvider() : regularPlayObjectivesManager(&logger)
{
  init();
}

void BallchaserProvider::init()
{
  regularPlayObjectivesManager.clear();
  regularPlayObjectivesManager.add(std::make_unique<GoalObjective>(this, logger));
  // Inner lists can always be left so the GoalObjective will always be tried, even if the OneVsOneObjective forbids to leave
  std::unique_ptr<ObjectivesList<BallchaserProvider, Ballchaser>> objectiveListPointer = std::make_unique<ObjectivesList<BallchaserProvider, Ballchaser>>(&logger);
  //objectiveListPointer->add(std::make_unique<TickKickObjective>(this, logger));
  objectiveListPointer->add(std::make_unique<MoveObjective>(this, logger));
  regularPlayObjectivesManager.add(std::move(objectiveListPointer));

  setPlayKickManager = {};

  auto kickEngineParameters = KicksProvider::loadKickEngineParameters();
  auto customStepFiles = KicksProvider::loadCustomStepFiles();
  kickOffKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, kickOffKickNames);
  kickInKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, kickInKickNames);
  cornerKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, cornerKickNames);
  goalKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, goalKickNames);
  penaltyKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, penaltyKickNames);
  pushingFreeKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, pushingFreeKickNames);
}

void BallchaserProvider::update(Ballchaser& ballchaser)
{
  DECLARE_DEBUG_DRAWING(DRAW_KICK_RANGE_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_DANGER_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICK_TARGET_AREA, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_MIN_WIDTH, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_TEST, "drawingOnField");

  DEBUG_RESPONSE_ONCE("module:BallchaserProvider:default") // Command: "dr module:BallchaserProvider:default"
  {
    testBehavior = false;
    init();
  }
  std::string kickToCenterTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickToCenterTest", kickToCenterTestKickName); // Command: "set module:BallchaserProvider:kickToCenterTest kickHack"
  if (!kickToCenterTestKickName.empty())
  {
    testBehavior = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickToCenterTestObjective>(this, logger, kickToCenterTestKickName));
  }
  std::string kickBallTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickBallTest", kickBallTestKickName); // Example command: "set module:BallchaserProvider:kickBallTest kickHack"
  if (!kickBallTestKickName.empty())
  {
    testBehavior = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickBallTestObjective>(this, logger, kickBallTestKickName));
  }
  std::string kickInPositionTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickInPositionTest", kickInPositionTestKickName); // Example command: "set module:BallchaserProvider:kickInPositionTest kickHack"
  if (!kickInPositionTestKickName.empty())
  {
    testBehavior = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickInPositionTestObjective>(this, logger, kickInPositionTestKickName));
  }

  // Reset
  ballchaser.kickType = MotionRequest::KickType::walkKick;
  ballchaser.walkKickType = WalkRequest::StepRequest::none;
  // Reset Logs
  ballchaser.log_currState = "None";
  ballchaser.log_currObj = "None";
  ballchaser.log_danger = "None";
  ballchaser.log_kickName = "None";
  logger.clear();

  // Update
  updateVariables(ballchaser);

  // Set
  ballchaser.stopAtTarget = theGameInfo.state != STATE_PLAYING;
  ballchaser.previewArrival = previewArrival;
  if (testBehavior)
  {
    regularPlay(ballchaser);
  }
  else
  {
    decide(ballchaser, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
  }
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
  setPlayKickManager.stop();

  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  PositionUtils::setPosition(positioningSymbols, -500.f, 0);
  PositionUtils::turnToPosition(positioningSymbols, kickOffPosition);
  ThresholdUtils::setThreshholdsLow(positioningSymbols);
}

void BallchaserProvider::statePlaying_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  const Filterer filterer = Filterer().filterTooFarBack(0.f).filterOutside().filterBlocked();
  const Factors factors = {0.3f,
      0.f,
      1.f, // Field
      0.01f,
      0.3f, // Team
      -0.01f,
      -0.3f, // Opponent
      0.f,
      0.f,
      0.f, // Pose
      0.f,
      0.f}; // Kick
  const std::optional<CurrentKick> currentKick = setPlayKickManager.getCurrentKick(ballPosition);

  auto selectedKickOptional = SelectFunctions::createAndFilterAndSelect(
      theRobotPoseAfterPreview, ballPosition, KickUtils::unpack(kickOffKicks), currentKick, filterer, factors, theFieldDimensions, theHeatMapCollection, theKickWheel, theRobotMap, theTacticSymbols);

  if (selectedKickOptional.has_value())
  {
    setPlayKickManager.kickTo(positioningSymbols, selectedKickOptional.value(), theFrameInfo);
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
  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  const Vector2f waitPos = Vector2f(theFieldDimensions.xPosOwnGoalArea - 300.f, left ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
  const float minX = theFieldDimensions.xPosOwnGoalArea + 300.f;
  actInOwnSetPlay(positioningSymbols, waitPos, minX, 0.5f, KickUtils::unpack(goalKicks));
  return 0.f;
}

float BallchaserProvider::goalKick_opponent(Ballchaser& positioningSymbols, bool left)
{
  defendOwnGoal(positioningSymbols, left);
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_own(Ballchaser& positioningSymbols)
{
  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  const Vector2f opponentGoalCenter = FieldUtils::getOpponentGoalCenter(theFieldDimensions);
  const Vector2f ballToOpponentGoalCenter = (opponentGoalCenter - ballPosition).normalized();
  const Vector2f waitPosition = ballPosition - 300 * ballToOpponentGoalCenter;
  const float minX = MathUtils::clamp_f(ballPosition.x(), theFieldDimensions.xPosOwnPenaltyArea + 200.f, theFieldDimensions.xPosOpponentGoalArea - 200.f);
  actInOwnSetPlay(positioningSymbols, waitPosition, minX, 0.3f, KickUtils::unpack(kickInKicks));
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_opponent(Ballchaser& positioningSymbols)
{
  defendOwnGoal(positioningSymbols, ballOnLeftSide);
  return 0.f;
}

float BallchaserProvider::cornerKick_own(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  Vector2f waitPos = Vector2f(theFieldDimensions.xPosOpponentGroundline + 200.f, left ? theFieldDimensions.yPosLeftSideline + 200.f : theFieldDimensions.yPosRightSideline - 200.f);
  const float minX = theFieldDimensions.xPosOpponentPenaltyArea - 1000.f;
  actInOwnSetPlay(positioningSymbols, waitPos, minX, 0.2f, KickUtils::unpack(cornerKicks));
  return 0.f;
}

float BallchaserProvider::cornerKick_opponent(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  actInOpponentsSetPlay(positioningSymbols,
      left,
      [this](const Vector2f& robotPosition)
      {
        return (abs(robotPosition.y()) >= abs(theFieldDimensions.yPosLeftSideline) + 60.f && robotPosition.x() < theFieldDimensions.xPosOwnGroundline + 20.f);
      });
  return 0.f;
}

float BallchaserProvider::kickIn_own(Ballchaser& positioningSymbols, bool left) //TODO: Wait for 10 seconds if time permits before shooting. Longer if we are winning. RoleProvider. 1vs1.
{
  danger = Danger::IMPOSSIBLE;
  positioningSymbols.log_danger = "None";

  Vector2f waitPos = Vector2f(theBallSymbols.ballPositionField.x(), left ? theFieldDimensions.yPosLeftSideline + 300.f : theFieldDimensions.yPosRightSideline - 300.f);
  const float minX = MathUtils::clamp_f(ballPosition.x() - 500.f, theFieldDimensions.xPosOwnPenaltyArea + 200.f, theFieldDimensions.xPosOpponentGoalArea - 200.f);
  actInOwnSetPlay(positioningSymbols, waitPos, minX, 0.3f, KickUtils::unpack(kickInKicks));
  return 0.f;
}

float BallchaserProvider::kickIn_opponent(Ballchaser& positioningSymbols, bool left)
{
  actInOpponentsSetPlay(positioningSymbols,
      left,
      [this](const Vector2f& robotPosition)
      {
        return abs(robotPosition.y()) >= abs(theFieldDimensions.yPosLeftSideline) - 20.f;
      });
  return 0.f;
}

float BallchaserProvider::stateReady_penaltyKick_own(Ballchaser& ballchaser)
{
  setPlayKickManager.stop();

  danger = Danger::IMPOSSIBLE;
  ballchaser.log_danger = "None";

  const Vector2f penaltyMarkPosition = {theFieldDimensions.xPosOpponentPenaltyMark, 0.f}; // TODo put Vector into theFieldDimensions
  const Vector2f position = {penaltyMarkPosition.x() - 500.f, penaltyMarkPosition.y()};
  ThresholdUtils::setThreshholdsMedium(ballchaser);
  PositionUtils::setPosition(ballchaser, position);
  PositionUtils::turnToPosition(ballchaser, position);
  return 0.f;
}

float BallchaserProvider::statePlaying_penaltyKick_own(Ballchaser& ballchaser)
{
  danger = Danger::IMPOSSIBLE;
  ballchaser.log_danger = "None";

  const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};

  const KickLine kickLine = {ballPosition, leftGoalPostPosition, rightGoalPostPosition, DistanceRequirement::mayShorterOrFurther, 0.f}; // only use kicks that should go far enough
  Filterer filterer = Filterer().filterOutsideKickRange(kickLine).filterBlocked();
  const Factors factors = {1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  const auto currentKick = setPlayKickManager.getCurrentKick(ballPosition);

  const auto selectableKick = SelectFunctions::createAndFilterAndSelect(
      theRobotPoseAfterPreview, ballPosition, KickUtils::unpack(penaltyKicks), currentKick, filterer, factors, theFieldDimensions, theHeatMapCollection, theKickWheel, theRobotMap, theTacticSymbols);

  if (selectableKick.has_value())
  {
    setPlayKickManager.kickTo(ballchaser, selectableKick.value(), theFrameInfo);
  }
  else
  {
    // TODO Warning only if Ballchaser
  }
  return 0.f;
}

float BallchaserProvider::stateReady_penaltyKick_opponent(Ballchaser& positioningSymbols)
{
  positioningSymbols.optPosition.translation.x() = theFieldDimensions.xPosOwnPenaltyArea + 300.f;
  positioningSymbols.optPosition.translation.y() = theFieldDimensions.yPosKickOffPoint + 800.f;
  const Vector2f penaltyCross{theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosKickOffPoint};
  positioningSymbols.optPosition.rotation = (penaltyCross - positioningSymbols.optPosition.translation).angle();
  return 0.f;
}

void BallchaserProvider::regularPlay(Ballchaser& positioningSymbols)
{
  regularPlayObjectivesManager.performObjective(positioningSymbols);
}

// Helper ==========================================================================================================================================================================

bool BallchaserProvider::waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition)
{
  // TODO HeadControl: The Robot only looks around once

  const int SET_PLAY_DURATION = 30; // TODO Variable
  const int MIN_REMAINING_TIME = 7;
  const int MAX_LOOSING_WAIT_TIME = 10; // Should be enough time to look around at least once
  const int MAX_EVEN_WAIT_TIME = 12;
  const float WAIT_POSE_DISTANCE_THRESHOLD = 100.f;

  const int remainingTime = SET_PLAY_DURATION - theGameSymbols.timeSinceSetPlayStarted;

  if (remainingTime >= SET_PLAY_DURATION - 1) // Used to reset when new setPlay
  {
    startedSetPlayWaitingRemainingTime = 0;
  }

  if (remainingTime < MIN_REMAINING_TIME)
  {
    return false;
  }

  const int timePassedSinceStartWaiting = startedSetPlayWaitingRemainingTime - remainingTime;
  if (theOwnTeamInfo.score < theOpponentTeamInfo.score && timePassedSinceStartWaiting > MAX_LOOSING_WAIT_TIME)
  {
    return false;
  }
  if (theOwnTeamInfo.score == theOpponentTeamInfo.score && timePassedSinceStartWaiting > MAX_EVEN_WAIT_TIME)
  {
    return false;
  }

  if (startedSetPlayWaitingRemainingTime == 0 && Geometry::distance(theRobotPose.translation, waitingPosition) < WAIT_POSE_DISTANCE_THRESHOLD)
  {
    startedSetPlayWaitingRemainingTime = remainingTime;
  }

  PositionUtils::setPosition(positioningSymbols, waitingPosition);
  PositionUtils::turnTowardsBall(positioningSymbols, theBallSymbols);
  positioningSymbols.stopAtTarget = true;
  return true;
}

void BallchaserProvider::actInOwnSetPlay(Ballchaser& ballchaser, const Vector2f& waitPosition, const float minX, const float goalsHeatFactor, const std::vector<Kick*>& kicks)
{
  if (waitInOwnSetPlay(ballchaser, waitPosition))
  {
    return;
  }
  Filterer filterer = Filterer().filterTooFarBack(minX).filterOutside().filterBlocked();
  const Factors factors = {0.5f, 0.f, goalsHeatFactor, 0.1f, 1.f, -0.5f, -0.5f, 0.f, 0.f, 0.f, 0.f, 0.f};
  const auto currentKick = setPlayKickManager.getCurrentKick(ballPosition);
  const auto selectableKick = SelectFunctions::createAndFilterAndSelect(
      theRobotPoseAfterPreview, ballPosition, kicks, currentKick, filterer, factors, theFieldDimensions, theHeatMapCollection, theKickWheel, theRobotMap, theTacticSymbols);
  if (selectableKick.has_value())
  {
    setPlayKickManager.kickTo(ballchaser, selectableKick.value(), theFrameInfo);
  }
  else
  {
    // TODO Nur ausgeben wenn ballchaser
    // OUTPUT_WARNING("Free kick not possible!");
  }
}

void BallchaserProvider::actInOpponentsSetPlay(Ballchaser& ballchaser, const bool left, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose)
{
  Vector2f kickerPose;
  int numberOfRobotsDetected = 0;
  for (const RobotMapEntry& robot : theRobotMap.robots)
  {
    const Vector2f robotPosition = robot.pose.translation;
    if ((robotPosition - ballPosition).norm() < 400.f && isRobotPoseKickerPose(robotPosition))
    {
      numberOfRobotsDetected++;
      if (numberOfRobotsDetected > 1)
      {
        break;
      }
      kickerPose = robotPosition;
    }
  }
  if (numberOfRobotsDetected == 1)
  {
    const Vector2f kickerToBall = ballPosition - kickerPose;
    float distanceFromKickerToBall = (kickerToBall).norm();
    const Vector2f targetPos = ballPosition + kickerToBall * oppSetPlayBallDistance / distanceFromKickerToBall;
    PositionUtils::setPosition(ballchaser, targetPos);
    PositionUtils::turnTowardsBall(ballchaser, theBallSymbols);
  }
  else
  {
    defendOwnGoal(ballchaser, left);
  }
}

void BallchaserProvider::defendOwnGoal(Ballchaser& ballchaser, const bool left)
{
  const Vector2f targetToCover{theFieldDimensions.xPosOwnGroundline,
      left ? (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosRightGoal) / 2.f : (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosLeftGoal) / 2.f};
  const Vector2f ballToOwnGoal{targetToCover - ballPosition};
  Vector2f position{ballPosition + ballToOwnGoal.normalized(oppSetPlayBallDistance)};
  position.x() = std::max(position.x(), theFieldDimensions.xPosOwnGroundline + 300.f);

  ballchaser.optPosition.translation = position;
  ballchaser.optPosition.rotation = (ballPosition - position).angle();
}

// Update to avoid hysteresis ======================================================================================================================================================

void BallchaserProvider::updateVariables(Ballchaser& ballchaser)
{
  // Update ballPosition
  ballPosition = BallUtils::getBallPosition(theBallSymbols, theFrameInfo, theRobotPoseAfterPreview);

  // Update ballOnLeftSide
  if (ballOnLeftSide && ballPosition.y() < -250.f)
    ballOnLeftSide = false;
  else if (!ballOnLeftSide && ballPosition.y() > 250.f)
    ballOnLeftSide = true;

  // Update danger
  if (isDanger(450.f, danger == Danger::HIGH))
  {
    ballchaser.log_danger = "High";
    danger = Danger::HIGH;
  }
  else if (isDanger(800.f, danger == Danger::MEDIUM))
  {
    ballchaser.log_danger = "Medium";
    danger = Danger::MEDIUM;
  }
  else if (isDanger(1200.f, danger == Danger::LOW))
  {
    ballchaser.log_danger = "Low";
    danger = Danger::LOW;
  }
  else
  {
    ballchaser.log_danger = "None";
    danger = Danger::NONE;
  }
}

bool BallchaserProvider::isDanger(float dangerDistance, const bool hysteresis) const
{
  dangerDistance = dangerDistance * (hysteresis ? 1.2f : 1.f);
  for (const auto& robot : theRobotMap.robots)
  {
    if (robot.robotType != RobotEstimate::teammateRobot)
    {
      const float robotDistance = Geometry::distance(ballPosition, robot.pose.translation);
      if (robotDistance < dangerDistance)
      {
        return true;
      }
    }
  }
  return false;
}

MAKE_MODULE(BallchaserProvider, behaviorControl)
