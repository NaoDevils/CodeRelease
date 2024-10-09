#include "BallchaserProvider.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Functions/SelectFunctions.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/ShotParameters.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Filterer/Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Ranges/DistanceRequirement.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Ranges/KickLine.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/ThresholdUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickBallTestObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickInPositionTestObjective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/Objectives/Test/KickToCenterTestObjective.h"

BallchaserProvider::BallchaserProvider() : regularPlayObjectivesManager(&logger)
{
  init();
}

void BallchaserProvider::init()
{
  regularPlayObjectivesManager.clear();
  regularPlayObjectivesManager.add(std::make_unique<ExecuteRecommendationObjective>(this, logger));

  setPlayCurrentKickManager = {};

  auto kickEngineParameters = KicksProvider::loadKickEngineParameters();
  auto customStepFiles = KicksProvider::loadCustomStepFiles();
  penaltyKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, penaltyKickNames);
}

void BallchaserProvider::update(Ballchaser& ballchaser)
{
  ballchaser = localBallchaser;
}

void BallchaserProvider::update(BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  ballchaserHeadPoiList = localBallchaserHeadPoiList;
}

void BallchaserProvider::execute(tf::Subflow& subflow)
{
  DECLARE_DEBUG_DRAWING(DRAW_KICK_RANGE_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_DANGER_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY, "drawingOnField");
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
  localBallchaser.kickType = MotionRequest::KickType::walkKick;
  localBallchaser.walkKickType = WalkRequest::StepRequest::none;
  // Reset Logs
  localBallchaser.log_currState = "None";
  localBallchaser.log_currObj = "None";
  localBallchaser.log_danger = "None";
  localBallchaser.log_kickName = "None";
  logger.clear();

  // Set Body
  localBallchaser.stopAtTarget = theGameInfo.state != STATE_PLAYING;
  localBallchaser.previewArrival = previewArrival;
  if (testBehavior)
  {
    regularPlay(localBallchaser);
  }
  else
  {
    decide(localBallchaser, theBallSymbols, theFieldDimensions, theGameInfo, theGameSymbols);
  }

  // Set head
  fillHeadPoiList(localBallchaserHeadPoiList, theRecommendedKick.hasRecommendation ? std::optional<Vector2f>{theRecommendedKick.kickTarget} : std::optional<Vector2f>{});

  // Set Logs
  localBallchaser.log_toBallDistance = std::to_string(Geometry::distance(theRobotPoseAfterPreview.translation, theBallSymbols.ballPositionFieldPredicted));
  localBallchaser.log_obj1 = logger.get(0);
  localBallchaser.log_obj2 = logger.get(1);
  localBallchaser.log_obj3 = logger.get(2);
  localBallchaser.log_obj4 = logger.get(3);
  localBallchaser.log_obj5 = logger.get(4);
}

void BallchaserProvider::fillHeadPoiList(BallchaserHeadPOIList& ballchaserHeadPoiList, const std::optional<Vector2f>& targetOptional)
{
  ballchaserHeadPoiList.targets.clear();

  if (theGameInfo.state == STATE_SET && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK)
  {
    ballchaserHeadPoiList.type = HeadPOIList::sweep;
    ballchaserHeadPoiList.addAngle({-90_deg, 15_deg});
    ballchaserHeadPoiList.addAngle({+90_deg, 15_deg});
    ballchaserHeadPoiList.addBall();
    return;
  }

  return;
  /*

  if (!targetOptional.has_value())
  {
    return;
  }

  const float SHOOT_NOW_TIME = 10.f;
  const float SHOOT_SOON_TIME = 15.f;

  const float playerTime = theRecommendedKick.estimatedKickTime + theRecommendedKick.estimatedPoseTime;
  const float opponentTime = theTacticSymbols.untilOpponentStealsBallTime;

  ballchaserHeadPoiList.playerShootsNow = playerTime < SHOOT_NOW_TIME;
  ballchaserHeadPoiList.opponentShootsNow = opponentTime < SHOOT_NOW_TIME;

  ballchaserHeadPoiList.playerShootsSoon = playerTime < SHOOT_SOON_TIME;
  ballchaserHeadPoiList.opponentShootsSoon = opponentTime < SHOOT_SOON_TIME;

  if (ballchaserHeadPoiList.playerShootsNow || ballchaserHeadPoiList.opponentShootsNow)
  {
    ballchaserHeadPoiList.type = HeadPOIList::focus;
    ballchaserHeadPoiList.addBall();
    return;
  }

  if (!ballchaserHeadPoiList.playerShootsSoon && !ballchaserHeadPoiList.opponentShootsSoon)
  {
    ballchaserHeadPoiList.type = HeadPOIList::sweep;
    // todo start with side on which the target is or with opponent side
    ballchaserHeadPoiList.addAngle({-90_deg, 0_deg});
    ballchaserHeadPoiList.addAngle({+90_deg, 0_deg});
    return;
  }

  ballchaserHeadPoiList.type = HeadPOIList::focus;
  ballchaserHeadPoiList.addBall();
  if (ballchaserHeadPoiList.playerShootsSoon)
  {
    ballchaserHeadPoiList.addFieldPosition(targetOptional.value());
  }
  if (ballchaserHeadPoiList.opponentShootsSoon)
  {
    ballchaserHeadPoiList.addFieldPosition(theTacticSymbols.closestToBallOpponentRobot.translation);
  }
  */
}

void BallchaserProvider::stateReady_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  positioningSymbols.recommendShot = false;
  PositionUtils::setPosition(positioningSymbols, -500.f, 0);
  PositionUtils::turnToPosition(positioningSymbols, kickOffPosition);
  ThresholdUtils::setThreshholdsLow(positioningSymbols);
}

void BallchaserProvider::statePlaying_kickOff_own(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  regularPlay(positioningSymbols);
}

void BallchaserProvider::stateReady_kickOff_opponent(Ballchaser& positioningSymbols, const Vector2f& kickOffPosition)
{
  positioningSymbols.recommendShot = false;
  positioningSymbols.optPosition.translation.x() = -theFieldDimensions.centerCircleRadius - theBehaviorConfiguration.behaviorParameters.kickOffLineDistance;
  positioningSymbols.optPosition.translation.y() = 0;
  positioningSymbols.optPosition.rotation = 0;
  positioningSymbols.thresholdXFront = 50;
  positioningSymbols.thresholdXBack = 50;
  positioningSymbols.thresholdY = 25;
}

float BallchaserProvider::goalKick_own(Ballchaser& positioningSymbols, bool left)
{
  const Vector2f waitPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea - 300.f, left ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
  actInOwnSetPlay(positioningSymbols, waitPosition);
  return 0.f;
}

float BallchaserProvider::goalKick_opponent(Ballchaser& positioningSymbols, bool left)
{
  defendOwnGoal(positioningSymbols, left);
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_own(Ballchaser& positioningSymbols)
{
  const Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;
  const Vector2f opponentGoalCenter = FieldUtils::getOpponentGoalCenter(theFieldDimensions);

  const Vector2f ballToOpponentGoalCenter = (opponentGoalCenter - ballPosition).normalized();
  const Vector2f waitPosition = ballPosition - 300 * ballToOpponentGoalCenter;

  actInOwnSetPlay(positioningSymbols, waitPosition);
  return 0.f;
}

float BallchaserProvider::pushingFreeKick_opponent(Ballchaser& positioningSymbols)
{
  defendOwnGoal(positioningSymbols, theBallSymbols.ballPositionFieldPredicted.y() > 0.f);
  return 0.f;
}

float BallchaserProvider::cornerKick_own(Ballchaser& positioningSymbols, const Vector2f& cornerKickPosition, bool left)
{
  const Vector2f waitPosition = Vector2f(theFieldDimensions.xPosOpponentGroundline + 200.f, left ? theFieldDimensions.yPosLeftSideline + 200.f : theFieldDimensions.yPosRightSideline - 200.f);
  actInOwnSetPlay(positioningSymbols, waitPosition);
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
  const Vector2f waitPosition = Vector2f(theBallSymbols.ballPositionField.x(), left ? theFieldDimensions.yPosLeftSideline + 300.f : theFieldDimensions.yPosRightSideline - 300.f);
  actInOwnSetPlay(positioningSymbols, waitPosition);
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
  ballchaser.recommendShot = false;

  setPlayCurrentKickManager.deleteCurrentKick();

  const Vector2f penaltyMarkPosition = {theFieldDimensions.xPosOpponentPenaltyMark, 0.f}; // TODo put Vector into theFieldDimensions
  const Vector2f position = {penaltyMarkPosition.x() - 500.f, penaltyMarkPosition.y()};
  ThresholdUtils::setThreshholdsMedium(ballchaser);
  PositionUtils::setPosition(ballchaser, position);
  PositionUtils::turnToPosition(ballchaser, position);
  return 0.f;
}

float BallchaserProvider::statePlaying_penaltyKick_own(Ballchaser& ballchaser)
{
  ballchaser.recommendShot = false;

  const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};

  const KickLine kickLine = {theBallSymbols.ballPositionFieldPredicted, leftGoalPostPosition, rightGoalPostPosition, DistanceRequirement::mayShorterOrFurther, 0.f}; // only use kicks that should go far enough
  Filterer filterer = Filterer().filterOutsideKickRange(kickLine).filterBlocked();
  const ShotParameters parameters = {ShotParameters::KickParameters(0.f, 0.f),
      ShotParameters::TargetParameters(0.f, 0.f, 0.f, 0.f, 0.f, 0.1f, 10.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f),
      ShotParameters::PoseParameters(0.f, 0.f, 0.f)};
  const auto currentKick = setPlayCurrentKickManager.getCurrentKick(theBallSymbols.ballPositionFieldPredicted);

  const auto executableShotOptional = SelectFunctions::createAndFilterAndSelect(
      theRobotPoseAfterPreview, theBallSymbols.ballPositionFieldPredicted, KickUtils::unpack(penaltyKicks), currentKick, filterer, parameters, theDirectionInfo, theFieldDimensions, thePositionInfo, theTacticSymbols);

  if (executableShotOptional.has_value())
  {
    setPlayCurrentKickManager.setCurrentKick(ballchaser, executableShotOptional.value(), theFrameInfo);
  }
  else
  {
    // TODO Warning only if Ballchaser
  }
  return 0.f;
}

float BallchaserProvider::stateReady_penaltyKick_opponent(Ballchaser& positioningSymbols)
{
  positioningSymbols.recommendShot = false;
  positioningSymbols.optPosition.translation.x() = theFieldDimensions.xPosOwnPenaltyArea + 300.f;
  positioningSymbols.optPosition.translation.y() = theFieldDimensions.yPosKickOffPoint + 800.f;
  const Vector2f penaltyCross{theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosKickOffPoint};
  positioningSymbols.optPosition.rotation = (penaltyCross - positioningSymbols.optPosition.translation).angle();
  return 0.f;
}

void BallchaserProvider::regularPlay(Ballchaser& positioningSymbols)
{
  positioningSymbols.recommendShot = true;
  regularPlayObjectivesManager.performObjective(positioningSymbols);
}

// Helper ==========================================================================================================================================================================

bool BallchaserProvider::waitInOwnSetPlay(Ballchaser& positioningSymbols, const Vector2f& waitingPosition)
{
  // TODO HeadControl: The Robot only looks around once

  const int SET_PLAY_DURATION = 30; // TODO Variable
  const int MIN_REMAINING_TIME = 7;
  const int MAX_LOOSING_OR_EVEN_WAIT_TIME = 0; // Should be enough time to look around at least once
  const int MAX_WINNING_WAIT_TIME = 12;
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
  if (theOwnTeamInfo.score <= theOpponentTeamInfo.score && timePassedSinceStartWaiting > MAX_LOOSING_OR_EVEN_WAIT_TIME)
  {
    return false;
  }
  if (theOwnTeamInfo.score > theOpponentTeamInfo.score && timePassedSinceStartWaiting > MAX_WINNING_WAIT_TIME)
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

void BallchaserProvider::actInOwnSetPlay(Ballchaser& ballchaser, const Vector2f& waitPosition)
{
  if (waitInOwnSetPlay(ballchaser, waitPosition))
  {
    ballchaser.recommendShot = false;
    return;
  }
  regularPlay(ballchaser);
}

void BallchaserProvider::actInOpponentsSetPlay(Ballchaser& ballchaser, const bool left, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose)
{
  ballchaser.recommendShot = false;

  const Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;

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
  ballchaser.recommendShot = false;

  const Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;

  const Vector2f targetToCover{theFieldDimensions.xPosOwnGroundline,
      left ? (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosRightGoal) / 2.f : (theFieldDimensions.yPosCenterGoal + theFieldDimensions.yPosLeftGoal) / 2.f};
  const Vector2f ballToOwnGoal{targetToCover - ballPosition};
  Vector2f position{ballPosition + ballToOwnGoal.normalized(oppSetPlayBallDistance)};
  position.x() = std::max(position.x(), theFieldDimensions.xPosOwnGroundline + 300.f);

  ballchaser.optPosition.translation = position;
  ballchaser.optPosition.rotation = (ballPosition - position).angle();
}

MAKE_MODULE(BallchaserProvider, behaviorControl)
