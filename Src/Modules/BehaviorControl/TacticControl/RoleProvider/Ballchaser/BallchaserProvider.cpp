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
  regularPlayObjectivesManager.add(std::make_unique<RecommendedKickObjective>(this, logger));

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
  declareDebugDrawings();

  // Reset
  localBallchaser.kickType = MotionRequest::KickType::walkKick;
  localBallchaser.walkKickType = WalkRequest::StepRequest::none;
  localBallchaserHeadPoiList.targets.clear();
  // Reset Logs
  localBallchaser.log_currState = "None";
  localBallchaser.log_currObj = "None";
  localBallchaser.log_danger = "None";
  localBallchaser.log_kickName = "None";
  logger.clear();

  // Set behavior
  readTestBehaviorCommandsAndUpdate();
  localBallchaser.stopAtTarget = theGameInfo.state != STATE_PLAYING;
  localBallchaser.previewArrival = previewArrival;
  if (testMode)
  {
    regularPlay(localBallchaser, localBallchaserHeadPoiList);
  }
  else
  {
    switch (theGameSymbols.gameSituation)
    {
    case GameSymbols::GameSituation::kickOff_own_ready:
      kickOff_own(localBallchaser, localBallchaserHeadPoiList, true);
      break;
    case GameSymbols::GameSituation::kickOff_own_set:
      kickOff_own(localBallchaser, localBallchaserHeadPoiList, false);
      break;
    case GameSymbols::GameSituation::kickOff_opponent_ready:
      kickOff_opponent(localBallchaser, localBallchaserHeadPoiList, true);
      break;
    case GameSymbols::GameSituation::kickOff_opponent_set:
    case GameSymbols::GameSituation::kickOff_opponent_playing_ballNotFree:
      kickOff_opponent(localBallchaser, localBallchaserHeadPoiList, false);
      break;
    case GameSymbols::GameSituation::goalKick_own:
      goalKick_own(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::pushingFreeKick_own:
      pushingFreeKick_own(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::goalKick_opponent:
    case GameSymbols::GameSituation::pushingFreeKick_opponent:
      defendOwnGoal(localBallchaser, localBallchaserHeadPoiList, false);
      break;
    case GameSymbols::GameSituation::cornerKick_own:
      cornerKick_own(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::cornerKick_opponent:
      cornerKick_opponent(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::kickIn_own:
      kickIn_own(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::kickIn_opponent:
      kickIn_opponent(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::penaltyKick_own_ready:
    case GameSymbols::GameSituation::penaltyKick_own_set:
      penaltyKick_own_ready(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::penaltyKick_own_playing:
      penaltyKick_own_playing(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::penaltyKick_opponent_ready:
    case GameSymbols::GameSituation::penaltyKick_opponent_set:
    case GameSymbols::GameSituation::penaltyKick_opponent_playing:
      penaltyKick_opponent_ready(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::kickOff_own_playing_ballNotFree:
    case GameSymbols::GameSituation::regularPlay:
      regularPlay(localBallchaser, localBallchaserHeadPoiList);
      break;
    case GameSymbols::GameSituation::none:
      OUTPUT_ERROR("GameSituation must not be handled in the BallchaserProvider!");
      break;
    default:
      OUTPUT_ERROR("Unknown GameSituation in BallchaserProvider!");
      break;
    }
  }

  // Set Logs
  localBallchaser.log_toBallDistance = std::to_string(Geometry::distance(theRobotPoseAfterPreview.translation, theBallSymbols.ballPositionFieldPredicted));
  localBallchaser.log_obj1 = logger.get(0);
  localBallchaser.log_obj2 = logger.get(1);
  localBallchaser.log_obj3 = logger.get(2);
  localBallchaser.log_obj4 = logger.get(3);
  localBallchaser.log_obj5 = logger.get(4);
}

void BallchaserProvider::declareDebugDrawings()
{
  DECLARE_DEBUG_DRAWING(DRAW_KICK_RANGE_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_DANGER_NAME, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_IN_GRID, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_EXECUTABLE_KICKS_FREELY, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_KICK_MIN_WIDTH, "drawingOnField");
  DECLARE_DEBUG_DRAWING(DRAW_TEST, "drawingOnField");
}

void BallchaserProvider::readTestBehaviorCommandsAndUpdate()
{
  DEBUG_RESPONSE_ONCE("module:BallchaserProvider:default") // Command: "dr module:BallchaserProvider:default"
  {
    testMode = false;
    init();
  }
  std::string kickToCenterTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickToCenterTest", kickToCenterTestKickName); // Command: "set module:BallchaserProvider:kickToCenterTest kickHack"
  if (!kickToCenterTestKickName.empty())
  {
    testMode = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickToCenterTestObjective>(this, logger, kickToCenterTestKickName));
  }
  std::string kickBallTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickBallTest", kickBallTestKickName); // Example command: "set module:BallchaserProvider:kickBallTest kickHack"
  if (!kickBallTestKickName.empty())
  {
    testMode = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickBallTestObjective>(this, logger, kickBallTestKickName));
  }
  std::string kickInPositionTestKickName;
  MODIFY_ONCE("module:BallchaserProvider:kickInPositionTest", kickInPositionTestKickName); // Example command: "set module:BallchaserProvider:kickInPositionTest kickHack"
  if (!kickInPositionTestKickName.empty())
  {
    testMode = true;
    regularPlayObjectivesManager.clear();
    regularPlayObjectivesManager.add(std::make_unique<KickInPositionTestObjective>(this, logger, kickInPositionTestKickName));
  }
}

void BallchaserProvider::kickOff_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const bool readyElseSet)
{
  ballchaser.recommendShot = false;

  PositionUtils::setPosition(ballchaser, -500.f, 0);
  PositionUtils::turnToPosition(ballchaser, Vector2f(0.f, 0.f));
  ThresholdUtils::setThreshholdsLow(ballchaser);

  if (readyElseSet)
  {
    // ready
  }
  else
  {
    // set
    ballchaserHeadPoiList.type = HeadPOIList::sweep;
    ballchaserHeadPoiList.addAngle({-90_deg, 15_deg});
    ballchaserHeadPoiList.addAngle({+90_deg, 15_deg});
  }
}

void BallchaserProvider::kickOff_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const bool readyElseSet)
{
  ballchaser.recommendShot = false;

  PositionUtils::setPosition(ballchaser, -theFieldDimensions.centerCircleRadius - theBehaviorConfiguration.behaviorParameters.kickOffLineDistance, 0.f);
  PositionUtils::turnToPosition(ballchaser, Vector2f(0.f, 0.f));
  ThresholdUtils::setThreshholdsLow(ballchaser);

  if (readyElseSet)
  {
    // ready
  }
  else
  {
    // set
    ballchaserHeadPoiList.type = HeadPOIList::focus;
    ballchaserHeadPoiList.addBall();
  }
}

void BallchaserProvider::goalKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  const bool left = theBallSymbols.ballPositionFieldPredicted.y() > 0;
  const Vector2f waitPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea - 300.f, left ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea);
  actInOwnSetPlay(ballchaser, ballchaserHeadPoiList, waitPosition);
}

void BallchaserProvider::pushingFreeKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  const Vector2f ballPosition = theBallSymbols.ballPositionFieldPredicted;
  const Vector2f opponentGoalCenter = FieldUtils::getOpponentGoalCenter(theFieldDimensions);

  const Vector2f ballToOpponentGoalCenter = (opponentGoalCenter - ballPosition).normalized();
  const Vector2f waitPosition = ballPosition - 300 * ballToOpponentGoalCenter;

  actInOwnSetPlay(ballchaser, ballchaserHeadPoiList, waitPosition);
}

void BallchaserProvider::cornerKick_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  const bool left = theBallSymbols.ballPositionFieldPredicted.y() > 0;
  const Vector2f waitPosition = Vector2f(theFieldDimensions.xPosOpponentGroundline + 200.f, left ? theFieldDimensions.yPosLeftSideline + 200.f : theFieldDimensions.yPosRightSideline - 200.f);
  actInOwnSetPlay(ballchaser, ballchaserHeadPoiList, waitPosition);
}

void BallchaserProvider::cornerKick_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  actInOpponentsSetPlay(ballchaser,
      ballchaserHeadPoiList,
      [this](const Vector2f& robotPosition)
      {
        return (abs(robotPosition.y()) >= abs(theFieldDimensions.yPosLeftSideline) + 60.f && robotPosition.x() < theFieldDimensions.xPosOwnGroundline + 20.f);
      });
}

void BallchaserProvider::kickIn_own(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList) //TODO: Wait for 10 seconds if time permits before shooting. Longer if we are winning. RoleProvider. 1vs1.
{
  const bool left = theBallSymbols.ballPositionFieldPredicted.y() > 0;
  const Vector2f waitPosition = Vector2f(theBallSymbols.ballPositionField.x(), left ? theFieldDimensions.yPosLeftSideline + 300.f : theFieldDimensions.yPosRightSideline - 300.f);
  actInOwnSetPlay(ballchaser, ballchaserHeadPoiList, waitPosition);
}

void BallchaserProvider::kickIn_opponent(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  actInOpponentsSetPlay(ballchaser,
      ballchaserHeadPoiList,
      [this](const Vector2f& robotPosition)
      {
        return abs(robotPosition.y()) >= abs(theFieldDimensions.yPosLeftSideline) - 20.f;
      });
}

void BallchaserProvider::penaltyKick_own_ready(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  ballchaser.recommendShot = false;

  setPlayCurrentKickManager.deleteCurrentKick();

  const Vector2f penaltyMarkPosition = {theFieldDimensions.xPosOpponentPenaltyMark, 0.f}; // TODo put Vector into theFieldDimensions
  const Vector2f position = {penaltyMarkPosition.x() - 500.f, penaltyMarkPosition.y()};
  ThresholdUtils::setThreshholdsMedium(ballchaser);
  PositionUtils::setPosition(ballchaser, position);
  PositionUtils::turnToPosition(ballchaser, position);
}

void BallchaserProvider::penaltyKick_own_playing(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
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
}

void BallchaserProvider::penaltyKick_opponent_ready(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  ballchaser.recommendShot = false;
  ballchaser.optPosition.translation.x() = theFieldDimensions.xPosOwnPenaltyArea + 300.f;
  ballchaser.optPosition.translation.y() = theFieldDimensions.yPosKickOffPoint + 800.f;
  const Vector2f penaltyCross{theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosKickOffPoint};
  ballchaser.optPosition.rotation = (penaltyCross - ballchaser.optPosition.translation).angle();
}

void BallchaserProvider::regularPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList)
{
  if (theGameSymbols.timeSincePlayingState < 25000)
  {
    Vector2f target = Vector2f(-2500.f, 1000.f);

    kickManager.kickTo(ballchaser, theRobotPose, theBallSymbols.ballPositionFieldPredicted, target, true, KickUtils::unpack(penaltyKicks), theFrameInfo);
  }
  else
  {
    ballchaser.recommendShot = true;
    regularPlayObjectivesManager.performObjective(ballchaser);
  }
}

// Helper ==========================================================================================================================================================================

bool BallchaserProvider::waitInOwnSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const Vector2f& waitingPosition)
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

  PositionUtils::setPosition(ballchaser, waitingPosition);
  PositionUtils::turnTowardsBall(ballchaser, theBallSymbols);
  ballchaser.stopAtTarget = true;
  return true;
}

void BallchaserProvider::actInOwnSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const Vector2f& waitPosition)
{
  if (waitInOwnSetPlay(ballchaser, ballchaserHeadPoiList, waitPosition))
  {
    ballchaser.recommendShot = false;
    return;
  }
  regularPlay(ballchaser, ballchaserHeadPoiList);
}

void BallchaserProvider::actInOpponentsSetPlay(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const std::function<bool(const Vector2f&)>& isRobotPoseKickerPose)
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
    defendOwnGoal(ballchaser, ballchaserHeadPoiList, false);
  }
}

void BallchaserProvider::defendOwnGoal(Ballchaser& ballchaser, BallchaserHeadPOIList& ballchaserHeadPoiList, const bool recommendShot)
{
  const bool left = theBallSymbols.ballPositionFieldPredicted.y() > 0;

  ballchaser.recommendShot = recommendShot;

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
