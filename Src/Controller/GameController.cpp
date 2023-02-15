/**
 * @file Controller/GameController.h
 * This file implements a class that simulates a console-based GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameController.h"
#include "SimulatedRobot.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Eigen.h"
#include "Platform/BHAssert.h"
#include <limits>
#include <algorithm>
#include <iostream>

FieldDimensions GameController::fieldDimensions;
Pose2f GameController::lastBallContactPose;
int GameController::timeOfLastBallContact;

// move to header when MSC supports constexpr
const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController()
    : timeWhenHalfStarted(0), timeOfLastDropIn(0), timeWhenLastRobotMoved(0), timeWhenStateBegan(0),

      automatic(true)
{
  gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
  gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
  gameInfo.gamePhase = GAME_PHASE_NORMAL;
  gameInfo.setPlay = SET_PLAY_NONE;
  gameInfo.playersPerTeam = numOfRobots / 2;
  gameInfo.firstHalf = 1;
  gameInfo.kickingTeam = 1;
  gameInfo.secsRemaining = durationOfHalf;
  teamInfos[TEAM_BLUE].teamNumber = 1;
  teamInfos[TEAM_BLUE].teamColour = TEAM_BLUE;
  teamInfos[TEAM_RED].teamNumber = 2;
  teamInfos[TEAM_RED].teamColour = TEAM_RED;
}

void GameController::registerSimulatedRobot(int robot, SimulatedRobot& simulatedRobot)
{
  ASSERT(!robots[robot].simulatedRobot);
  robots[robot].simulatedRobot = &simulatedRobot;
  robots[robot].info.number = robot % (numOfRobots / 2) + 1;
  if (fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if (command == "initial")
  {
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
      gameInfo.secsRemaining = durationOfHalf;
    else
      gameInfo.secsRemaining = durationOfPS;
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = timeWhenHalfStarted = 0;
    return true;
  }
  else if (command == "ready")
  {
    if (gameInfo.state == STATE_INITIAL)
      gameInfo.timeFirstReadyState = SystemCall::getCurrentSystemTime();
    gameInfo.state = STATE_READY;
    for (int i = 0; i < numOfRobots; ++i)
      if (robots[i].info.penalty)
        handleRobotCommand(i, "none");
    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "set")
  {
    // in case ready state is skipped
    if (gameInfo.state == STATE_INITIAL)
      gameInfo.timeFirstReadyState = SystemCall::getCurrentSystemTime();
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
    {
      if (gameInfo.setPlay == SET_PLAY_PENALTY_KICK)
      {
        handlePenaltyKickSet();
      }
      else
      {
        gameInfo.state = STATE_SET;
        for (int i = 0; i < numOfRobots; ++i)
          robots[i].info.penalty = none;

        if (automatic)
        {
          placeGoalie(0);
          placeGoalie(numOfRobots / 2);
          placeDefensivePlayers(gameInfo.kickingTeam == 1 ? numOfRobots / 2 + 1 : 1);
          placeOffensivePlayers(gameInfo.kickingTeam == 1 ? 1 : numOfRobots / 2 + 1);
          executePlacement();
        }

        timeWhenStateBegan = SystemCall::getCurrentSystemTime();

        /*if(Global::getSettings().isCornerChallenge)
          SimulatedRobot::moveBall(Vector3f(-4500.f, -3000.f, 50.f), true);
        else*/
        SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
      }
    }
    else
    {
      gameInfo.state = STATE_SET;
      for (int i = 0; i < numOfRobots; ++i)
        robots[i].info.penalty = none;

      gameInfo.secsRemaining = durationOfPS;
      timeWhenHalfStarted = 0;
      timeWhenStateBegan = SystemCall::getCurrentSystemTime();

      SimulatedRobot::moveBall(Vector3f(-3200.f, 0.f, 50.f), true);
    }
    return true;
  }
  else if (command == "playing")
  {
    // in case ready state is skipped
    if (gameInfo.state == STATE_INITIAL)
      gameInfo.timeFirstReadyState = SystemCall::getCurrentSystemTime();
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
    {
      gameInfo.state = STATE_PLAYING;
      if (gameInfo.setPlay != SET_PLAY_PENALTY_KICK)
        gameInfo.setPlay = SET_PLAY_NONE; // do not reset penelty kicks when switching to playing
      else
        timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime(); // 30 secs for penalty kick start at switch to playing
      if (gameInfo.competitionPhase == COMPETITION_PHASE_PLAYOFF || !timeWhenHalfStarted)
        timeWhenHalfStarted = SystemCall::getCurrentSystemTime() - (durationOfHalf - gameInfo.secsRemaining) * 1000;
    }
    else
    {
      gameInfo.state = STATE_PLAYING;
      gameInfo.secsRemaining = durationOfPS;
      timeWhenHalfStarted = SystemCall::getCurrentSystemTime() - (durationOfPS - gameInfo.secsRemaining) * 1000;
    }
    return true;
  }
  else if (command == "finished")
  {
    gameInfo.state = STATE_FINISHED;
    return true;
  }
  else if (command == "kickOffBlue")
  {
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if (command == "kickOffRed")
  {
    gameInfo.kickingTeam = 2;
    return true;
  }
  else if (command == "kickInForRed")
  {
    placeBallAfterLeavingField(KICK_IN_FOR_RED);
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 2;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "kickInForBlue")
  {
    placeBallAfterLeavingField(KICK_IN_FOR_BLUE);
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 1;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "cornerKickForRed")
  {
    placeBallAfterLeavingField(CORNER_KICK_FOR_RED);
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 2;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "cornerKickForBlue")
  {
    placeBallAfterLeavingField(CORNER_KICK_FOR_BLUE);
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 1;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "pushingFreeKickForRed")
  {
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 2;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "pushingFreeKickForBlue")
  {
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 1;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "goalFreeKickForRed")
  {
    placeBallAfterLeavingField(GOAL_FREE_KICK_FOR_RED);
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    gameInfo.kickingTeam = 2;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "goalFreeKickForBlue")
  {
    placeBallAfterLeavingField(GOAL_FREE_KICK_FOR_BLUE);
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    gameInfo.kickingTeam = 1;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "penaltyKickForRed")
  {
    float penaltyCrossX = fieldDimensions.xPosOpponentPenaltyMark;
    SimulatedRobot::moveBall(Vector3f(penaltyCrossX, 0, 100.f), true);
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.kickingTeam = 2;
    VERIFY(handleGlobalCommand("ready"));
    return true;
  }
  else if (command == "penaltyKickForBlue")
  {
    float penaltyCrossX = fieldDimensions.xPosOwnPenaltyMark;
    SimulatedRobot::moveBall(Vector3f(penaltyCrossX, 0, 100.f), true);
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.kickingTeam = 1;
    VERIFY(handleGlobalCommand("ready"));
    return true;
  }
  else if (command == "setPlayCompleted")
  {
    gameInfo.setPlay = SET_PLAY_NONE;
    timeWhenSetPlayStarted = -1;
    return true;
  }
  else if (command == "gamePlayoff")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_PLAYOFF;
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
    return true;
  }
  else if (command == "gameRoundRobin")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
    return true;
  }
  return false;
}

bool GameController::handleGlobalConsole(In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleGlobalCommand(command);
}

bool GameController::handleRobotCommand(int robot, const std::string& command)
{
  Robot& r = robots[robot];
  RoboCup::RobotInfo& tr = teamInfos[robot * 2 / numOfRobots].players[robot % (numOfRobots / 2)];
  for (int i = 0; i < numOfPenalties; ++i)
    if (command == getName((Penalty)i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (uint8_t)i;
      tr.penalty = r.info.penalty;
      tr.secsTillUnpenalised = 45;
      if (i)
      {
        r.timeWhenPenalized = SystemCall::getCurrentSystemTime();
        if (automatic)
          placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
      }
      else if (automatic)
      {
        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(robot,
            fieldDimensions.xPosOpponentPenaltyMark,
            ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline - safeDistance : fieldDimensions.yPosLeftSideline + safeDistance,
            ballPos.y() >= 0 ? pi_2 : -pi_2);
      }
      return true;
    }
  return false;
}

bool GameController::handleRobotConsole(int robot, In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleRobotCommand(robot, command);
}

void GameController::placeForPenalty(int robot, float x, float y, float rotation)
{
  Robot& r = robots[robot];
  ASSERT(r.simulatedRobot);
  Vector2f newPos(robot < numOfRobots / 2 ? x : -x, y);
  for (;;)
  {
    int j = 0;
    while (j < numOfRobots && (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).norm() >= 300))
      ++j;
    if (j == numOfRobots)
    {
      r.lastPose = Pose2f(rotation, newPos);
      r.simulatedRobot->moveRobot(Vector3f(newPos.x(), newPos.y(), dropHeight), Vector3f(0, 0, rotation), true);
      break;
    }
    else
      newPos.x() += newPos.x() < 0 ? -400 : 400;
  }
}

/**
 * \brief Returns the number of the team a robot is playing for.
 * 
 * The global array "robots" contains all robots on the field. Its length is equal to the number of players both
 * teams combined can have. The first half of the "robots" array contains the robots of team 1 the other half
 * the robots of team 2. 
 * Therefore a robot with an index lower than "numOfRobots / 2"  belongs to team 1 and all other robots to team 2.
 * 
 * \param robotNum The index of the robot in the "robots" array.
 * 
 * \return 1 if the robot plays for team 1 (blue) or 2 if the robot plays for team 2 (red).
 */
int GameController::getTeamNumberOfRobot(int robotNum)
{
  return (robotNum < (numOfRobots / 2)) ? 1 : 2;
}

bool GameController::inOwnPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if (r.lastPose.translation.y() < fieldDimensions.yPosRightPenaltyArea || r.lastPose.translation.y() > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if (robot < numOfRobots / 2)
    return r.lastPose.translation.x() >= fieldDimensions.xPosOpponentPenaltyArea
        && (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGroundline
            || (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGoal && r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal
                && r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x() <= fieldDimensions.xPosOwnPenaltyArea
        && (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGroundline
            || (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGoal && r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal
                && r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
}

bool GameController::inOpponentPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if (r.lastPose.translation.y() < fieldDimensions.yPosRightPenaltyArea || r.lastPose.translation.y() > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if (robot < numOfRobots / 2)
    return r.lastPose.translation.x() <= fieldDimensions.xPosOwnPenaltyArea
        && (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGroundline
            || (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGoal && r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal
                && r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x() >= fieldDimensions.xPosOpponentPenaltyArea
        && (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGroundline
            || (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGoal && r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal
                && r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  r.manuallyPlaced = r.simulatedRobot
      && (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline || r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline
          || (robot < numOfRobots / 2 && (r.lastPose.translation.x() < footLength || r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline))
          || (robot >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength || r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
  if (r.manuallyPlaced)
    r.lastPose = robot < numOfRobots / 2 ? Pose2f(-pi, fieldDimensions.xPosOpponentGroundline - safeDistance, 0) : Pose2f(0, fieldDimensions.xPosOwnGroundline + safeDistance, 0);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2f* poses)
{
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if (i != robot && robots[i].simulatedRobot)
    {
      const Robot& r2 = robots[i];
      float minDistance = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for (int j = 0; j < numOfFieldPlayers; ++j)
      {
        const Pose2f& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).norm();
        if (!occupied[j] && distance < minDistance)
        {
          minDistance = distance;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }

  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while (i < numOfFieldPlayers && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] = {
      {Pose2f(0, -fieldDimensions.centerCircleRadius - footLength, 0),
          Pose2f(0, fieldDimensions.xPosOwnPenaltyMark, fieldDimensions.yPosRightGoal),
          Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftPenaltyArea),
          Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightPenaltyArea)},
      {Pose2f(-pi, fieldDimensions.centerCircleRadius + footLength, 0),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosLeftGoal),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftPenaltyArea),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightPenaltyArea)}};

  // Move all field players that are not in their own half.
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot
        && (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline || r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline
            || (i < numOfRobots / 2 && (r.lastPose.translation.x() < footLength || r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline))
            || (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength || r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if (r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }

  freePenaltyArea(minRobot, poses[minRobot < numOfRobots / 2 ? 1 : 0]);
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] = {
      {Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
          Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
          Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
          Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)},
      {Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightGoal / 2.f),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
          Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)}};

  // Move all field players that are not in their own half or in the center circle.
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot
        && (r.lastPose.translation.norm() < fieldDimensions.centerCircleRadius + footLength || r.lastPose.translation.y() < fieldDimensions.yPosRightSideline
            || r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline
            || (i < numOfRobots / 2 && (r.lastPose.translation.x() < footLength || r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline))
            || (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength || r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if (r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }

  freePenaltyArea(minRobot, poses[minRobot < numOfRobots / 2 ? 1 : 0]);
}

void GameController::freePenaltyArea(int minRobot, const Pose2f* poses)
{
  // Count robots in penalty area and determine the one that is
  // furthest away from the field center.
  int numOfRobotsInOwnPenaltyArea = 0;
  float maxDistance = -1.f;
  int indexOfMaxDistance = 0;
  for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if (inOwnPenaltyArea(i))
    {
      ++numOfRobotsInOwnPenaltyArea;
      float distance = robots[i].lastPose.translation.squaredNorm();
      if (distance > maxDistance)
      {
        maxDistance = distance;
        indexOfMaxDistance = i;
      }
    }

  if (numOfRobotsInOwnPenaltyArea > 1)
  {
    // Move all remaining robots that are in the penalty area away
    for (int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
      if (inOwnPenaltyArea(i) && i != indexOfMaxDistance)
      {
        robots[i].manuallyPlaced = true;
        placeFromSet(i, minRobot, poses);
      }
  }
}

void GameController::executePlacement()
{
  for (int i = 0; i < numOfRobots; ++i)
  {
    const Robot& r = robots[i];
    if (r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3f(r.lastPose.translation.x(), r.lastPose.translation.y(), dropHeight), Vector3f(0, 0, r.lastPose.rotation), true);
  }
}

void GameController::referee()
{
  for (int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if (r.info.penalty)
    {
      r.info.secsTillUnpenalised = (uint8_t)(std::max(int(45 - SystemCall::getTimeSince(r.timeWhenPenalized) / 1000), 0));
      RoboCup::RobotInfo& tr = teamInfos[i * 2 / numOfRobots].players[i % (numOfRobots / 2)];
      tr.secsTillUnpenalised = r.info.secsTillUnpenalised;

      if (automatic && r.info.secsTillUnpenalised <= 0)
      {
        r.info.penalty = PENALTY_NONE;
        tr.penalty = PENALTY_NONE;

        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark, ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline, ballPos.y() >= 0 ? pi_2 : -pi_2);
      }
    }
  }

  if (automatic)
  {
    SYNC;
    int maxTimeInReady = (gameInfo.setPlay == SET_PLAY_PENALTY_KICK) ? 30000 : 45000;
    switch (gameInfo.state)
    {
    case STATE_READY:
      if (SystemCall::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if (SystemCall::getTimeSince(timeWhenStateBegan) >= maxTimeInReady || (timeWhenLastRobotMoved && SystemCall::getTimeSince(timeWhenLastRobotMoved) > 5000))
        handleGlobalCommand("set");
      break;

    case STATE_PLAYING:
      checkForSetPlayCompletion();
      switch (updateBall())
      {
      case GOAL_BY_BLUE:
        std::cout << "Goal by blue" << std::endl;
        ++teamInfos[TEAM_BLUE].score;
        VERIFY(handleGlobalCommand("kickOffRed"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case GOAL_BY_RED:
        std::cout << "Goal by red" << std::endl;
        ++teamInfos[TEAM_RED].score;
        VERIFY(handleGlobalCommand("kickOffBlue"));
        VERIFY(handleGlobalCommand("ready"));
        break;
      case KICK_IN_FOR_RED:
        VERIFY(handleGlobalCommand("kickInForRed"));
        break;
      case KICK_IN_FOR_BLUE:
        VERIFY(handleGlobalCommand("kickInForBlue"));
        break;
      case CORNER_KICK_FOR_RED:
        VERIFY(handleGlobalCommand("cornerKickForRed"));
        break;
      case CORNER_KICK_FOR_BLUE:
        VERIFY(handleGlobalCommand("cornerKickForBlue"));
        break;
      case GOAL_FREE_KICK_FOR_RED:
        VERIFY(handleGlobalCommand("goalFreeKickForRed"));
        break;
      case GOAL_FREE_KICK_FOR_BLUE:
        VERIFY(handleGlobalCommand("goalFreeKickForBlue"));
        break;
      case NONE:
        break;
      }
    }
  }
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = NONE;
  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  if (!fieldDimensions.isBallInsideField(ballPos))
  {
    if (std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
    {
      result = ballPos.x() > fieldDimensions.xPosOpponentGroundline ? GOAL_BY_RED : GOAL_BY_BLUE;
    }
    else
    {
      /*
      To distinguish between the teams they will be called red and blue.
      The blue teams groundline is defined by fieldDimensions.xPosOpponentGroundline and wears yellow jerseys in the simulator.
      The red teams groundline is defined by fieldDimensions.xPosOwnGroundline and wears blue(!) jerseys in the simulator.
      */
      bool ballOverRedGroundline = ballPos.x() < fieldDimensions.xPosOwnGroundline;
      bool ballOverBlueGroundline = ballPos.x() > fieldDimensions.xPosOpponentGroundline;
      bool ballOverLeftSideline = ballPos.y() > fieldDimensions.yPosLeftSideline;
      bool ballOverRightSideline = ballPos.y() < fieldDimensions.yPosRightSideline;
      //bool ballOnLeftFieldHalf = ballPos.y() >= 0;
      bool lastTouchedByRed = lastBallContactPose.rotation == 0; // the rotation encodes the team that last touched the ball (0=red; pi=blue)
      bool lastTouchedByBlue = lastBallContactPose.rotation == pi;
      bool cornerKickForRed = ballOverBlueGroundline && lastTouchedByBlue;
      bool cornerKickForBlue = ballOverRedGroundline && lastTouchedByRed;
      bool goalFreeKickForRed = ballOverRedGroundline && lastTouchedByBlue;
      bool goalFreeKickForBlue = ballOverBlueGroundline && lastTouchedByRed;

      if (ballOverLeftSideline || ballOverRightSideline) // ball went over sidelines -> kick-in
      {
        //newXPos = ballPos.x();
        //newYPos = ballOverLeftSideline ? fieldDimensions.yPosLeftSideline : fieldDimensions.yPosRightSideline;
        //SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true); // move ball to position where it left the field
        result = lastTouchedByRed ? KICK_IN_FOR_BLUE : KICK_IN_FOR_RED;
      }
      else
      {
        if (cornerKickForRed)
        {
          result = CORNER_KICK_FOR_RED;
        }
        else if (cornerKickForBlue)
        {
          //newXPos = fieldDimensions.xPosOwnGroundline;
          //newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftSideline : fieldDimensions.yPosRightSideline;
          //SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true); // move ball to position where it left the field
          result = CORNER_KICK_FOR_BLUE;
        }
        else if (goalFreeKickForRed)
        {
          //newXPos = fieldDimensions.xPosOwnPenaltyMark;
          //newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftPenaltyArea : fieldDimensions.yPosRightPenaltyArea;
          //SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true); // move ball to position where it left the field
          result = GOAL_FREE_KICK_FOR_RED;
        }
        else if (goalFreeKickForBlue)
        {
          //newXPos = fieldDimensions.xPosOpponentPenaltyMark;
          //newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftPenaltyArea : fieldDimensions.yPosRightPenaltyArea;
          //SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true); // move ball to position where it left the field
          result = GOAL_FREE_KICK_FOR_BLUE;
        }
      }
    }
  }
  return result;
}

void GameController::placeBallAfterLeavingField(BallOut typeOfBallout)
{
  /*
  To distinguish between the teams they will be called red and blue.
  The blue teams groundline is defined by fieldDimensions.xPosOpponentGroundline and wears yellow jerseys in the simulator.
  The red teams groundline is defined by fieldDimensions.xPosOwnGroundline and wears blue(!) jerseys in the simulator.
  */
  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  bool ballOnLeftFieldHalf = ballPos.y() >= 0;

  float newXPos;
  float newYPos;
  switch (typeOfBallout)
  {
  case KICK_IN_FOR_BLUE:
  case KICK_IN_FOR_RED:
    newXPos = ballPos.x();
    newYPos = ballPos.y() >= 0 ? fieldDimensions.yPosLeftSideline : fieldDimensions.yPosRightSideline;
    SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
    break;
  case CORNER_KICK_FOR_RED:
    newXPos = fieldDimensions.xPosOpponentGroundline;
    newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftSideline : fieldDimensions.yPosRightSideline;
    SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
    break;
  case CORNER_KICK_FOR_BLUE:
    newXPos = fieldDimensions.xPosOwnGroundline;
    newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftSideline : fieldDimensions.yPosRightSideline;
    SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
    break;
  case GOAL_FREE_KICK_FOR_RED:
    newXPos = fieldDimensions.xPosOwnGoalArea;
    newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftGoalArea : fieldDimensions.yPosRightGoalArea;
    SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
    break;
  case GOAL_FREE_KICK_FOR_BLUE:
    newXPos = fieldDimensions.xPosOpponentGoalArea;
    newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftGoalArea : fieldDimensions.yPosRightGoalArea;
    SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
    break;
  case GOAL_BY_RED:
  case GOAL_BY_BLUE:
    SimulatedRobot::moveBall(Vector3f(-3200.f, 0.f, 50.f), true);
    break;
  default:
    break;
  }
}


void GameController::checkForSetPlayCompletion()
{
  bool setPlayActive = (gameInfo.setPlay != SET_PLAY_NONE);
  if (setPlayActive)
  {
    bool ballContactAfterSetPlayStarted = (timeWhenSetPlayStarted < timeOfLastBallContact); // has the ball been touched after the set play was called?
    uint8_t lastBallContactTeam = lastBallContactPose.rotation == 0 ? 2 : 1;

    bool touchedByActiveTeam = (lastBallContactTeam == gameInfo.kickingTeam);
    int timeSinceSetPlayStarted = SystemCall::getTimeSince(timeWhenSetPlayStarted);
    bool timePassedOnSetPlay = (timeSinceSetPlayStarted > 30 * 1000); // set play is completed after ball was not touched for 30 seconds
    if ((ballContactAfterSetPlayStarted && touchedByActiveTeam) || (timePassedOnSetPlay))
    {
      VERIFY(handleGlobalCommand("setPlayCompleted"));
    }
  }
}

/**
 * \brief Executes all necessary actions when game switches from ready to set during a penalty kick.
 * 
 * This includes 
 *    - placing the ball on the correct penalty mark 
 *    - removinng all illegaly positioned players from the penalty area
 */
void GameController::handlePenaltyKickSet()
{
  gameInfo.state = STATE_SET;
  float penaltyCrossX = gameInfo.kickingTeam == 1 // team 1 = blue; team 2 = red
      ? fieldDimensions.xPosOwnPenaltyMark
      : fieldDimensions.xPosOpponentPenaltyMark;
  SimulatedRobot::moveBall(Vector3f(penaltyCrossX, 0, 100.f), true);
  removeIllegalPlayersFromPenaltyArea(Vector2f(penaltyCrossX, 0));
  timeWhenStateBegan = SystemCall::getCurrentSystemTime();
}

/**
 * \brief Returns the player of a team that is closes to a given ballPosition,
 * 
 * This is used to determine which robot of the kicking team is supposed to execute a penalty kick and
 * therefore allowed in the opponents penalty area.
 * 
 * \param ballPosition The position of the ball to use as a reference point.
 * 
 * \return The index of the striking player in the robots array.
 */
int GameController::findPlayerNumberOfPenaltyKickStriker(Vector2f ballPosition)
{
  float minDistance = std::numeric_limits<float>::max();
  int strikerNum = numOfRobots + 1;
  for (int i = 0; i < numOfRobots; ++i)
  {
    if (robots[i].simulatedRobot)
    {
      int robotTeamNumber = getTeamNumberOfRobot(i);
      if (robotTeamNumber == gameInfo.kickingTeam)
      {
        Vector2f robotPosition = robots[i].lastPose.translation;
        float distRobotToBall = (ballPosition - robotPosition).norm();
        if (distRobotToBall < minDistance)
        {
          minDistance = distRobotToBall;
          strikerNum = i;
        }
      }
    }
  }
  return strikerNum;
}

/**
 * \brief Finds the number of the goalie of the team defending a set play.
 * 
 * \return The index of the goalkeeper in the robots array. -1 if no set play is active.
 */
int GameController::findGoalieOfDefendingTeam()
{
  if (gameInfo.setPlay == SET_PLAY_NONE)
    return -1;
  int defendingTeamNumber = 3 - gameInfo.kickingTeam;
  // the first half of the robots is part of team 1 the other half is part of team 2
  // the robot with the lowest number in a team is the goalkeeper
  int firstRobotOfDefendingTeam = defendingTeamNumber == 1 ? 0 : numOfRobots / 2;
  return firstRobotOfDefendingTeam;
}

/**
 * \brief Determines if a robot is placed on its own groundlinde between the posts of its own goal.
 * 
 * \param robotNum The index of the robot in the robots array.
 * 
 * \return True if the feet of the robot touch the part of its teams groundline between the posts of its own goal.
 *          False otherwise.
 */
bool GameController::isRobotPlacedOnItsGoalLine(int robotNum)
{
  int robotTeamNum = getTeamNumberOfRobot(robotNum);
  float groundLineX = (robotTeamNum == 1) ? fieldDimensions.xPosOpponentGroundline : fieldDimensions.xPosOwnGroundline;
  Vector2f groundLineStart(groundLineX, fieldDimensions.yPosLeftGoal);
  Vector2f groundLineDirection(0.f, 2 * fieldDimensions.yPosRightGoal);
  Geometry::Line goalLine(groundLineStart, groundLineDirection);
  Vector2f robotPosition = robots[robotNum].lastPose.translation;
  float dRobotToLine = Geometry::getDistanceToEdge(goalLine, robotPosition);
  return (dRobotToLine <= 200.f);
}

/**
 * \brief Repositions all players that are illegaly placed in the penalty area during a penalty kick.
 * 
 * The only robots allowed in the penalty area are:
 *  - The striker of the attacking team
 *  - The goalkeeper of the defending team, IF it is positioned on its own groundline
 * All other robots will be placed in penalty positions (on the sidelines), but will not actually get a penalty.
 * 
 * \param ballPosition The position of the ball during the penalty kick.
 */
void GameController::removeIllegalPlayersFromPenaltyArea(Vector2f ballPosition)
{
  int strikerNum = findPlayerNumberOfPenaltyKickStriker(ballPosition);
  int goalieNum = findGoalieOfDefendingTeam();
  bool goalieInLegalPosition = isRobotPlacedOnItsGoalLine(goalieNum);
  for (int i = 0; i < numOfRobots; ++i)
  {
    if (robots[i].simulatedRobot)
    {
      int teamNum = getTeamNumberOfRobot(i);
      bool robotIsStriker = (i == strikerNum);
      bool robotIsValidGoalkeeper = (i == goalieNum && goalieInLegalPosition);
      bool robotIsInDefendingPenaltyArea = (teamNum == gameInfo.kickingTeam && inOpponentPenaltyArea(i)) || (teamNum != gameInfo.kickingTeam && inOwnPenaltyArea(i));
      if (!robotIsStriker && !robotIsValidGoalkeeper && robotIsInDefendingPenaltyArea)
      {
        float penaltyPosX = fieldDimensions.xPosOpponentPenaltyMark;
        float penaltyPosY = (ballPosition.y() >= 0.f) ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline;
        float penaltyPosRotation = (penaltyPosY >= 0.f) ? -0.5f * pi : 0.5f * pi;
        placeForPenalty(i, penaltyPosX, penaltyPosY, penaltyPosRotation);
      }
    }
  }
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isBlue(robot) ? pi : 0, SimulatedRobot::getPosition(robot));
  timeOfLastBallContact = SystemCall::getCurrentSystemTime();
}

void GameController::decreaseMessageBudget(int robot)
{
  SYNC;
  uint16_t& budget = teamInfos[robot * 2 / numOfRobots].messageBudget;
  if (budget > 0)
    --budget;
}

void GameController::writeGameInfo(Out& stream)
{
  SYNC;
  if (gameInfo.state == STATE_PLAYING || (gameInfo.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeWhenHalfStarted))
  {
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
      gameInfo.secsRemaining = (uint16_t)(durationOfHalf - SystemCall::getTimeSince(timeWhenHalfStarted) / 1000);
    else
      gameInfo.secsRemaining = (uint16_t)(durationOfPS - SystemCall::getTimeSince(timeWhenHalfStarted) / 1000);
  }
  gameInfo.timeLastPackageReceived = SystemCall::getCurrentSystemTime();
  stream << gameInfo;
}

void GameController::writeTeamInfo(TeamInfo& teamInfo, Out& stream)
{
  SYNC;
  // starting value: 1200 packages or 1680 for 7v7 (see rules and GC 2022)
  if (gameInfo.state == STATE_INITIAL || gameInfo.state == STATE_FINISHED)
    teamInfo.messageBudget = (gameInfo.competitionType == COMPETITION_TYPE_7V7) ? 1680 : 1200;

  teamInfo.teamPort = Global::getSettings().teamPort;

  stream << teamInfo;
}

void GameController::writeOwnTeamInfo(int robot, Out& stream)
{
  writeTeamInfo(teamInfos[robot * 2 / numOfRobots], stream);
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream)
{
  writeTeamInfo(teamInfos[1 - robot * 2 / numOfRobots], stream);
}

void GameController::writeRobotInfo(int robot, Out& stream)
{
  SYNC;
  Robot& r = robots[robot];
  Pose2f pose;
  ASSERT(r.simulatedRobot);
  r.simulatedRobot->getRobotPose(pose);
  if (robot < numOfRobots / 2)
    pose = Pose2f(pi) + pose;
  if ((pose.translation - r.lastPose.translation).norm() > 5 || Angle::normalize(pose.rotation - r.lastPose.rotation) > 0.05)
  {
    timeWhenLastRobotMoved = SystemCall::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::setTeamColors(uint8_t firstTeamColor, uint8_t secondTeamColor)
{
  teamInfos[0].teamNumber = 1;
  teamInfos[0].teamColour = firstTeamColor;
  teamInfos[1].teamNumber = 2;
  teamInfos[1].teamColour = secondTeamColor;
}

void GameController::addCompletion(std::set<std::string>& completion) const
{
  static const char* commands[] = {"initial",
      "ready",
      "set",
      "playing",
      "finished",
      "kickOffBlue",
      "kickOffRed",
      "kickInForRed",
      "kickInForBlue",
      "cornerKickForRed",
      "cornerKickForBlue",
      "goalFreeKickForRed",
      "goalFreeKickForBlue",
      "pushingFreeKickForRed",
      "pushingFreeKickForBlue",
      "penaltyKickForRed",
      "penaltyKickForBlue",
      "setPlayCompleted",
      "gameDropIn",
      "gamePlayoff",
      "gameRoundRobin"};
  const int num = sizeof(commands) / sizeof(commands[0]);
  for (int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  for (int i = 0; i < numOfPenalties; ++i)
    completion.insert(std::string("pr ") + getName((Penalty)i));
}
