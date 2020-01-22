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

GameController::GameController() :
timeWhenHalfStarted(0),
timeOfLastDropIn(0),
timeWhenLastRobotMoved(0),
timeWhenStateBegan(0),

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
  if(fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}

bool GameController::handleGlobalCommand(const std::string& command)
{
  if(command == "initial")
  {if (Global::getSettings().gameMode != Settings::penaltyShootout)
    gameInfo.secsRemaining = durationOfHalf;
  else
    gameInfo.secsRemaining = durationOfPS;
    gameInfo.state = STATE_INITIAL;
    timeOfLastDropIn = timeWhenHalfStarted = 0;
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
      gameInfo.secsRemaining = durationOfHalf;
    else
      gameInfo.secsRemaining = durationOfPS;
    return true;
  }
  else if(command == "ready")
  {
    gameInfo.state = STATE_READY;
    for(int i = 0; i < numOfRobots; ++i)
      if(robots[i].info.penalty)
        handleRobotCommand(i, "none");
    timeWhenStateBegan = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if(command == "set")
  {
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
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
  else if(command == "playing")
  {
    if (Global::getSettings().gameMode != Settings::penaltyShootout)
    {
      gameInfo.state = STATE_PLAYING;
      gameInfo.setPlay = SET_PLAY_NONE;
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
  else if(command == "finished")
  {
    gameInfo.state = STATE_FINISHED;
    return true;
  }
  else if(command == "kickOffBlue")
  {
    gameInfo.kickingTeam = 1;
    return true;
  }
  else if(command == "kickOffRed")
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
    gameInfo.setPlay = SET_PLAY_GOAL_FREE_KICK;
    gameInfo.kickingTeam = 2;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "goalFreeKickForBlue")
  {
    placeBallAfterLeavingField(GOAL_FREE_KICK_FOR_BLUE);
	  gameInfo.setPlay = SET_PLAY_GOAL_FREE_KICK;
    gameInfo.kickingTeam = 1;
    timeWhenSetPlayStarted = SystemCall::getCurrentSystemTime();
    return true;
  }
  else if (command == "setPlayCompleted")
  {
    gameInfo.setPlay = SET_PLAY_NONE;
    timeWhenSetPlayStarted = -1;
    return true;
  }
  else if(command == "gamePlayoff")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_PLAYOFF;
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
    return true;
  }
  else if(command == "gameRoundRobin")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
    return true;
  }
  // Removed drop in here, because RoboCupGameControlData v10 has no GAME_DROPIN
  else if(command == "gameMixedTeamRoundRobin")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
    gameInfo.competitionType = COMPETITION_TYPE_MIXEDTEAM;
    return true;
  }
  else if (command == "gameMixedTeamPlayOff")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_PLAYOFF;
    gameInfo.competitionType = COMPETITION_TYPE_MIXEDTEAM;
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
  for(int i = 0; i < numOfPenalties; ++i)
    if(command == getName((Penalty) i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (uint8_t) i;
      tr.penalty = r.info.penalty;
      tr.secsTillUnpenalised = 45;
      if(i)
      {
        r.timeWhenPenalized = SystemCall::getCurrentSystemTime();
        if(automatic)
          placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
                          fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
      }
      else if(automatic)
      {
        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(robot, fieldDimensions.xPosOpponentPenaltyMark,
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
  for(;;)
  {
    int j = 0;
    while(j < numOfRobots &&
      (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).norm() >= 300))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2f(rotation, newPos);
      r.simulatedRobot->moveRobot(Vector3f(newPos.x(), newPos.y(), dropHeight), Vector3f(0, 0, rotation), true);
      break;
    }
    else
      newPos.x() += newPos.x() < 0 ? -400 : 400;
  }
}

bool GameController::inOwnPenaltyArea(int robot) const
{
  const Robot& r = robots[robot];
  if(r.lastPose.translation.y() < fieldDimensions.yPosRightPenaltyArea ||
     r.lastPose.translation.y() > fieldDimensions.yPosLeftPenaltyArea)
    return false;
  else if(robot < numOfRobots / 2)
    return r.lastPose.translation.x() >= fieldDimensions.xPosOpponentPenaltyArea &&
            (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGroundline ||
            (r.lastPose.translation.x() <= fieldDimensions.xPosOpponentGoal &&
             r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
  else
    return r.lastPose.translation.x() <= fieldDimensions.xPosOwnPenaltyArea &&
            (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGroundline ||
            (r.lastPose.translation.x() >= fieldDimensions.xPosOwnGoal &&
             r.lastPose.translation.y() >= fieldDimensions.yPosRightGoal &&
             r.lastPose.translation.y() <= fieldDimensions.yPosLeftGoal));
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  r.manuallyPlaced = r.simulatedRobot && (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                                          r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                                          (robot < numOfRobots / 2 &&  (r.lastPose.translation.x() < footLength ||
                                                                        r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                                          (robot >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                                                                        r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
  if(r.manuallyPlaced)
    r.lastPose = robot < numOfRobots / 2 ? Pose2f(-pi, fieldDimensions.xPosOpponentGroundline - safeDistance, 0)
                                         : Pose2f(0, fieldDimensions.xPosOwnGroundline + safeDistance, 0);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2f* poses)
{
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].simulatedRobot)
    {
      const Robot& r2 = robots[i];
      float minDistance = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfFieldPlayers; ++j)
      {
        const Pose2f& pose = poses[j];
        float distance = (pose.translation - r2.lastPose.translation).norm();
        if(!occupied[j] && distance < minDistance)
        {
          minDistance = distance;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }

  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while(i < numOfFieldPlayers && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0, -fieldDimensions.centerCircleRadius - footLength, 0),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyMark, fieldDimensions.yPosRightGoal),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2f(0, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightPenaltyArea)
    },
    {
      Pose2f(-pi, fieldDimensions.centerCircleRadius + footLength, 0),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, fieldDimensions.yPosLeftGoal),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftPenaltyArea),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightPenaltyArea)
    }
  };

  // Move all field players that are not in their own half.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot &&
                       (r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 &&  (r.lastPose.translation.x() < footLength ||
                                                  r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                                                  r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
      placeFromSet(i, minRobot, poses[i < numOfRobots / 2 ? 1 : 0]);
  }

  freePenaltyArea(minRobot, poses[minRobot < numOfRobots / 2 ? 1 : 0]);
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosLeftGoal / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, fieldDimensions.yPosRightGoal / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };

  // Move all field players that are not in their own half or in the center circle.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    r.manuallyPlaced = r.simulatedRobot &&
                       (r.lastPose.translation.norm() < fieldDimensions.centerCircleRadius + footLength ||
                        r.lastPose.translation.y() < fieldDimensions.yPosRightSideline ||
                        r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline ||
                        (i < numOfRobots / 2 &&  (r.lastPose.translation.x() < footLength ||
                                                  r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundline)) ||
                        (i >= numOfRobots / 2 && (r.lastPose.translation.x() > -footLength ||
                                                  r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundline)));
    if(r.manuallyPlaced)
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
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(inOwnPenaltyArea(i))
    {
      ++numOfRobotsInOwnPenaltyArea;
      float distance = robots[i].lastPose.translation.squaredNorm();
      if(distance > maxDistance)
      {
        maxDistance = distance;
        indexOfMaxDistance = i;
      }
    }

  if(numOfRobotsInOwnPenaltyArea > 1)
  {
    // Move all remaining robots that are in the penalty area away
    for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
      if(inOwnPenaltyArea(i) && i != indexOfMaxDistance)
      {
        robots[i].manuallyPlaced = true;
        placeFromSet(i, minRobot, poses);
      }
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    const Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3f(r.lastPose.translation.x(), r.lastPose.translation.y(), dropHeight),
                                Vector3f(0, 0, r.lastPose.rotation), true);
  }
}

void GameController::referee()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty)
    {
      r.info.secsTillUnpenalised = (uint8_t) (std::max(int(45 - SystemCall::getTimeSince(r.timeWhenPenalized) / 1000), 0));
      RoboCup::RobotInfo& tr = teamInfos[i * 2 / numOfRobots].players[i % (numOfRobots / 2)];
      tr.secsTillUnpenalised = r.info.secsTillUnpenalised;

      if(automatic && r.info.secsTillUnpenalised <= 0)
      {
        r.info.penalty = PENALTY_NONE;
        tr.penalty = PENALTY_NONE;

        ASSERT(r.simulatedRobot);
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0 ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0 ? pi_2 : -pi_2);
      }
    }
  }

  if(automatic)
  {
    SYNC;
    switch(gameInfo.state)
    {
      case STATE_READY:
        if(SystemCall::getTimeSince(timeWhenStateBegan) < 2000)
          timeWhenLastRobotMoved = 0;
        if(SystemCall::getTimeSince(timeWhenStateBegan) >= 45000 ||
           (timeWhenLastRobotMoved && SystemCall::getTimeSince(timeWhenLastRobotMoved) > 5000))
          handleGlobalCommand("set");
        break;

      case STATE_PLAYING:
        switch(updateBall())
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
            VERIFY(handleGlobalCommand("outByBlue"));
            VERIFY(handleGlobalCommand("kickInForRed"));
            break;
          case KICK_IN_FOR_BLUE:
            VERIFY(handleGlobalCommand("outByRed"));
            VERIFY(handleGlobalCommand("kickInForBlue"));
            break;
          case CORNER_KICK_FOR_RED:
            VERIFY(handleGlobalCommand("outByBlue"));
            VERIFY(handleGlobalCommand("cornerKickForRed"));
            break;
          case CORNER_KICK_FOR_BLUE:
            VERIFY(handleGlobalCommand("outByRed"));
            VERIFY(handleGlobalCommand("cornerKickForBlue"));
            break;
          case GOAL_FREE_KICK_FOR_RED:
            VERIFY(handleGlobalCommand("outByBlue"));
            VERIFY(handleGlobalCommand("goalFreeKickForRed"));
            break;
          case GOAL_FREE_KICK_FOR_BLUE:
            VERIFY(handleGlobalCommand("outByRed"));
            VERIFY(handleGlobalCommand("goalFreeKickForBlue"));
            break;
          case OUT_BY_BLUE:
            VERIFY(handleGlobalCommand("outByBlue"));
            break;
          case OUT_BY_RED:
            VERIFY(handleGlobalCommand("outByRed"));
            break;
          case NONE:
            break;
        }
    }
    checkForSetPlayCompletion();
  }
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = NONE;
  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  if(!fieldDimensions.isBallInsideField(ballPos))
  {
    if(std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
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
      newXPos = fieldDimensions.xPosOwnPenaltyMark;
      newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftPenaltyArea : fieldDimensions.yPosRightPenaltyArea;
      SimulatedRobot::moveBall(Vector3f(newXPos, newYPos, 100.f), true);
      break;
    case GOAL_FREE_KICK_FOR_BLUE:
      newXPos = fieldDimensions.xPosOpponentPenaltyMark;
      newYPos = ballOnLeftFieldHalf ? fieldDimensions.yPosLeftPenaltyArea : fieldDimensions.yPosRightPenaltyArea;
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
  bool setPlayActive = gameInfo.setPlay != SET_PLAY_NONE;
  if (setPlayActive) {
    bool ballContactAfterSetPlayStarted = (timeWhenSetPlayStarted < timeOfLastBallContact); // has the ball been touched after the set play was called?
    uint8_t lastBallContactTeam = lastBallContactPose.rotation == 0 ? 2 : 1;

    bool touchedByActiveTeam = (lastBallContactTeam == gameInfo.kickingTeam);
    int timeSinceSetPlayStarted = SystemCall::getTimeSince(timeWhenSetPlayStarted);
    bool timePassedOnSetPlay = (timeSinceSetPlayStarted > 30 * 1000); // set play is completed after ball was not touched for 30 seconds
    if ((ballContactAfterSetPlayStarted && touchedByActiveTeam)
      || (timePassedOnSetPlay))
    {
      VERIFY(handleGlobalCommand("setPlayCompleted"));
    }
  }
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isBlue(robot) ? pi : 0, SimulatedRobot::getPosition(robot));
  timeOfLastBallContact = SystemCall::getCurrentSystemTime();
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

void GameController::writeOwnTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[robot * 2 / numOfRobots];
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[1 - robot * 2 / numOfRobots];
}

void GameController::writeRobotInfo(int robot, Out& stream)
{
  SYNC;
  Robot& r = robots[robot];
  Pose2f pose;
  ASSERT(r.simulatedRobot);
  r.simulatedRobot->getRobotPose(pose);
  if(robot < numOfRobots / 2)
    pose = Pose2f(pi) + pose;
  if((pose.translation - r.lastPose.translation).norm() > 5 ||
     Angle::normalize(pose.rotation - r.lastPose.rotation) > 0.05)
  {
    timeWhenLastRobotMoved = SystemCall::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::addCompletion(std::set<std::string>& completion) const
{
  static const char* commands[] =
  {
    "initial",
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
    "setPlayCompleted",
    "gameDropIn"
    "gamePlayoff",
    "gameRoundRobin"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  for(int i = 0; i < numOfPenalties; ++i)
    completion.insert(std::string("pr ") + getName((Penalty) i));
}
