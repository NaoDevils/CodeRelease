/**
 * @file AnnotationManager.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "AnnotationManager.h"

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/BehaviorControl/BehaviorData.h"

#include <cstring>
#include <stdarg.h>

AnnotationManager::AnnotationManager()
    : lastGameState(STATE_INITIAL), lastSetPlay(SET_PLAY_NONE), lastPenalty(PENALTY_NONE), lastOwnScore(0), lastOpponentScore(0), lastTransitionToFramework(0.f)
{
  outData.setSize(100000);
}

void AnnotationManager::signalProcessStart()
{
  if (Blackboard::getInstance().exists("GameInfo"))
  {
    const GameInfo& gameInfo = Blackboard::get<GameInfo>();

    if (gameInfo.state != lastGameState)
    {
      addAnnotation();
      outData.out.text << "GameInfo" << gameInfo.getStateAsString() << " state.";
      endAnnotation();
    }
    lastGameState = gameInfo.state;

    if (gameInfo.setPlay != SET_PLAY_NONE && gameInfo.setPlay != lastSetPlay)
    {
      addAnnotation();
      outData.out.text << "GameInfo" << gameInfo.getSetPlayAsString();
      endAnnotation();
    }
    lastSetPlay = gameInfo.setPlay;
  }

  if (Blackboard::getInstance().exists("OwnTeamInfo") && Blackboard::getInstance().exists("OpponentTeamInfo"))
  {
    const OwnTeamInfo& ownTeamInfo = Blackboard::get<OwnTeamInfo>();
    const OpponentTeamInfo& opponentTeamInfo = Blackboard::get<OpponentTeamInfo>();

    if (ownTeamInfo.score != lastOwnScore || opponentTeamInfo.score != lastOpponentScore)
    {
      addAnnotation();
      outData.out.text << "TeamInfo" << ownTeamInfo.score << ":" << opponentTeamInfo.score;
      endAnnotation();
    }

    lastOwnScore = ownTeamInfo.score;
    lastOpponentScore = opponentTeamInfo.score;
  }

  if (Blackboard::getInstance().exists("RobotInfo"))
  {
    const RobotInfo& robotInfo = Blackboard::get<RobotInfo>();

    if (robotInfo.penalty != lastPenalty)
    {
      addAnnotation();
      outData.out.text << "RobotInfo" << robotInfo.getPenaltyAsString();
      endAnnotation();
    }
    lastPenalty = robotInfo.penalty;

    if (lastTransitionToFramework == 0.f && robotInfo.transitionToFramework > 0.f)
    {
      addAnnotation();
      outData.out.text << "RobotInfo"
                       << "Transitioned to Framework";
      endAnnotation();
    }
    else if (lastTransitionToFramework == 1.f && robotInfo.transitionToFramework < 1.f)
    {
      addAnnotation();
      outData.out.text << "RobotInfo"
                       << "Transitioned to NDevilsBase";
      endAnnotation();
    }
    lastTransitionToFramework = robotInfo.transitionToFramework;
  }
}

void AnnotationManager::clear()
{
  outData.clear();
}

void AnnotationManager::addAnnotation()
{
  mutex.lock();
  // compatibility with old logs
  outData.out.bin << (0x80000000 | annotationCounter++);
}

void AnnotationManager::endAnnotation()
{
  outData.out.finishMessage(idAnnotation);
  mutex.unlock();
}

MessageQueue& AnnotationManager::getOut()
{
  return outData;
}
