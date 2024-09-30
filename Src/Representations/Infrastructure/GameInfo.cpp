/**
 * @file GameInfo.cpp
 * The file implements a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "GameInfo.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"

GameInfo::GameInfo()
{
  memset((RoboCup::RoboCupGameControlData*)this, 0, sizeof(RoboCup::RoboCupGameControlData));
}

static void drawDigit(int digit, const Vector3f& pos, float size, const ColorRGBA& color)
{
  static const Vector3f points[8] = {Vector3f(1, 0, 1), Vector3f(1, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 1), Vector3f(0, 0, 2), Vector3f(1, 0, 2), Vector3f(1, 0, 1), Vector3f(0, 0, 1)};
  static const unsigned char digits[11] = {0x3f, 0x0c, 0x76, 0x5e, 0x4d, 0x5b, 0x7b, 0x0e, 0x7f, 0x5f, 0x40};
  digit = digits[std::abs(digit)];
  for (int i = 0; i < 7; ++i)
    if (digit & (1 << i))
    {
      Vector3f from = pos - points[i] * size;
      Vector3f to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, color);
    }
}

void GameInfo::draw() const
{
  DEBUG_DRAWING3D("representation:GameInfo", "field")
  {
    const int mins = std::abs((int)(short)secsRemaining) / 60;
    const int secs = std::abs((int)(short)secsRemaining) % 60;
    const ColorRGBA color = state == STATE_READY ? ColorRGBA::blue : state == STATE_SET ? ColorRGBA::yellow : state == STATE_PLAYING ? ColorRGBA::darkgreen : ColorRGBA::black;

    if (secsRemaining < 0)
      drawDigit(10, Vector3f(-620, 3500, 1000), 200, color); // -
    drawDigit(mins / 10, Vector3f(-350, 3500, 1000), 200, color);
    drawDigit(mins % 10, Vector3f(-80, 3500, 1000), 200, color);
    drawDigit(secs / 10, Vector3f(280, 3500, 1000), 200, color);
    drawDigit(secs % 10, Vector3f(550, 3500, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, 3500, 890, 0, 3500, 910, 3, color);
    LINE3D("representation:GameInfo", 0, 3500, 690, 0, 3500, 710, 3, color);
  }

  DEBUG_DRAWING("representation:GameInfo", "drawingOnField")
  {
    DRAWTEXT("representation:GameInfo", -5000, -3200, 200, ColorRGBA::white, "Time remaining: " << (int)(secsRemaining / 60) << ":" << (secsRemaining % 60));
    DRAWTEXT("representation:GameInfo", -5000, -3400, 200, ColorRGBA::white, (firstHalf ? "First" : "Second") << " half");
    DRAWTEXT("representation:GameInfo", -3500, -3400, 180, ColorRGBA::white, "State: " << getStateAsString());
  }
}

std::string GameInfo::getStateAsString() const
{
  switch (state)
  {
  case STATE_INITIAL:
    return "Initial";
  case STATE_STANDBY:
    return "Standby";
  case STATE_READY:
    return "Ready";
  case STATE_SET:
    return "Set";
  case STATE_PLAYING:
    return "Playing";
  case STATE_FINISHED:
    return "Finished";
  default:
    return "Unknown";
  }
}

std::string GameInfo::getSetPlayAsString() const
{
  switch (setPlay)
  {
  case SET_PLAY_GOAL_KICK:
    return "Goal Free Kick";
  case SET_PLAY_PUSHING_FREE_KICK:
    return "Pushing Free Kick";
  case SET_PLAY_CORNER_KICK:
    return "Corner Kick";
  case SET_PLAY_KICK_IN:
    return "Kick In";
  case SET_PLAY_PENALTY_KICK:
    return "Penalty Kick";
  default:
    return "";
  }
}

Streamable& GameInfo::operator=(const Streamable& other) noexcept
{
  return *this = dynamic_cast<const GameInfo&>(other);
}

bool GameInfo::isSetPlay() const
{
  return setPlay != SET_PLAY_NONE;
}

void GameInfo::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(packetNumber); // number incremented with each packet sent (with wraparound)
  STREAM(playersPerTeam); // the number of players on a team
  STREAM(competitionPhase); // phase of the game (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType); // type of game (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_GENERAL_PENALTY_KICK, ..)
  STREAM(gamePhase); // game phase - (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // special set phases within game i.e. setting up for free kick (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, ..)
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickingTeam); // team number of team with kick off/free kick off
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime); // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  STREAM(timeLastPackageReceived); // used to decide whether a gameController is running
  STREAM(timeFirstReadyState);
  STREAM(oppTeamNumber);
  STREAM(controllerConnected);
  STREAM_REGISTER_FINISH;
}
