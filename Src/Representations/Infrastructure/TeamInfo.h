/**
 * @file TeamInfo.h
 * The file declares a struct that encapsulates the structure TeamInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"
#include "Platform/BHAssert.h"

struct TeamInfo : public RoboCup::TeamInfo, public Streamable
{
private:
  using RoboCup::TeamInfo::penaltyShot; // Hide, because it is not streamed.
  using RoboCup::TeamInfo::singleShots; // Hide, because it is not streamed.

public:
  int teamPort = 0;
  bool onLeftSide = true;

  TeamInfo();

  /** Draws the score in the scene view. */
  void draw() const;

  static const char* getColorName(uint8_t color)
  {
    switch (color)
    {
    case TEAM_BLUE:
      return "blue";
    case TEAM_RED:
      return "red";
    case TEAM_YELLOW:
      return "yellow";
    case TEAM_BLACK:
      return "black";
    case TEAM_WHITE:
      return "white";
    case TEAM_GREEN:
      return "green";
    case TEAM_ORANGE:
      return "orange";
    case TEAM_PURPLE:
      return "purple";
    case TEAM_BROWN:
      return "brown";
    case TEAM_GRAY:
      return "gray";
    default:
      ASSERT(false);
      return "";
    }
  }

private:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out);
};

struct OwnTeamInfo : public TeamInfo
{
  OwnTeamInfo();
  void draw() const;
  Streamable& operator=(const Streamable& other) noexcept;
};

struct OpponentTeamInfo : public TeamInfo
{
  OpponentTeamInfo();
  void draw() const;
  Streamable& operator=(const Streamable& other) noexcept;
};
