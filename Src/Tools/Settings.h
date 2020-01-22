/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Configuration/RobotConfig.h"

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings,
{
public:
  ENUM(TeamColor,
  {,
    blue,
    red,
    yellow,
    black,
  });
  
  ENUM(GameMode,
  { ,
    mixedTeam,
    preliminary,
    playOff,
    penaltyShootout,
    demoIRF,
  });
  
  std::string robotName; /**< The name of this robot. */
  std::string bodyName; /**< The name of this robot's body. */
  
  static bool recover; /**< Start directly without the pre-initial state. */

  static constexpr int highestValidPlayerNumber = 6; /**< No player can have a number greater than this */
  static constexpr int lowestValidPlayerNumber = 1;  /**< No player can have a number smaller than this */
  bool isGoalkeeper;            /**< Is this robot the goaliekeeper? */
  
  friend class Framework; /**< To access loaded. */


  Settings();

  static bool loadingSucceeded() { return loaded; }
private:
  static Settings settings; /**< The master settings instance. */
  static bool loaded; /**< The load() of the master settings instance was called or not. */

  /**
   * Constructor for the master settings instance.
   */
  Settings(bool master);


  /**
   * Assignment operator
   * @param other The other settings that is assigned to this one
   * @return A reference to this object after the assignment.
   */
  Settings& operator=(const Settings& other)
  {
    naoVersion = other.naoVersion;
    teamNumber = other.teamNumber;
    teamColor = other.teamColor;
    playerNumber = other.playerNumber;
    location = other.location.c_str(); // avoid copy-on-write
    teamPort = other.teamPort;
    robotName = other.robotName.c_str(); // avoid copy-on-write
    bodyName = other.bodyName.c_str(); // avoid copy-on-write
    gameMode = other.gameMode;
    return *this;
  }
  
  /**
   * The function loads the settings from disk.
   * @return Whether the settings were loaded successfully.
   */
  bool load();

public:  
  ,
  ((RobotConfig) NaoVersion) naoVersion,
  (int)(0) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor)(blue) teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColor instead. */
  (int)(0) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string)("Default") location, /**< The name of the location. */
  (int)(0) teamPort, /**< The UDP port our team uses for team communication. */
  (GameMode)(preliminary) gameMode,
});
